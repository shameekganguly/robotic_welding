/* Control robot in 6-DOF EE position, orientation */
/* States that the robot control goes through:
	(optional) CALIBRATE_FORCE_TORQUE_SENSOR
	PRE_HAPTIC_CONTROL
	WAIT_FOR_HAPTIC_DEVICE
	HAPTIC_CONTROL
	COMPUTE_TRAJECTORY
	TRAJECTORY
	FAILURE
*/

#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>
#include "demo3.h"
#include <json/json.h>
#include <fstream>
#include <streambuf>
#include "demo2/Logger.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

#include <iostream>
#include <string>

using namespace std;

// Redis keys
const string RKEY_IIWA_JOINT_POS = "sai2::KUKA_IIWA::sensors::q"; // read from sim/ driver
const string RKEY_IIWA_JOINT_VEL = "sai2::KUKA_IIWA::sensors::dq"; // read from sim/ driver
const string RKEY_IIWA_FORCE = "sai2::optoforceSensor::6Dsensor::force"; // read from OptoForce driver/ simulator
// const string RKEY_IIWA_FORCE = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::force_sensor::force";
const string RKEY_IIWA_DES_POS = "sai2::iiwaForceControl::iiwaBot::haptic::xp_des"; // read from haptic device
const string RKEY_IIWA_DES_ORI = "sai2::iiwaForceControl::iiwaBot::haptic::xr_des"; // read from haptic device
const string RKEY_IIWA_DES_VEL = "sai2::iiwaForceControl::iiwaBot::haptic::dx_des"; // Not used currently
const string RKEY_IIWA_DES_TORQUE = "sai2::KUKA_IIWA::actuators::fgc"; // write to sim/ driver
const string FGC_ENABLE_KEY  = "sai2::iiwaForceControl::iiwaBot::fgc_command_enabled"; // ? signal read from driver
const string RKEY_HAPTIC_READY = "sai2::iiwaForceControl::iiwaBot::haptic::device_ready"; // read from haptic device to indicate haptic device ready
const string RKEY_IIWA_READY = "sai2::iiwaForceControl::iiwaBot::haptic::robot_ready"; // sent to haptic device to indicate robot ready
const string RKEY_IIWA_CTRL_STATUS = "sai2::iiwaForceControl::iiwaBot::ctrl_status"; // controller state machine status
const string RKEY_IIWA_CTRL_STATE_INPUT = "sai2::iiwaForceControl::iiwaBot::ctrl_state_des"; // user input to change controller state
const string RKEY_IIWA_CTRL_FORCE_CALIB = "sai2::iiwaForceControl::iiwaBot::ctrl::calib_force"; // controller output calibrted force
const string RKEY_IIWA_CURR_POS = "sai2::iiwaForceControl::iiwaBot::ctrl::xp_curr"; // diagnostic


// models
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";
// calibration files
const string sensor_calib_file = "sensor_calib.json";
const string joint_estimation_file = "result.json";

// global constants
const Eigen::Vector3d GRAVITY_BASE (0.0, 0.0, -9.81);

#include <signal.h>
bool runloop = true;
unsigned long long controller_counter = 0;
void sighandler(int sig) { runloop = false;}

void jsonArrToVector3d (const Json::Value& jarray, Eigen::Vector3d& ret_vec) {
	ret_vec << jarray[0].asDouble(), jarray[1].asDouble(), jarray[2].asDouble();
}

// load json weld joint data
void loadJson(
	const string& fname,
	Eigen::Vector3d& joint_start,
	Eigen::Vector3d& joint_end,
	Eigen::Vector3d& normal1,
	Eigen::Vector3d& normal2
) {
	Json::Value root;
	std::ifstream t(fname);
	Json::Reader reader;
	reader.parse(t, root);
	jsonArrToVector3d(root["joint_start"], joint_start);
	jsonArrToVector3d(root["joint_end"], joint_end);
	jsonArrToVector3d(root["plane1"]["normal"], normal1);
	jsonArrToVector3d(root["plane2"]["normal"], normal2);
}

// force sensor calibration. NOTE: should only be called when sensor is static
void getSensorBias(
	const Vector6d& raw_force, // in sensor frame
	const Eigen::Matrix3d& ee_ori, // wrt base frame
	Vector6d& sensor_bias // in sensor frame
) {
//	cout << ee_ori << endl;
	// orientation offset from sensor to ee
	Eigen::Matrix3d ori_sensor_ee;
	ori_sensor_ee << cos(FORCE_SENSOR_ROT), -sin(FORCE_SENSOR_ROT), 0.0,
					sin(FORCE_SENSOR_ROT), cos(FORCE_SENSOR_ROT), 0.0,
						0.0,			0.0,				1.0;
	Eigen::Matrix3d ori_sensor_base = ee_ori*ori_sensor_ee;
	Eigen::Vector3d tool_local_com (TOOL_COM);
	Eigen::Vector3d sensor_tool_gravity_moment = tool_local_com.cross(ori_sensor_base.transpose()*GRAVITY_BASE);
	// subtract tool weight and moment from raw_force
	sensor_bias << raw_force.segment<3>(0) - ori_sensor_base.transpose()*TOOL_MASS*GRAVITY_BASE,
				raw_force.segment<3>(3) - sensor_tool_gravity_moment;
	cout << sensor_bias.transpose() << endl;
}

// get calibrated force from raw force.
// NOTE: We only calibrate and return the measured force right now, not the moment
void getCalibratedForce( 
	const Vector6d& raw_force, // in sensor frame
	const Vector6d& sensor_bias, // in sensor frame
	const Eigen::Matrix3d& ee_ori, // wrt base frame
	Vector6d& calib_force // return: calib force in base frame
) {
	// orientation offset from sensor to ee
	Eigen::Matrix3d ori_sensor_ee;
	ori_sensor_ee << cos(FORCE_SENSOR_ROT), -sin(FORCE_SENSOR_ROT), 0.0,
					sin(FORCE_SENSOR_ROT), cos(FORCE_SENSOR_ROT), 0.0,
						0.0,			0.0,				1.0;
	Eigen::Matrix3d ori_sensor_base = ee_ori*ori_sensor_ee;
	// pseudocode:
	// temp1 = sensor_force - bias
	// temp2 = ori_ee_world*ori_sensor_ee*temp1;
	// calib_force = temp2 - gravity_ee;

	// calibrated force
	Vector6d unbiased_force = raw_force - sensor_bias; //TODO: use FORCE_SENSOR_SIGN?
	Eigen::Vector3d tool_local_com (TOOL_COM);
	calib_force.segment<3>(0) = ori_sensor_base*(unbiased_force.segment<3>(0)) - TOOL_MASS*GRAVITY_BASE;
	
	// calibrated moment
	Eigen::Vector3d sensor_tool_gravity_moment = tool_local_com.cross(ori_sensor_base.transpose()*GRAVITY_BASE);
	calib_force.segment<3>(3) = ori_sensor_base*(unbiased_force.segment<3>(3) - TOOL_MASS*sensor_tool_gravity_moment);
}

// sanity check force sensor data. assumes data is gravity compensated and bias is removed.
bool isForceSensorValueGood(Vector6d& force) {
	const double force_threshold = 25; //N of force magnitude
	const double moment_threshold = 4;//Nm, assumes applied at the torch tip
	// first 3 in array is force, next 3 is moment
	bool ret_flag = true;
	if (force.segment<3>(0).norm() > force_threshold) {
		cerr << "ERROR: Force sensor - force magnitude exceeded threshold: " << force_threshold << endl;
		ret_flag = false;
	}
	if (force.segment<3>(3).norm() > moment_threshold) {
		cerr << "ERROR: Force sensor - moment magnitude exceeded threshold: " << moment_threshold << endl;
		ret_flag = false;
	}
	return ret_flag;
}

// check if IIWA joint motion is complete
bool isReachedHomePos(const Eigen::VectorXd& jpos_curr, const Eigen::VectorXd& jpos_des, const Eigen::VectorXd& jvel) {
	// IIWA joint position thresholds 
	Eigen::VectorXd iiwa_pos_thresh(jpos_curr.size());
	iiwa_pos_thresh << 1.5, 1.5, 10.0, 2.0, 2.0, 5.0, 10.0; //degrees 
	iiwa_pos_thresh *= M_PI/180.0;
	// IIWA joint velocity magnitude threshold
	const double iiwa_vel_thresh = 0.003; // rad/s
	Eigen::VectorXd pos_diff_abs = (jpos_curr - jpos_des).array().abs();
	if ((iiwa_pos_thresh - pos_diff_abs).minCoeff() > 0) { // position threshold achieved
		if (jvel.norm() < iiwa_vel_thresh) {
			return true;
		}
	}
	return false;
}

// check if IIWA Cartesian motion is complete
bool isReachedDesPos(const Eigen::Vector3d& oppos_curr, const Eigen::Vector3d& oppos_des, const Eigen::VectorXd& jvel) {
	// IIWA joint position thresholds 
	const double pos_threshold = 3.0*WELD_OFFSET; //TODO: tune this. should be lower
	// IIWA joint velocity magnitude threshold
	const double iiwa_vel_thresh = 0.003; // rad/s
	Eigen::Vector3d pos_diff = oppos_curr - oppos_des;
	if (pos_diff.norm() < pos_threshold) { // position threshold achieved
		if (jvel.norm() < iiwa_vel_thresh) {
			return true;
		}
	}
	return false;
}

// sanity check position/orientation updates from haptic device
bool isPosOriChangeSafe(
	const Eigen::Vector3d& des_pos,
	const Eigen::Vector3d& curr_pos,
	const Eigen::Matrix3d& des_ori,
	const Eigen::Matrix3d& curr_ori
) {
	const double pos_change_thresh = 0.1; // 10 cm
	const double ori_change_thresh = 20.0 * M_PI/180.0; // 10 degrees
	bool ret_flag = true;
	if ((curr_pos - des_pos).norm() > pos_change_thresh) {
		cerr << "Haptic device position update exceeded threshold: " << pos_change_thresh << endl;
		cerr << "Curr position " << curr_pos << endl;
		cerr << "Des position " << des_pos << endl;
		ret_flag = false;
	}
	Eigen::AngleAxisd ori_change (curr_ori.transpose()*des_ori);
	//TODO: verify behavior with identity matrix
	if (ori_change.angle() > ori_change_thresh) {
		cerr << "Haptic device orientation update exceeded threshold: " << ori_change_thresh << 
		" commanded: " << ori_change.angle() << endl;
		ret_flag = false;
	}
	return ret_flag;
}

int main() {
	cout << "Loading URDF robot model file: " << robot_fname << endl;

	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);
	// home position of robot, TODO: update from teach pendant
	Eigen::VectorXd IIWA_HOME_POS = Eigen::Map<Eigen::VectorXd>(IIWA_HOME_JPOS, robot->dof());

	// read from Redis
	robot->_q = redis_client.getEigenMatrixJSON(RKEY_IIWA_JOINT_POS);
	robot->_dq = redis_client.getEigenMatrixJSON(RKEY_IIWA_JOINT_VEL);

	// write not_ready to redis
	redis_client.set(RKEY_IIWA_READY, std::to_string(0));

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_velocity_saturation_flag = true;
	joint_task->_saturation_velocity.head<3>() *= 0.25;
	joint_task->_saturation_velocity.tail<4>() *= 0.7;
	joint_task->_kp = 65.0;
	joint_task->_kv = 8.0;
	Eigen::VectorXd joint_task_torques = Eigen::VectorXd::Zero(dof);

	// set joint task desired position
	joint_task->_desired_position = IIWA_HOME_POS;

	// posori controller
	// set op space task link name, position in link
	const string oppoint_link_name = "link6";
	const Eigen::Vector3d oppoint_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.42774);
	// TODO: set desired orientation

	auto oppoint_task = new Sai2Primitives::PosOriTask(robot, oppoint_link_name, oppoint_pos_in_link);
	Eigen::VectorXd oppoint_task_torques = Eigen::VectorXd::Zero(dof);
	oppoint_task->_kp_pos = 60.0;
	oppoint_task->_kv_pos = 8.0;
	oppoint_task->_kp_ori = 60.0;
	oppoint_task->_kv_ori = 11.0;

	// controller states:
	enum ControllerState {Init=0, WaitForHapticDevice, Haptic, Failure, Float, CalibrateForceSensor, WeldTrajectory};
	// NOTE: force sensor calibration itself can be a state machine. for now, we only have one state
	// NOTE: we use a scoped enum for ForceSensorCalibrationState to avoid name collision wit the ControllerState
	enum class ForceSensorCalibrationState {Init=0, Finished};
	auto state = ControllerState::Init;
	ControllerState des_control_state;
	auto force_calib_state = ForceSensorCalibrationState::Init;
	enum class WeldTrajectoryState {Init=0, PrePoint, StartPoint, Weld, PostPoint, Finished};
	auto weld_traj_state = WeldTrajectoryState::Init;

	// flags
	bool haptic_device_ready = false;
	bool f_update_task_models = true;

	// cache
	Eigen::Vector3d des_pos; des_pos.setZero();
	Eigen::Matrix3d des_rot; des_rot.setIdentity();
	Eigen::Vector3d curr_pos; curr_pos.setZero();
	Eigen::Matrix3d curr_rot; curr_rot.setIdentity();
	Eigen::Quaterniond des_rot_quat;
	Eigen::MatrixXd Jv;
	Eigen::Vector3d curr_vel; curr_vel.setZero();
	Vector6d force; force.setZero();
	Vector6d force_sensor_bias; force_sensor_bias.setZero();
	Eigen::Vector3d joint_start, joint_stop; // extreme points of the weld joint
	Eigen::Vector3d plane1_normal, plane2_normal; // normals are corrected so that they indicate the free space
	Eigen::Vector3d pre_weld_point, weld_start, weld_stop, post_weld_point; // computed from weld points and planes
	uint state_counter = 0;
	bool fReachedPos = false;
	Vector6d calib_force; calib_force.setZero();

	// create a logger
	Logging::Logger logger(100000, "datalog.csv");
	logger.addVectorToLog(&calib_force, "force");
	logger.addVectorToLog(&curr_pos, "oppos");

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000000); // 1 s pause before starting loop

	// while window is open:
	double start_time = timer.elapsedTime();
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		robot->_q = redis_client.getEigenMatrixJSON(RKEY_IIWA_JOINT_POS);
		robot->_dq = redis_client.getEigenMatrixJSON(RKEY_IIWA_JOINT_VEL);
		force = redis_client.getEigenMatrixJSON(RKEY_IIWA_FORCE);
		robot->rotation(curr_rot, oppoint_link_name);
		getCalibratedForce(force, force_sensor_bias, curr_rot, calib_force);

		// update the model 20 times slower or when task hierarchy changes
		if(controller_counter%2 == 0 || f_update_task_models) {
			robot->updateModel();
			// update A matrix to stiffen last joints
			robot->_M(6,6) += 0.08;
			robot->_M(5,5) += 0.04;
			robot->_M(4,4) += 0.04;
			robot->_M_inv = robot->_M.inverse();
			f_update_task_models = true;
		}

		// update non-critical redis info for debugging
		if(controller_counter%200 == 0) {
			// write status to redis
			redis_client.set(RKEY_IIWA_CTRL_STATUS, to_string(state));
			redis_client.setEigenMatrixJSON(RKEY_IIWA_CTRL_FORCE_CALIB, calib_force);
			redis_client.setEigenMatrixJSON(RKEY_IIWA_CURR_POS, curr_pos);
		}

		////////////////////////////// Compute joint torques in state machine
		command_torques.setZero(dof);
		switch (state) {
			case ControllerState::Init:
				joint_task->_desired_position = IIWA_HOME_POS;
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					joint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					f_update_task_models = false;
				}
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;
				if(isReachedHomePos(robot->_q, joint_task->_desired_position, robot->_dq)) {
					if (isForceSensorValueGood(calib_force)) {
						// update state
						redis_client.set(RKEY_IIWA_READY, std::to_string(1));
						state = ControllerState::WaitForHapticDevice;
						haptic_device_ready = false;
					} else {
						state = ControllerState::Failure;
					}
				}
				break;
			case ControllerState::WaitForHapticDevice:
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					joint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					f_update_task_models = false;
				}
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;
				// read haptic device ack
				robot->position(des_pos, oppoint_link_name, oppoint_pos_in_link);
				robot->rotation(des_rot, oppoint_link_name);
				if(controller_counter%200 == 0) {
					try {
						haptic_device_ready = std::stoi(redis_client.get(RKEY_HAPTIC_READY));
					} catch (...) {
						cout << "Haptic ready fetch error" << endl;
						haptic_device_ready = false;
					}
					try {
						des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
					} catch (...) {
						des_control_state = state;
					}
					if (des_control_state == ControllerState::Float) {
						cout << "Switch to float" << endl;
						state = ControllerState::Float;
					}
					if (haptic_device_ready) {
						cout << "Switch to haptic" << endl;
						state = ControllerState::Haptic;
						// turn off OTG for haptic mode
						oppoint_task->_use_interpolation_flag = false;
						oppoint_task->reInitializeTask();
						f_update_task_models = true;
						// start logging
						if(!logger._f_is_logging){
							logger.start();
						}
					}
				}
				break;
			case ControllerState::Haptic:
				// sanity check desired position, orientation from haptic device
				des_pos = redis_client.getEigenMatrixJSON(RKEY_IIWA_DES_POS);
				des_rot = redis_client.getEigenMatrixJSON(RKEY_IIWA_DES_ORI);
				robot->position(curr_pos, oppoint_link_name, oppoint_pos_in_link);
				robot->rotation(curr_rot, oppoint_link_name);
				if (!isPosOriChangeSafe(des_pos, curr_pos, des_rot, curr_rot)) {
					state = ControllerState::Failure;
					joint_task->_desired_position = IIWA_HOME_POS;
					break;
				}
				oppoint_task->_desired_position = des_pos;
				des_rot_quat = Eigen::Quaterniond(des_rot);
				oppoint_task->_desired_orientation = des_rot_quat.toRotationMatrix();
				// cout << (des_rot.transpose() * des_rot - Eigen::Matrix3d::Identity()).norm() << endl;
				// cout << des_rot.determinant() << endl;
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					oppoint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					joint_task->updateTaskModel(oppoint_task->_N);
					f_update_task_models = false;
				}
				oppoint_task->computeTorques(oppoint_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques = oppoint_task_torques + joint_task_torques;
				// check for force sensor failure
				if (!isForceSensorValueGood(calib_force)) {
					state = ControllerState::Failure;
					joint_task->_desired_position = IIWA_HOME_POS;
				}
				if(controller_counter%200 == 0) {
					try {
						des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
					} catch (...) {
						des_control_state = state;
					}
					if (des_control_state == ControllerState::Float) {
						cout << "Switch to float" << endl;
						state = ControllerState::Float;
					} else if (des_control_state == ControllerState::WeldTrajectory) {
						cout << "Switch to weld" << endl;
						weld_traj_state = WeldTrajectoryState::Init;
						// turn on OTG for welding mode
						oppoint_task->_use_interpolation_flag = true;
						oppoint_task->reInitializeTask();
						oppoint_task->_desired_orientation = des_rot_quat.toRotationMatrix();
						f_update_task_models = true;
						state = ControllerState::WeldTrajectory;
					}
				}
				break;
			case ControllerState::Failure:
				command_torques.setZero(dof);
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					joint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					f_update_task_models = false;
				}
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;
				//TODO: if force sensor readings are good, we should switch to position control in free space with 0 force in the direction of the force sensor reading.
				//TODO: if not, we should switch to pure joint space damping.
				if(controller_counter%200 == 0) {
					redis_client.set(RKEY_IIWA_READY, std::to_string(0));
					try {
						des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
					} catch (...) {
						des_control_state = state;
					}
					if (des_control_state == ControllerState::Float) {
						cout << "Switch to float" << endl;
						state = ControllerState::Float;
					}
					if (des_control_state == ControllerState::Init) {
						cout << "Switch to init" << endl;
						state = ControllerState::Init;
					}
				}
				break;
			case ControllerState::Float:
				command_torques.setZero(dof);
				if(controller_counter%200 == 0) {
					redis_client.set(RKEY_IIWA_READY, std::to_string(0));
					try {
						des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
					} catch (...) {
						des_control_state = state;
					}
					if (des_control_state == ControllerState::Init) {
						cout << "Switch to init" << endl;
						state = ControllerState::Init;
					}
					if (des_control_state == ControllerState::CalibrateForceSensor) {
						// initialize force sensor state
						force_calib_state = ForceSensorCalibrationState::Init;
						cout << "Switch to force sensor calibration" << endl;
						state = ControllerState::CalibrateForceSensor;
					}
				}
				break;
			case ControllerState::CalibrateForceSensor:
				command_torques.setZero(dof);
				switch (force_calib_state) {
					case ForceSensorCalibrationState::Init:
						joint_task->_desired_position = Eigen::Map<Eigen::VectorXd> (IIWA_FORCE_CALIB_JPOS, robot->dof());
						if(isReachedHomePos(robot->_q, joint_task->_desired_position, robot->_dq)) {
							robot->rotation(curr_rot, oppoint_link_name);
							getSensorBias(force, curr_rot, force_sensor_bias);
							force_calib_state = ForceSensorCalibrationState::Finished;
						}
						break;
					case ForceSensorCalibrationState::Finished:
						if(controller_counter%200 == 0) {
							redis_client.set(RKEY_IIWA_READY, std::to_string(0));
							try {
								des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
							} catch (...) {
								des_control_state = state;
							}
							if (des_control_state == ControllerState::Float) {
								cout << "Switch to float" << endl;
								state = ControllerState::Float;
							}
							if (des_control_state == ControllerState::Init) {
								cout << "Switch to init" << endl;
								state = ControllerState::Init;
							}
						}
						break;
				}
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					joint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					f_update_task_models = false;
				}
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;
				break;
			case ControllerState::WeldTrajectory:
				robot->position(curr_pos, oppoint_link_name, oppoint_pos_in_link);
				// robot->Jv(Jv, oppoint_link_name, oppoint_pos_in_link);
				// curr_vel = Jv * robot->_dq;
				switch (weld_traj_state) {
					case WeldTrajectoryState::Init:
						// load data from json file
						loadJson(joint_estimation_file, joint_start, joint_stop, plane1_normal, plane2_normal);
						// NOTE: we do not sanity check the data
						// compute weld start and stop points
						weld_start = joint_start + WELD_OFFSET * (plane1_normal + plane2_normal);
						weld_stop = joint_stop + WELD_OFFSET * (plane1_normal + plane2_normal);
						// compute preweld point
						pre_weld_point = joint_start + PART_BACKOFF_DIST * (plane1_normal + plane2_normal);
						// TODO change above to be closer to the current ee position
						post_weld_point = joint_stop + PART_BACKOFF_DIST * (plane1_normal + plane2_normal);
						// TODO: compute optimal orientation
						// set oppos desired position to pre_point
						oppoint_task->_desired_position = pre_weld_point;
						// oppoint_task->_use_velocity_saturation_flag = true;
						// oppoint_task->_linear_saturation_velocity = 0.05;
						oppoint_task->_otg->setMaxLinearVelocity(0.05);
						oppoint_task->_otg->setMaxLinearAcceleration(0.025);
						cout << "Pre weld point" << endl;
						cout << pre_weld_point.transpose() << endl;
						joint_task->_desired_position = IIWA_HOME_POS;
						// set oppos task saturation velocity
						// switch state to PrePoint
						weld_traj_state = WeldTrajectoryState::PrePoint;
						state_counter = 0;
						fReachedPos = false;
						break;
					case WeldTrajectoryState::PrePoint:
						oppoint_task->_desired_position = pre_weld_point;
						// check if reached PrePoint
						if (!fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
							cout << "Reached pre weld point" << endl;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 500) {
							state_counter = 0;
							// set oppos desired point to weld start
							oppoint_task->_desired_position = weld_start;
							cout << "Weld start" << endl;
							cout << weld_start.transpose() << endl;
							fReachedPos = false;
							// f_update_task_models = true;
							// TODO: use OTG, higher gains
							// switch state to StartPoint
							weld_traj_state = WeldTrajectoryState::StartPoint;
						}
						break;
					case WeldTrajectoryState::StartPoint:
						oppoint_task->_desired_position = weld_start;
							// check if reached weld start
						if (!fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
							cout << "Reached weld start point" << endl;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 500) {
							state_counter = 0;
							// set oppos desired point to weld stop
							oppoint_task->_desired_position = weld_stop;
							cout << "Weld stop" << endl;
							cout << weld_stop.transpose() << endl;
							// lower oppos task saturation velocity
							// oppoint_task->_linear_saturation_velocity = 0.01;
							// oppoint_task->_saturation_velocity = 0.08;
							oppoint_task->_otg->setMaxLinearVelocity(0.02);
							fReachedPos = false;
							// TODO: use OTG, higher gains
							// switch state to Weld
							weld_traj_state = WeldTrajectoryState::Weld;
						}
						break;
					case WeldTrajectoryState::Weld:
						// check if reached weld stop
						if (!fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 500) {
							state_counter = 0;
							// set oppos desired point to post point
							oppoint_task->_desired_position = post_weld_point;
							cout << "Post weld point" << endl;
							cout << post_weld_point.transpose() << endl;
							// up oppos task saturation velocity again
							// oppoint_task->_linear_saturation_velocity = 0.05;
							// oppoint_task->_saturation_velocity = 0.08;
							oppoint_task->_otg->setMaxLinearVelocity(0.05);
							fReachedPos = false;
							// TODO: use OTG, higher gains
							// switch state to Weld
							weld_traj_state = WeldTrajectoryState::PostPoint;
						}
						break;
					case WeldTrajectoryState::PostPoint:
						// check if reached weld stop
						if (isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							weld_traj_state = WeldTrajectoryState::Finished;
							cout << "Finished weld trajectory" << endl;
						}
						break;
					case WeldTrajectoryState::Finished:
						// nothing to do. handle user input to float or init again.
						break;
				}
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					oppoint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					joint_task->updateTaskModel(oppoint_task->_N);
					f_update_task_models = false;
				}
				if(controller_counter%200 == 0) {
					try {
						des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
					} catch (...) {
						des_control_state = state;
					}
					if (des_control_state == ControllerState::Float) {
						cout << "Switch to float" << endl;
						state = ControllerState::Float;
					}
					if (des_control_state == ControllerState::Init) {
						cout << "Switch to init" << endl;
						state = ControllerState::Init;
						f_update_task_models = true;
					}
				}
				// comput joint torques
				oppoint_task->computeTorques(oppoint_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques = oppoint_task_torques + joint_task_torques;
				break;
		}

		//------ send torques to robot
		// cout << command_torques.transpose() << endl;
		redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_TORQUE, command_torques);

		controller_counter++;
	}

    command_torques.setZero(dof);
    redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_TORQUE, command_torques);
    redis_client.set(RKEY_IIWA_READY, std::to_string(0));

    //stop logging
    logger.stop();

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
