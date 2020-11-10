/* Control robot in 6-DOF EE position, orientation for curvilinear welding */
#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>
#include <json/json.h>
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

// models
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";

const string joint_estimation_file = "Result_stitched.json";

// backoff distance from part
const double PART_BACKOFF_DIST = 0.05; // m

// offset from joint at which to weld
const double WELD_OFFSET = 0.002; // m

const double PART_HEIGHT = 0.05; // m

const double WELD_LINEAR_SPEED = 0.025; // m/s

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
	Eigen::Vector3d& circle_center,
	double& circle_radius,
	Eigen::Vector3d& normal
) {
	Json::Value root;
	std::ifstream t(fname);
	Json::Reader reader;
	reader.parse(t, root);
	jsonArrToVector3d(root["circle_center"], circle_center);
	circle_radius = root["circle_radius"].asDouble();
	jsonArrToVector3d(root["circle_normal"], normal);
}

struct JointLinearSegment {
	Vector3d joint_start;
	Vector3d joint_end;
	Vector3d normal1;
	Vector3d normal2;
};

struct JointCircularSegment {
	Vector3d circle_center;
	double circle_radius;
	Vector3d circle_normal;
};

struct ThreePartJointDemo5 {
	JointLinearSegment segment1;
	JointCircularSegment segment2;
	JointLinearSegment segment3;
};

void loadMultisegmentJson(
	const string& fname,
	ThreePartJointDemo5& joint_data
) {
	Json::Value root;
	std::ifstream t(fname);
	Json::Reader reader;
	reader.parse(t, root);
	Json::Value segment1 = root[0];
	jsonArrToVector3d(segment1["joint_start"], joint_data.segment1.joint_start);
	jsonArrToVector3d(segment1["joint_end"], joint_data.segment1.joint_end);
	jsonArrToVector3d(segment1["plane1_normal"], joint_data.segment1.normal1);
	jsonArrToVector3d(segment1["plane2_normal"], joint_data.segment1.normal2);

	Json::Value segment2 = root[1];
	jsonArrToVector3d(segment2["circle_center"], joint_data.segment2.circle_center);
	joint_data.segment2.circle_radius = segment2["circle_radius"].asDouble();
	jsonArrToVector3d(segment2["circle_normal"], joint_data.segment2.circle_normal);


	Json::Value segment3 = root[2];
	jsonArrToVector3d(segment3["joint_start"], joint_data.segment3.joint_start);
	jsonArrToVector3d(segment3["joint_end"], joint_data.segment3.joint_end);
	jsonArrToVector3d(segment3["plane1_normal"], joint_data.segment3.normal1);
	jsonArrToVector3d(segment3["plane2_normal"], joint_data.segment3.normal2);
}


// sanity check force sensor data. assumes data is gravity compensated.
bool isForceSensorValueGood(const Vector6d& force) { // force vector is size six. 
	const double force_threshold = 10; //N of force magnitude
	const double moment_threshold = 4;//Nm, assumes applied at the torch tip
	// first 3 in array is force, next 3 is moment
	bool ret_flag = true;
	if (force.head(3).norm() > force_threshold) {
		cerr << "ERROR: Force sensor - force magnitude exceeded threshold: " << force_threshold << endl;
		ret_flag = false;
	}
	if (force.tail(3).norm() > moment_threshold) {
		cerr << "ERROR: Force sensor - moment magnitude exceeded threshold: " << moment_threshold << endl;
		ret_flag = false;
	}
	return ret_flag;
}

// check if IIWA joint motion is complete
bool isReachedHomePos(const Eigen::VectorXd& jpos_curr, const Eigen::VectorXd& jpos_des, const Eigen::VectorXd& jvel) {
	// IIWA joint position thresholds 
	Eigen::VectorXd iiwa_pos_thresh(jpos_curr.size());
	iiwa_pos_thresh << 1.5, 1.5, 10.0, 2.0, 2.0, 2.0, 10.0; //degrees 
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
	const double iiwa_vel_thresh = 0.006; // rad/s
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
	const double pos_change_thresh = 0.3; // 10 cm
	const double ori_change_thresh = 60.0 * M_PI/180.0; // 10 degrees
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
		cerr << "Haptic device orientation update exceeded threshold: " << ori_change_thresh << endl;
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
	Eigen::VectorXd IIWA_HOME_POS(robot->dof());
	// joint angles in radians
	IIWA_HOME_POS << -115/180.0*M_PI,
				-25/180.0*M_PI,
				-2/180.0*M_PI,
				82/180.0*M_PI,
				6.3/180.0*M_PI,
				-55/180.0*M_PI,
				150/180.0*M_PI;

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
	joint_task->_saturation_velocity *= 0.4;
	joint_task->_kp = 20.0;
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
	oppoint_task->_kv_ori = 8.0;

	// controller states:
	enum ControllerState {Init, WaitForHapticDevice, Haptic, Failure, WeldTrajectory};
	auto state = ControllerState::Init;
	enum class WeldTrajectoryState {
		Init=0,
		PrePoint,
		StartPointSegment1,
		WeldSegment1,
		WeldSegment2,
		WeldSegment3,
		PostPoint,
		Finished
	};
	auto weld_traj_state = WeldTrajectoryState::Init;

	// flags
	bool haptic_device_ready = false;
	bool f_update_task_models = true;

	// cache
	Eigen::Vector3d des_pos; des_pos.setZero();
	Eigen::Matrix3d des_rot; des_rot.setIdentity();
	Eigen::Vector3d curr_pos; curr_pos.setZero();
	Eigen::Quaterniond des_rot_quat;
	Eigen::Matrix3d curr_rot; curr_rot.setIdentity();
	Vector6d force; force.setZero();

	ControllerState des_control_state;

	// weld trajectory parameters
	ThreePartJointDemo5 joint_data;
	loadMultisegmentJson(joint_estimation_file, joint_data);
	Eigen::Vector3d local_x_vector_init, local_x_vector;
	Eigen::Vector3d weld_start_onjoint;
	Eigen::Vector3d pre_weld_point, weld_start, weld_stop, post_weld_point; // computed from weld points and planes
	double weld_theta = 0.0; // trajectory variable around the circle;
	double weld_theta_stop = M_PI; // only do half trajectory to start with
	uint state_counter = 0;
	bool fReachedPos = false;
	double weld_start_time = 0.0;
	double weld_omega = 0.0;
	Vector3d weld_temp1;

	// create a logger. log at 10Hz
	Logging::Logger logger(100000, "datalog.csv");
	logger.addVectorToLog(&force, "force");
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
		// TODO: gravity compensate force sensor data
		// TODO: transform force to correct frame

		// update the model 20 times slower or when task hierarchy changes
		if(controller_counter%20 == 0 || f_update_task_models) {
			robot->updateModel();
			// // update A matrix to stiffen last joints
			// robot->_M(6,6) += 0.08;
			// robot->_M(5,5) += 0.04;
			// robot->_M(4,4) += 0.04;
			// robot->_M_inv = robot->_M.inverse();
			f_update_task_models = true;
		}

		// update non-critical redis info for debugging
		if(controller_counter%200 == 0) {
			// write status to redis
			redis_client.set(RKEY_IIWA_CTRL_STATUS, to_string(state));
		}

		////////////////////////////// Compute joint torques in state machine
		command_torques.setZero(dof);
		switch (state) {
			case ControllerState::Init:
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					joint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					f_update_task_models = false;
				}
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;
				if(isReachedHomePos(robot->_q, IIWA_HOME_POS, robot->_dq)) {
					if (true) {// (isForceSensorValueGood(force)) {
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
						des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
						if (des_control_state == ControllerState::WeldTrajectory) {
							cout << "Switch to weld" << endl;
							weld_traj_state = WeldTrajectoryState::Init;
							// turn on OTG for welding mode
							oppoint_task->_use_interpolation_flag = true;
							oppoint_task->reInitializeTask();
							oppoint_task->_desired_orientation = des_rot; //des_rot_quat.toRotationMatrix();
							f_update_task_models = true;
							state = ControllerState::WeldTrajectory;
						}
					} catch (...) {
						des_control_state = state;
					}
					try {
						haptic_device_ready = std::stoi(redis_client.get(RKEY_HAPTIC_READY));
					} catch (...) {
						haptic_device_ready = false;
					}
					if (haptic_device_ready) {
						state = ControllerState::Haptic;
						oppoint_task->_use_interpolation_flag = false;
						oppoint_task->reInitializeTask();
						f_update_task_models = true;
						logger.start();
					}
				}
				break;
			case ControllerState::Haptic:
				// sanity check desired position, orientation from haptic device
				des_pos = redis_client.getEigenMatrixJSON(RKEY_IIWA_DES_POS);
				des_rot = redis_client.getEigenMatrixJSON(RKEY_IIWA_DES_ORI);
				des_rot_quat = Eigen::Quaterniond(des_rot);
				robot->position(curr_pos, oppoint_link_name, oppoint_pos_in_link);
				robot->rotation(curr_rot, oppoint_link_name);
				// cout << "curr x: " << curr_pos.transpose() << endl;
				// cout << "des x: " << des_pos.transpose() << endl;
				if (!isPosOriChangeSafe(des_pos, curr_pos, des_rot, curr_rot)) {
					cout << "q: " << robot->_q.transpose() << endl;
					cout << "x: " << curr_pos.transpose() << endl;
					state = ControllerState::Failure;
					joint_task->_desired_position = IIWA_HOME_POS;
					break;
				}
				oppoint_task->_desired_position = des_pos;
				oppoint_task->_desired_orientation = des_rot_quat.toRotationMatrix();
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					oppoint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					joint_task->updateTaskModel(oppoint_task->_N);
					f_update_task_models = false;
				}
				oppoint_task->computeTorques(oppoint_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques = oppoint_task_torques + joint_task_torques;
				// check for force sensor failure
				if (false) {//(!isForceSensorValueGood(force)) {
					state = ControllerState::Failure;
					joint_task->_desired_position = IIWA_HOME_POS;
				}
				break;
			case ControllerState::Failure:
				command_torques.setZero(dof);
				redis_client.set(RKEY_IIWA_READY, std::to_string(0));
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					joint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					f_update_task_models = false;
				}
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;
				//TODO: if force sensor readings are good, we should switch to position control in free space with 0 force in the direction of the force sensor reading.
				//TODO: if not, we should switch to pure joint space damping.
				break;
			case ControllerState::WeldTrajectory:
				robot->position(curr_pos, oppoint_link_name, oppoint_pos_in_link);
				// robot->Jv(Jv, oppoint_link_name, oppoint_pos_in_link);
				// curr_vel = Jv * robot->_dq;
				switch (weld_traj_state) {
					case WeldTrajectoryState::Init:
						weld_start = joint_data.segment1.joint_start + WELD_OFFSET * (joint_data.segment1.normal1 + joint_data.segment1.normal2);
						weld_stop = joint_data.segment3.joint_end + WELD_OFFSET * (joint_data.segment3.normal1 + joint_data.segment3.normal2);
						// compute preweld point
						pre_weld_point = joint_data.segment1.joint_start + PART_BACKOFF_DIST * (joint_data.segment1.normal1 + joint_data.segment1.normal2);

						oppoint_task->_desired_position = pre_weld_point;
						oppoint_task->_desired_orientation = des_rot;
						// cout << "curr_rot " << endl;
						// cout << des_rot << endl;
						// oppoint_task->_desired_orientation.col(0) << -local_x_vector_init;
						// oppoint_task->_desired_orientation.col(1) << circle_normal.cross(local_x_vector_init);
						// oppoint_task->_desired_orientation.col(2) << -circle_normal;
						// cout << "des_rot " << endl;
						// cout << oppoint_task->_desired_orientation << endl;
						// oppoint_task->_use_velocity_saturation_flag = true;
						// oppoint_task->_linear_saturation_velocity = 0.05;
						oppoint_task->_otg->setMaxLinearVelocity(2.0*WELD_LINEAR_SPEED);
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
							oppoint_task->_otg->setMaxLinearVelocity(1.0*WELD_LINEAR_SPEED);
							oppoint_task->_desired_position = weld_start;
							// desired orientation stays the same as the pre weld point
							cout << "Weld start" << endl;
							cout << weld_start.transpose() << endl;
							fReachedPos = false;
							// f_update_task_models = true;
							// switch state to StartPoint
							weld_traj_state = WeldTrajectoryState::StartPointSegment1;
						}
						break;
					case WeldTrajectoryState::StartPointSegment1:
							// check if reached weld start
						if (!fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
							cout << "Reached weld start point" << endl;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 200) {
							state_counter = 0;
							cout << "Execute linear segment 1" << endl;
							weld_start_time = timer.elapsedTime();
							oppoint_task->_desired_position = joint_data.segment1.joint_end + WELD_OFFSET * (joint_data.segment1.normal1 + joint_data.segment1.normal2);
							// lower oppos task saturation velocity
							// oppoint_task->_linear_saturation_velocity = 0.01;
							// oppoint_task->_saturation_velocity = 0.08;
							oppoint_task->_otg->setMaxLinearVelocity(WELD_LINEAR_SPEED);
							fReachedPos = false;
							// TODO: use OTG, higher gains
							// switch state to Weld
							weld_traj_state = WeldTrajectoryState::WeldSegment1;
						}
						break;
					case WeldTrajectoryState::WeldSegment1:
						// check if reached weld start
						if (!fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
							cout << "Reached end of linear segment 1" << endl;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 100) {
							state_counter = 0;
							cout << "Execute circular segment 2" << endl;
							weld_start_time = timer.elapsedTime();
							oppoint_task->_otg->setMaxLinearVelocity(WELD_LINEAR_SPEED);
							fReachedPos = false;
							weld_traj_state = WeldTrajectoryState::WeldSegment2;

							// compute weld start and stop points
							weld_theta = 0.0;
							local_x_vector_init = joint_data.segment2.circle_center - joint_data.segment1.joint_end;
							local_x_vector_init /= local_x_vector_init.norm();
							weld_temp1 = joint_data.segment2.circle_center - joint_data.segment3.joint_start;
							weld_temp1 /= weld_temp1.norm();
							weld_theta_stop = acos(weld_temp1.dot(local_x_vector_init));

							weld_omega = WELD_LINEAR_SPEED / joint_data.segment2.circle_radius;
						}
						break;
					case WeldTrajectoryState::WeldSegment2:
						// compute weld trajectory
						weld_theta = (timer.elapsedTime() - weld_start_time) * weld_omega;
						weld_theta = min(weld_theta_stop, weld_theta);
						local_x_vector = Eigen::AngleAxisd(weld_theta, -joint_data.segment2.circle_normal) * local_x_vector_init;
						
						oppoint_task->_desired_position = joint_data.segment2.circle_center - local_x_vector * joint_data.segment2.circle_radius;
						oppoint_task->_desired_position += WELD_OFFSET * (joint_data.segment2.circle_normal - local_x_vector);
						// TODO: set desired velocity
						// oppoint_task->_desired_orientation.col(0) << -local_x_vector;
						// oppoint_task->_desired_orientation.col(1) << circle_normal.cross(local_x_vector);
						// oppoint_task->_desired_orientation.col(2) << -circle_normal;

						// check if reached weld stop
						if ( abs(weld_theta - weld_theta_stop) < 0.08 && !fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
							cout << "Reached end of circular segment 2" << endl;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 100) {
							state_counter = 0;
							// set oppos desired point to post point
							oppoint_task->_desired_position = joint_data.segment3.joint_end + WELD_OFFSET * (joint_data.segment3.normal1 + joint_data.segment3.normal2);
							// keep desired orientation same as weld_point
							cout << "Execute linear segment 3" << endl;
							// up oppos task saturation velocity again
							// oppoint_task->_linear_saturation_velocity = 0.05;
							// oppoint_task->_saturation_velocity = 0.08;
							oppoint_task->_otg->setMaxLinearVelocity(WELD_LINEAR_SPEED);
							fReachedPos = false;
							// TODO: use OTG, higher gains
							// switch state to Weld
							weld_traj_state = WeldTrajectoryState::WeldSegment3;
						}
						break;
					case WeldTrajectoryState::WeldSegment3:
							// check if reached weld start
						if (!fReachedPos && isReachedDesPos(curr_pos, oppoint_task->_desired_position, robot->_dq)) {
							state_counter = 0;
							fReachedPos = true;
							cout << "Reached end of linear segment 3" << endl;
						}
						// waste a few cycles to pause motion
						state_counter++;
						if (fReachedPos && state_counter > 500) {
							state_counter = 0;
							// set oppos desired point to post point
							post_weld_point = joint_data.segment3.joint_end + PART_BACKOFF_DIST * (joint_data.segment3.normal1 + joint_data.segment3.normal2);
							oppoint_task->_desired_position = post_weld_point;
							// keep desired orientation same as weld_point
							cout << "Go to post point" << endl;
							// up oppos task saturation velocity again
							// oppoint_task->_linear_saturation_velocity = 0.05;
							// oppoint_task->_saturation_velocity = 0.08;
							oppoint_task->_otg->setMaxLinearVelocity(2.0*WELD_LINEAR_SPEED);
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
				// if(controller_counter%200 == 0) {
				// 	try {
				// 		des_control_state = static_cast<ControllerState>(std::stoi(redis_client.get(RKEY_IIWA_CTRL_STATE_INPUT)));
				// 	} catch (...) {
				// 		des_control_state = state;
				// 	}
				// 	if (des_control_state == ControllerState::Float) {
				// 		cout << "Switch to float" << endl;
				// 		state = ControllerState::Float;
				// 	}
				// 	if (des_control_state == ControllerState::Init) {
				// 		cout << "Switch to init" << endl;
				// 		state = ControllerState::Init;
				// 		f_update_task_models = true;
				// 	}
				// }
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

    // stop logging
    if(logger._f_is_logging) {
    	logger.stop();
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
