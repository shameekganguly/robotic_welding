/* Control robot in 6-DOF EE position, orientation */
#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

#include <iostream>
#include <string>

using namespace std;

// Redis keys
const string RKEY_IIWA_JOINT_POS = "sai2::iiwaForceControl::iiwaBot::sensors::q"; // read from sim/ driver
const string RKEY_IIWA_JOINT_VEL = "sai2::iiwaForceControl::iiwaBot::sensors::dq"; // read from sim/ driver
const string RKEY_IIWA_FORCE = "sai2::optoforceSensor::6Dsensor::force"; // read from OptoForce driver/ simulator
// const string RKEY_IIWA_FORCE = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::force_sensor::force";
const string RKEY_IIWA_DES_POS = "sai2::iiwaForceControl::iiwaBot::haptic::xp_des"; // read from haptic device
const string RKEY_IIWA_DES_ORI = "sai2::iiwaForceControl::iiwaBot::haptic::xr_des"; // read from haptic device
const string RKEY_IIWA_DES_VEL = "sai2::iiwaForceControl::iiwaBot::haptic::dx_des"; // Not used currently
const string RKEY_IIWA_DES_TORQUE = "sai2::iiwaForceControl::iiwaBot::actuators::fgc"; // write to sim/ driver
const string FGC_ENABLE_KEY  = "sai2::iiwaForceControl::iiwaBot::fgc_command_enabled"; // ? signal read from driver
const string RKEY_HAPTIC_READY = "sai2::iiwaForceControl::iiwaBot::haptic::device_ready"; // read from haptic device to indicate haptic device ready
const string RKEY_IIWA_READY = "sai2::iiwaForceControl::iiwaBot::haptic::robot_ready"; // sent to haptic device to indicate robot ready
const string RKEY_IIWA_CTRL_STATUS = "sai2::iiwaForceControl::iiwaBot::ctrl_status"; // controller state machine status

// models
const string world_fname = "resources/world.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";

#include <signal.h>
bool runloop = true;
unsigned long long controller_counter = 0;
void sighandler(int sig) { runloop = false;}

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
	iiwa_pos_thresh << 0.5, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0; //degrees 
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

// sanity check position/orientation updates from haptic device
bool isPosOriChangeSafe(
	const Eigen::Vector3d& des_pos,
	const Eigen::Vector3d& curr_pos,
	const Eigen::Matrix3d& des_ori,
	const Eigen::Matrix3d& curr_ori
) {
	const double pos_change_thresh = 0.1; // 10 cm
	const double ori_change_thresh = 5.0 * M_PI/180.0; // 5 degrees
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
	cout << "Loading URDF world model file: " << world_fname << endl;

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
	IIWA_HOME_POS << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				0/180.0*M_PI,
				60.2/180.0*M_PI,
				187.2/180.0*M_PI;

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
	// joint_task->_kp = 100.0;
	// joint_task->_kv = 14.0;
	Eigen::VectorXd joint_task_torques = Eigen::VectorXd::Zero(dof);

	// set joint task desired position
	joint_task->_desired_position = IIWA_HOME_POS;

	// posori controller
	// TODO: set op space task link name, position in link
	const string oppoint_link_name = "link6";
	const Eigen::Vector3d oppoint_pos_in_link = Eigen::Vector3d(0.01, 0.0, 0.44);
	// TODO: set desired orientation

	auto oppoint_task = new Sai2Primitives::PosOriTask(robot, oppoint_link_name, oppoint_pos_in_link);
	Eigen::VectorXd oppoint_task_torques = Eigen::VectorXd::Zero(dof);
	oppoint_task->_kp_pos = 200.0;
	oppoint_task->_kv_pos = 30.0;
	oppoint_task->_kp_ori = 200.0;
	oppoint_task->_kv_ori = 30.0;

	// controller states:
	enum ControllerState {Init, WaitForHapticDevice, Haptic, Failure};
	auto state = ControllerState::Init;

	// flags
	bool haptic_device_ready = false;
	bool f_update_task_models = true;

	// cache
	Eigen::Vector3d des_pos; des_pos.setZero();
	Eigen::Matrix3d des_rot; des_rot.setIdentity();
	Eigen::Vector3d curr_pos; curr_pos.setZero();
	Eigen::Matrix3d curr_rot; curr_rot.setIdentity();
	Vector6d force; force.setZero();

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

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

		// update the model 20 times slower or when task hierarchy changes
		if(controller_counter%20 == 0 || f_update_task_models) {
			robot->updateModel();
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
					if (isForceSensorValueGood(force)) {
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
				if(controller_counter%200 == 0) {
					try {
						haptic_device_ready = std::stoi(redis_client.get(RKEY_HAPTIC_READY));
					} catch (...) {
						haptic_device_ready = false;
					}
					if (haptic_device_ready) {
						state = ControllerState::Haptic;
						f_update_task_models = true;
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
					break;
				}
				oppoint_task->_desired_position = des_pos;
				oppoint_task->_desired_orientation = des_rot;
				if (f_update_task_models) { // set to true when robot kinematic model is updated
					oppoint_task->updateTaskModel(Eigen::MatrixXd::Identity(dof, dof));
					joint_task->updateTaskModel(oppoint_task->_N);
					f_update_task_models = false;
				}
				oppoint_task->computeTorques(oppoint_task_torques);
				joint_task->computeTorques(joint_task_torques);
				command_torques = oppoint_task_torques + joint_task_torques;
				// check for force sensor failure
				if (!isForceSensorValueGood(force)) {
					state = ControllerState::Failure;
				}
				break;
			case ControllerState::Failure:
				command_torques.setZero(dof);
				redis_client.set(RKEY_IIWA_READY, std::to_string(0));
				//TODO: if force sensor readings are good, we should switch to position control in free space with 0 force in the direction of the force sensor reading.
				//TODO: if not, we should switch to pure joint space damping.
				break;
		}

		//------ send torques to robot
		redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_TORQUE, command_torques);

		controller_counter++;
	}

    command_torques.setZero(dof);
    redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_TORQUE, command_torques);
    redis_client.set(RKEY_IIWA_READY, std::to_string(0));

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
