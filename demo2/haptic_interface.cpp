/* Read position and force from robot, compute and render feedback force to haptic device */

/* Control robot in 6-DOF EE position, orientation */
#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>
typedef Eigen::Matrix<double, 6, 1> Vector6d;
#include "chai3d.h"

#include <iostream>
#include <string>

using namespace std;
using namespace chai3d;

// Redis keys
const string RKEY_IIWA_JOINT_POS = "sai2::iiwaForceControl::iiwaBot::sensors::q"; // read from sim/ driver
const string RKEY_IIWA_JOINT_VEL = "sai2::iiwaForceControl::iiwaBot::sensors::dq"; // read from sim/ driver
const string RKEY_IIWA_FORCE = "sai2::optoforceSensor::6Dsensor::force"; // read from OptoForce driver/ simulator
const string RKEY_HAPTIC_READY = "sai2::iiwaForceControl::iiwaBot::haptic::device_ready"; // read from haptic device to indicate haptic device ready
const string RKEY_IIWA_READY = "sai2::iiwaForceControl::iiwaBot::haptic::robot_ready"; // sent to haptic device to indicate robot ready
const string RKEY_HAPTIC_STATUS = "sai2::iiwaForceControl::iiwaBot::haptic::haptic_status";
const string RKEY_IIWA_DES_POS = "sai2::iiwaForceControl::iiwaBot::haptic::xp_des"; // write to robot
const string RKEY_IIWA_DES_ORI = "sai2::iiwaForceControl::iiwaBot::haptic::xr_des"; // write to robot

// models
const string world_fname = "resources/world.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";

#include <signal.h>
bool runloop = true;
unsigned long long controller_counter = 0;
void sighandler(int sig) { runloop = false;}

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
	const string oppoint_link_name = "link6";
	const Eigen::Vector3d oppoint_pos_in_link = Eigen::Vector3d(0,0,0.2);

	// write not_ready to redis
	redis_client.set(RKEY_HAPTIC_READY, std::to_string(0));

	// haptic device
	int dof = robot->dof();
	Eigen::Vector3d command_position; command_position.setZero();
	Eigen::Matrix3d command_rotation = Eigen::Matrix3d::Identity();

	// create a haptic device handler
    auto handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
    cGenericHapticDevicePtr hapticDevice;
	if (!handler->getDevice(hapticDevice, 0)) {
		cout << "No haptic device found. " << endl;
		exit(1);
	} else {
		hapticDevice->open();
		hapticDevice->calibrate();
	}

	// haptic controller states:
	enum HapticState {WaitForRobot, Haptic, Failure};
	auto state = HapticState::WaitForRobot;

	// flags
	bool robot_ready = false;
	bool first_haptic_iteration = false;

	// cache
	Eigen::Vector3d des_pos, curr_robot_pos; des_pos.setZero(); curr_robot_pos.setZero();
	Eigen::Matrix3d des_rot, curr_robot_rot; des_rot.setIdentity(); curr_robot_rot.setIdentity();
	Eigen::Vector3d robot_wspace_pos; robot_wspace_pos.setZero();
	Eigen::Matrix3d robot_wspace_rot; robot_wspace_rot.setIdentity();
	chai3d::cVector3d raw_pos;
	chai3d::cMatrix3d raw_rot;
	Eigen::Vector3d haptic_pos; haptic_pos.setZero();
	Eigen::Matrix3d haptic_rot; haptic_rot.setIdentity();
	Vector6d force; force.setZero();

	// parameters
	const double haptic_to_robot_pos_scaling = 4.0;
	const double haptic_to_robot_rot_scaling = 1.0;
	Eigen::Matrix3d haptic_to_robot_rotation_frame;
	haptic_to_robot_rotation_frame.setIdentity(); // TODO: set this based on actual scenario, camera

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
		if(controller_counter%20 == 0) {
			robot->updateModel();
		}

		// update non-critical redis info for debugging
		if(controller_counter%200 == 0) {
			// write status to redis
			redis_client.set(RKEY_HAPTIC_STATUS, to_string(state));
		}

		// read haptic device position
		hapticDevice->getPosition(raw_pos);
		haptic_pos = raw_pos.eigen();
		// TODO: orientation

		////////////////////////////// Compute feedback force for haptic device, set position for robot
		switch (state) {
			case HapticState::WaitForRobot:
				// read robot ready
				if(controller_counter%200 == 0) {
					try {
						robot_ready = std::stoi(redis_client.get(RKEY_IIWA_READY));
					} catch (...) {
						robot_ready = false;
					}
					if (robot_ready) {
						state = HapticState::Haptic;
						first_haptic_iteration = true;
						// compute robot current position
						robot->updateModel();
						robot->position(curr_robot_pos, oppoint_link_name, oppoint_pos_in_link);
						robot->rotation(curr_robot_rot, oppoint_link_name);
						des_pos = curr_robot_pos;
						des_rot = curr_robot_rot;
						// compute robot workspace center. TODO: use haptic primitive
						robot_wspace_pos = -haptic_pos*haptic_to_robot_pos_scaling;
						robot_wspace_pos = haptic_to_robot_rotation_frame*robot_wspace_pos + curr_robot_pos;
						cout << "robot_wspace_pos " << robot_wspace_pos << endl;
						// TODO: compute transform for rotation
					}
				}
				break;
			case HapticState::Haptic:
				// update desired position, orientation
				des_pos = haptic_pos*haptic_to_robot_pos_scaling;
				des_pos = robot_wspace_pos + haptic_to_robot_rotation_frame*des_pos;
				redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_POS, des_pos);
				redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_ORI, des_rot);
				// TODO: check for force sensor failure
				if (first_haptic_iteration) {
					// send ack the first time after setting desired position
					redis_client.set(RKEY_HAPTIC_READY, std::to_string(1));
					first_haptic_iteration = false;
				}
				// intermittently check if controller is still running
				if(controller_counter%200 == 0) {
					robot_ready = std::stoi(redis_client.get(RKEY_IIWA_READY));
					if (!robot_ready) {
						state = HapticState::WaitForRobot;
					}
				}
				break;
			case HapticState::Failure:
				redis_client.set(RKEY_HAPTIC_READY, std::to_string(0));
				break;
		}

		controller_counter++;
	}

    redis_client.set(RKEY_HAPTIC_READY, std::to_string(0));

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
