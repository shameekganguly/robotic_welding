/* Read position and force from robot, compute and render feedback force to haptic device */

/* Control robot in 6-DOF EE position, orientation */
#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>
typedef Eigen::Matrix<double, 6, 1> Vector6d;
#include "chai3d.h"
#include "ForceFilter.h"

#include <iostream>
#include <string>

using namespace std;
using namespace chai3d;

// Redis keys
const string RKEY_IIWA_JOINT_POS = "sai2::KUKA_IIWA::sensors::q"; // read from sim/ driver
const string RKEY_IIWA_JOINT_VEL = "sai2::KUKA_IIWA::sensors::dq"; // read from sim/ driver
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

// force sensor calibration
// NOTE: We only calibrate and return the measured force right now, not the moment
void calibrated_force(const Vector6d& raw_force, const Eigen::Matrix3d& ee_ori, Eigen::Vector3d* calib_force) {
	// orientation offset from sensor to ee
	Eigen::Matrix3d ori_sensor_ee;
	double rot_ee_sensor = -90.0*M_PI/180.0; // measured from hardware setup
	ori_sensor_ee << cos(rot_ee_sensor), -sin(rot_ee_sensor), 0.0,
					sin(rot_ee_sensor), cos(rot_ee_sensor), 0.0,
						0.0,			0.0,				1.0;
	Eigen::Vector3d sensor_force_bias;
	sensor_force_bias << -12.9,  -1.0, -28.0;
	double tool_mass = 2.35;
	// pseudocode:
	// temp1 = sensor_force - bias
	// temp2 = ori_ee_world*ori_sensor_ee*temp1;
	// calib_force = temp2 - gravity_ee;

	// Assume calib_force not null. should be a pass by reference
	*calib_force = ee_ori*ori_sensor_ee*(raw_force.head(3) - sensor_force_bias) - tool_mass*Eigen::Vector3d(0.0, 0.0, -9.8);
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
	const string oppoint_link_name = "link6";
	const Eigen::Vector3d oppoint_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.42774);

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
	bool is_in_contact = false;

	// cache
	Eigen::Vector3d des_pos, curr_robot_pos; des_pos.setZero(); curr_robot_pos.setZero();
	Eigen::Matrix3d des_rot, curr_robot_rot; des_rot.setIdentity(); curr_robot_rot.setIdentity();
	Eigen::Vector3d robot_wspace_pos; robot_wspace_pos.setZero();
	Eigen::Matrix3d robot_wspace_rot; robot_wspace_rot.setIdentity();
	chai3d::cVector3d raw_pos;
	chai3d::cMatrix3d raw_rot;
	Eigen::Vector3d haptic_pos; haptic_pos.setZero();
	Eigen::Matrix3d haptic_rot; haptic_rot.setIdentity();
	Vector6d raw_force;
	Eigen::Vector3d force; force.setZero();
	Eigen::Vector3d haptic_force;

	vector<string> ret_str(3);

	// force filter
	const uint filter_order = 10;
	ForceFilter<filter_order, 3> ffilter;
	auto filter_coeffs = ForceFilter<filter_order, 3>::CoeffFd::Ones()*(1.0/filter_order);
	ffilter = ForceFilter<filter_order, 3>(filter_coeffs, 0.001);

	// parameters
	const double haptic_to_robot_pos_scaling = 1.0;
	const double haptic_to_robot_rot_scaling = 1.0;
	const double haptic_imped_force_scaling_free_space = 10.0;
	const double haptic_imped_force_scaling_contact = 50.0;
	Eigen::Matrix3d haptic_to_robot_rotation_frame;
	haptic_to_robot_rotation_frame.setIdentity(); // TODO: set this based on actual scenario, camera

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(3000000000); // 3 s pause before starting loop

	// while window is open:
	double start_time = timer.elapsedTime();
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		ret_str = redis_client.pipeget({RKEY_IIWA_JOINT_POS, RKEY_IIWA_JOINT_VEL, RKEY_IIWA_FORCE});
		robot->_q = RedisClient::decodeEigenMatrixJSON(ret_str[0]);
		robot->_dq = RedisClient::decodeEigenMatrixJSON(ret_str[1]);
		robot->position(curr_robot_pos, oppoint_link_name, oppoint_pos_in_link);
		robot->rotation(curr_robot_rot, oppoint_link_name);
		raw_force = RedisClient::decodeEigenMatrixJSON(ret_str[2]);
		calibrated_force(raw_force, curr_robot_rot, &force);
		double contact_thresh = 3.5; //N
		is_in_contact = (force.norm() > contact_thresh);
		// if(controller_counter%1000 == 0) {
		// 	cout << "Raw force: " << raw_force.head(3).transpose() << " "
		// 		<< "Calib force: " << force.transpose() << " "
		// 		<< "Calib force norm: " << force.norm() << endl;
		// }

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
				// set zero haptic force
				hapticDevice->setForce(cVector3d(0, 0, 0));
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
				redis_client.pipeset({
					{RKEY_IIWA_DES_POS, RedisClient::encodeEigenMatrixJSON(des_pos)},
					{RKEY_IIWA_DES_ORI, RedisClient::encodeEigenMatrixJSON(des_rot)}
				});
				// set haptic device force
				robot->position(curr_robot_pos, oppoint_link_name, oppoint_pos_in_link);
				robot->rotation(curr_robot_rot, oppoint_link_name);
				haptic_force = haptic_to_robot_rotation_frame.transpose()*(curr_robot_pos - des_pos);
				if (!is_in_contact) {
					haptic_force *= haptic_imped_force_scaling_free_space;
				} else {
					haptic_force *= haptic_imped_force_scaling_contact;
				}
				ffilter.addSample(haptic_force);
				// cout << haptic_force.norm() << endl;
				hapticDevice->setForce(cVector3d(ffilter.filteredSample()));
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
