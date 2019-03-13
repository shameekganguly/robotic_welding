
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <Eigen/Dense>
typedef Eigen::Matrix<double, 6, 1> Vector6d;

using namespace std;

// Redis keys
const string RKEY_IIWA_JOINT_POS = "sai2::iiwaForceControl::iiwaBot::sensors::q"; // read from sim/ driver
const string RKEY_IIWA_JOINT_VEL = "sai2::iiwaForceControl::iiwaBot::sensors::dq"; // read from sim/ driver
const string RKEY_IIWA_FORCE = "sai2::optoforceSensor::6Dsensor::force"; // read from OptoForce driver/ simulator
// const string RKEY_IIWA_FORCE = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::force_sensor::force";
const string RKEY_IIWA_DES_TORQUE = "sai2::iiwaForceControl::iiwaBot::actuators::fgc"; // write to sim/ driver

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

const string world_fname = "resources/world.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";

int main(int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// create a loop timer
	LoopTimer timer;
	const int SIM_FREQ = 10000;
	timer.setLoopFrequency(SIM_FREQ);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(10 * 1e6); // 10 ms pause before starting loop

	Eigen::VectorXd robot_torques = Eigen::VectorXd::Zero(robot->dof());
	// Eigen::VectorXd robot_torques_interact = Eigen::VectorXd::Zero(robot->dof());
	robot->_q.setZero(); //TODO: set IIWA home configuration
	robot->_dq.setZero();
	redis_client.setEigenMatrixJSON(RKEY_IIWA_JOINT_POS, robot->_q);
	redis_client.setEigenMatrixJSON(RKEY_IIWA_JOINT_VEL, robot->_dq);
	redis_client.setEigenMatrixJSON(RKEY_IIWA_DES_TORQUE, robot_torques);
	// redis_client.setEigenMatrixJSON(JOINT_INTERACTION_TORQUES_COMMANDED_KEY, robot_torques_interact);

	double time_sensor_last = timer.elapsedTime();
	const double SENSOR_WRITE_FREQ = 1000;
	Eigen::VectorXd gravity(robot->dof());
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		robot_torques = redis_client.getEigenMatrixJSON(RKEY_IIWA_DES_TORQUE);
		robot->gravityVector(gravity);
		// redis_client.getEigenMatrixDerivedString(JOINT_INTERACTION_TORQUES_COMMANDED_KEY, robot_torques_interact);
		sim->setJointTorques(robot_name, robot_torques+gravity);// + robot_torques_interact);

		// update simulation by 1ms
		sim->integrate(1.0 / SIM_FREQ);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		double time_sensor = timer.elapsedTime();
		if (time_sensor - time_sensor_last >= 1.0 / SENSOR_WRITE_FREQ) {
			// write joint kinematics to redis
			redis_client.setEigenMatrixJSON(RKEY_IIWA_JOINT_POS, robot->_q);
			redis_client.setEigenMatrixJSON(RKEY_IIWA_JOINT_VEL, robot->_dq);

			// redis_client.setCommandIs(SIM_TIMESTAMP_KEY, std::to_string(timer.elapsedSimTime()));
			time_sensor_last = time_sensor;
		}

	}
	return 0;
}
