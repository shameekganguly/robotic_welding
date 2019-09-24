/* demo1 - main.cpp
Demo 1: Simple trajectory following for welding

Author: Shameek Ganguly shameekg@stanford.edu
Date: 10/22/18
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;

const string world_fname = "resources/world.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";
// string camera_name = "camera_front";
// string camera_name = "camera_isometric";
// string camera_name = "camera_front";
string camera_name = "camera_front";
// string ee_link_name = "link6";

// contact link names
const string end_effector_name = "link6";


// global variables
Eigen::VectorXd q_home;
ForceSensorSim* tool_force_sensor;
ForceSensorDisplay* tool_force_display;

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

bool f_global_sim_pause = false; // use with caution!
// bool f_global_sim_pause = true; // use with caution!

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);
	graphics->_world->setBackgroundColor(0.3, 0.3, 0.3);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	// sim->setCollisionRestitution(0.0);
 //    // set co-efficient of friction also to zero for now as this causes jitter
 //    sim->setCoeffFrictionStatic(1.0);
 //    sim->setCoeffFrictionDynamic(1.0);

	// set initial condition
	q_home.setZero(robot->dof());
	q_home << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				-70.0/180.0*M_PI,
				0/180.0*M_PI,
				60.2/180.0*M_PI,
				187.2/180.0*M_PI;
	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	// control variables
	Eigen::VectorXd tau(robot->dof());
	Eigen::VectorXd tau_grav(robot->dof());

	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd Jv;
	Eigen::MatrixXd Jw;
	Eigen::MatrixXd J0;
	Eigen::VectorXd opspace_vel(6); // operational space velocity

	string ee_link_name = "link6";
	Eigen::Vector3d ee_link_pos(0.0, 0.0, 0.4);
	int phase = -1;

	bool fTimerDidSleep = true;
	double phase_change_time = timer.elapsedTime();
	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// check if paused
		if (f_global_sim_pause) { continue;}

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		// sim->getJointPositions(robot_name, robot->_q);
		// sim->getJointVelocities(robot_name, robot->_dq);

		robot->_q += robot->_dq*loop_dt;

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update kinematic models
		robot->updateModel();

		// ------------------------------------

		// - find operational space desired velocities
		double dx;
		double dy;
		double dz;
		double T = 5.0; //sec
		if (phase == -1) {
			dx = 0.0;
			dy = 0.0;
			dz = 0.0;
			// pause
			if (curr_time - phase_change_time > 2.0) {
				phase = 0;
				phase_change_time = curr_time;
			}
		} else if (phase == 0) {
			dx = 0.0;
			dy = -0.02;
			dz = -0.1;
			if (curr_time - phase_change_time > 1.0) {
				phase = 1;
				phase_change_time = curr_time;
			}
		} else if (phase == 1) {
			dx = 0.05;
			dy = 0.0;
			dz = 0.0;
			if (curr_time - phase_change_time > 4.0) {
				phase = 2;
				phase_change_time = curr_time;
			}
		} else if (phase == 2) {
			dx = 0.0;
			dy = 0.02;
			dz = 0.1;
			if (curr_time - phase_change_time > 1.0) {
				phase = 3;
				phase_change_time = curr_time;
			}
		} else if (phase == 3) {
			dx = -0.1;
			dy = 0.0;
			dz = 0.0;
			if (curr_time - phase_change_time > 2.0) {
				phase = 0;
				phase_change_time = curr_time;
			}
		} else {
			dx = 0.0;
			dy = 0.0;
			dz = 0.0;
		}
		// - calculate omega. Note that we use the quaternion form of the E+ matrix here
		Eigen::Vector3d omega = Eigen::Vector3d::Zero();
		// - obtain basic Jacobian at the end-effector frame.
		// - Note: we are using the switched Jacobian [Jw' Jv']' here.
		robot->J(J0, ee_link_name, ee_link_pos);
		// - assemble operational space velocity
		opspace_vel << omega.x(), omega.y(), omega.z(), dx, dy, dz;
		// - calculate the inertia weighted pseudo-inverse of J0
		Jbar = robot->_M_inv * J0.transpose() * (J0 * robot->_M_inv * J0.transpose()).inverse();
		// - finally, calculate the desired joint velocities to achieve the desired operational space
		// velocity
		robot->_dq = Jbar*opspace_vel;
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		if (!f_global_sim_pause) {
			// sim->integrate(loop_dt);
		}

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Robotic welding Sim", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
    if ((key == 'P' || key == 'p') && action == GLFW_PRESS)
    {
        // pause simulation
        f_global_sim_pause = !f_global_sim_pause;
    }
    if ((key == '1') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_front";
    }
    if ((key == '2') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_top";
    }
}
