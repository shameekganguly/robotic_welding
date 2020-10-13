// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "redis/RedisClient.h"
// #include "uiforce/UIForceWidget.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>
#include <cmath>

using namespace std;

const string world_fname = "resources/world5.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "IIWA";
string camera_name = "camera_front";


// Redis keys
const string RKEY_IIWA_JOINT_POS = "sai2::KUKA_IIWA::sensors::q"; // read from sim/ driver
const string RKEY_IIWA_JOINT_VEL = "sai2::KUKA_IIWA::sensors::dq"; // read from sim/ driver
const string RKEY_CAMERA_VIEW_FRAME = "sai2::camera::view_frame";

#include <signal.h>
bool runloop = true;
unsigned long long controller_counter = 0;
void sighandler(int sig) { runloop = false;}

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// callback when user scrolls
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset);

// flags for scene camera movement
static bool fTransXp = false;
static bool fTransXn = false;
static bool fTransYp = false;
static bool fTransYn = false;
static bool fRotPanTilt = false;
static bool fZoom = false;
static double zoomSpeed = 0.0;
static bool fRobotLinkSelect = false;


int main(int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);
	graphics->_world->setBackgroundColor(0.3, 0.3, 0.3);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// create a UI widget to apply mouse forces to the robot
	// UIForceWidget force_widget(robot_name, robot, graphics);
	// force_widget.setEnable(false);

	/*------- Set up visualization -------*/
	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);
	glfwSetScrollCallback(window, mouseScroll);

	// cache and temp variables
	double last_cursorx, last_cursory;

	Eigen::VectorXd interaction_torques;

    // while window is open:
    // create a loop timer
	double update_freq = 30;
	LoopTimer timer;
	timer.setLoopFrequency(update_freq);   // 30Hz
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 10 ms pause before starting loop

	// while window is open:
	double start_time = timer.elapsedTime();
    while (!glfwWindowShouldClose(window) && runloop) {
    	// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		robot->_q = redis_client.getEigenMatrixJSON(RKEY_IIWA_JOINT_POS);
		robot->_dq = redis_client.getEigenMatrixJSON(RKEY_IIWA_JOINT_VEL);

		// update transformations
		robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents();

	    // move scene camera as required
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			// camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
			// TODO: the above doesn't work as intended because Chai treats the lookat
			// vector as a direction vector in the local frame, rather than as a lookat point
			camera_pos = m_pan*(camera_pos);
			camera_lookat = m_pan*(camera_lookat);
			// TODO: the above fix is a HUGE hack. Think about improving this.
	    }
	    if (fZoom) {
			camera_pos = camera_pos + 0.04*camera_lookat*zoomSpeed;
			fZoom = false;
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);

	    // set redis key
	    Eigen::Vector3d cam_x = (camera_pos - camera_lookat);
	    cam_x -= cam_up_axis*(cam_up_axis.dot(cam_x));
	    cam_x /= cam_x.norm();
	    Eigen::Matrix3d world_to_camera_frame;
	    world_to_camera_frame.col(0) = cam_x;
	    world_to_camera_frame.col(1) = (cam_up_axis.cross(cam_x));
	    world_to_camera_frame.col(2) = cam_up_axis;
	    // cout << "\n" << world_to_camera_frame << endl;

	    redis_client.setEigenMatrixDerived(RKEY_CAMERA_VIEW_FRAME, world_to_camera_frame);

	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	 //    if (fRobotLinkSelect) {
		// 	//activate widget
		// 	force_widget.setEnable(true);
		// 	// get current cursor position
		// 	double cursorx, cursory;
		// 	glfwGetCursorPos(window, &cursorx, &cursory);
		// 	int wwidth_scr, wheight_scr;
		// 	int wwidth_pix, wheight_pix;
		// 	glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
		// 	glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);
		// 	int viewx, viewy;
		// 	viewx = floor(cursorx/wwidth_scr * wwidth_pix);
		// 	viewy = floor(cursory/wheight_scr * wheight_pix);
		// 	std::string ret_link_name;
		// 	Eigen::Vector3d ret_pos;
		// 	if (cursorx > 0 && cursory > 0) {
		// 		force_widget.setInteractionParams(camera_name, viewx, wheight_pix-viewy, wwidth_pix, wheight_pix);
		// 		//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
		// 		// then drag the mouse over a link to start applying a force to it.
		// 	}
	 //    } else {
		// 	force_widget.setEnable(false);
	 //    }
		// // get UI torques
		// force_widget.getUIJointTorques(interaction_torques);
		// //write to redis
		// redis_client.setEigenMatrixDerivedString(JOINT_INTERACTION_TORQUES_COMMANDED_KEY, interaction_torques);
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
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
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		default:
			break;
    }
    if ((key == '1') && action == GLFW_PRESS) {
        // change camera
        camera_name = "camera_front";
    }
    if ((key == '2') && action == GLFW_PRESS) {
        // change camera
        camera_name = "camera_top";
    }
    if ((key == '3') && action == GLFW_PRESS) {
        // change camera
        // camera_name = "camera_front_zoom";
        camera_name = "camera_front_watchwelding";
    }
    if ((key == '4') && action == GLFW_PRESS) {
        // change camera
        camera_name = "camera_inside_zoom";
    }
    if ((key == '5') && action == GLFW_PRESS) {
    	camera_name = "camera_back_zoom";
    }
}

//------------------------------------------------------------------------------
void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			// TODO: move link select to shift + left click
			// TODO: set menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
	fZoom = true;
	zoomSpeed = yoffset;
}
