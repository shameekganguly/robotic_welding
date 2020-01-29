/* Constants for demo 4 */

#ifndef DEMO4_H
#define DEMO4_H

// joint positions the robot goes to before haptic control
double IIWA_HOME_JPOS [7] = {
	4/180.0*M_PI,
	-25/180.0*M_PI,
	-2/180.0*M_PI,
	77/180.0*M_PI,
	6.3/180.0*M_PI,
	-55/180.0*M_PI,
	-84/180.0*M_PI
};

// base configuration away from workpiece before calibrating force sensor
/*
double IIWA_FORCE_CALIB_JPOS [7] = {
	6.5/180.0*M_PI,
	-33/180.0*M_PI,
	-7.3/180.0*M_PI,
	52/180.0*M_PI,
	1.56/180.0*M_PI,
	-91/180.0*M_PI,
	-4.75/180.0*M_PI
};*/

double IIWA_FORCE_CALIB_JPOS [7] = {
	4/180.0*M_PI,
        -25/180.0*M_PI,
        -2/180.0*M_PI,
        77/180.0*M_PI,
        6.3/180.0*M_PI,
        -55/180.0*M_PI,
        -84/180.0*M_PI
};


// range of motion of joint 7 (-ve lim, neutral, +ve lim)
// const double IIWA_FORCE_CALIB_JOINT7_RANGE [3] = {-}

// wrist point in joint 7 frame
// const double IIWA_WRIST_CONTROL_OPPOS [3] = {0.0, 0.0, -0.081};

// force sensor sign. +1 if tension is positive. -1 if compression is positive.
const int FORCE_SENSOR_SIGN = -1; // for old optoforce sensor, sign is -ve

// Z-rotation in radians of sensor frame wrt last link frame. measured wrt link frame
const double FORCE_SENSOR_ROT = 0.0/180.0*M_PI; // radians

// tool mass
const double TOOL_MASS = 2.0; // kg

// tool COM in force sensor frame
// Note that the force sensor frame is at the open face away from the robot
const double TOOL_COM [3] = {0.0904, -0.001, 0.1474};

// backoff distance from part
const double PART_BACKOFF_DIST = 0.05; // m

// offset from joint at which to weld
const double WELD_OFFSET = 0.005; // m

#endif
