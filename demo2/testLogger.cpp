// testLogger.cpp: to test the Logger class

#include "Logger.h"
#include <iostream>

using namespace std;
using namespace Logging;

int main(int argc, char ** argv) {
	cout << "Starting test. Writing log to test.txt" << endl;

	// create a logger instance
	auto logger = Logger(500000, "test.txt");

	// create variables for the logger to log
	Eigen::Vector3d a1;
	Eigen::VectorXd a2(4);

	// set initial values	
	a1 << 1, 2, 3;
	a2 << 0, 1, 0, 1;

	// add logger to track variables
	logger.addVectorToLog(&a1, "a1");
	logger.addVectorToLog(&a2, "a2");

	// start logger
	logger.start();

	// sleep 1 sec, change variable values
	usleep(1e6); // 1 seconds
	a1 << 4, 5, 6;
	a2 << 0, 1, 1, 1;

	// repeat above for 3 more secs.
	usleep(1e6); // 1 seconds
	a1 << 7, 8, 9;
	a2 << 1, 0, 0, 1;
	usleep(1e6); // 1 seconds
	a1 << 20, 11, 12;
	a2 << 1, 1, 0, 1;
	usleep(1e6); // 1 seconds

	// stop logger
	logger.stop();

	cout << "Done." << endl;
	return 0;
}