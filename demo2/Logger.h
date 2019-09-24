#ifndef WLOGGER_H
#define WLOGGER_H

#include <fstream>
#include <unistd.h>
#include <chrono>
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
#include <Eigen/Dense>
#include <thread>
#include <vector>

namespace Logging {

// Log formatter
// TODO: allow user defined log formatter
Eigen::IOFormat logVecFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ");

class Logger {
public:
	// ctor
	Logger(long interval, std::string fname)
	: _log_interval_(interval)
	{
		// create log file
		_logfile.open(fname, std::ios::out);
	}

	// add Eigen::VectorXd variable to watch
	bool addVectorToLog (Eigen::VectorXd* var) {
		if (_f_is_logging) {
			return false;
		}
		_vars_to_log.push_back(var);
		return true;
	}

	// start logging
	bool start() {
		// save start time
		_t_start = system_clock::now();

		// set logging to true
		_f_is_logging = true;

		// start logging thread by move assignment
		_log_thread = std::thread{&Logger::logWorker, this};

		return true;
	}

	void stop() {
		// set logging false
		_f_is_logging = false;

		// join thread
		_log_thread.join();

		// close file
		_logfile.close();
	}

	// // vector of pointers to Eigen::VectorXd objects that are registered with the logger
	std::vector<Eigen::VectorXd*> _vars_to_log;

	// state
	bool _f_is_logging;

	// start time
	system_clock::time_point _t_start;

	// log interval
	long _log_interval_;

	// log file
	std::fstream _logfile;

	// thread
	std::thread _log_thread;

private:
	// thread function for logging. Note that we are not using mutexes here, no there might be weirdness
	void logWorker () {
		system_clock::time_point curr_time;
		system_clock::time_point last_time = system_clock::now();
		while (_f_is_logging) {
			usleep(_log_interval_/2);
			curr_time = system_clock::now();
			auto time_diff = std::chrono::duration_cast<microseconds>(curr_time - last_time);
			if (_log_interval_ > 0 && time_diff >= microseconds(static_cast<uint>(_log_interval_))) {
				microseconds t_elapsed = std::chrono::duration_cast<microseconds>(curr_time - _t_start);
				// log something
				_logfile << t_elapsed.count();
				for (auto iter: _vars_to_log) {
					_logfile << ", " << (*iter).transpose().format(logVecFmt);
				}
				_logfile << "\n";
				last_time = curr_time;
			}
		}
	}

	// hide default constructor
	Logger () {

	}
};

}

#endif //WLOGGER_H