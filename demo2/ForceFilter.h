// ForceFilter: Implements a Butterworth filter of 20Hz default cut off 
// cascaded with a median filter

#ifndef FORCE_FILTER_H
#define FORCE_FILTER_H

#include <Eigen/Core>
#include <vector>
#include <iostream>

// template <uint filter_order, uint len_med_filter, uint num_filt_channels>
template <uint filter_order, uint num_filt_channels>
class ForceFilter {
public:
	typedef Eigen::Matrix<double, filter_order, 1> CoeffFd;
	typedef Eigen::Matrix<double, num_filt_channels, 1> SampleFd;
	typedef Eigen::Matrix<double, num_filt_channels, filter_order> FirBufferFd;
	// typedef Eigen::Matrix<double, num_filt_channels, len_med_filter> MedBufferFd;
public:
	// empty ctor
	ForceFilter(){
		// nothing to do
	}

	//ctor
	ForceFilter(const CoeffFd& filter_coeffs,
				double dt)
	: _filter_coeffs(filter_coeffs),
	_count_samples(0),
	_dt(dt)
	{
		// _data_buffer_med.fill(0.0);
		_data_buffer_fir.fill(0.0);
		_data_filtered.fill(0.0);
	}

	// add sample
	void addSample(const SampleFd& sample) {
		if (_count_samples < filter_order) {
			_data_buffer_fir.col(_count_samples) = sample;
			_data_filtered = sample;
		} else {
			// // median filter
			// for (uint k = 1; k < len_med_filter; ++j) {
			// 	// shift columns
			// 	_data_buffer_med.col(k-1) = _data_buffer_med.col(k);
			// }
			// _data_buffer_med.col(len_med_filter - 1) = sample;
			// update output
			for (uint j = 1; j < filter_order; ++j) {
				// shift columns
				_data_buffer_fir.col(j-1) = _data_buffer_fir.col(j);
			}
			// for (uint i = 0; i < num_filt_channels; ++i) {
			// 	// find median of values
			// 	Eigen::Matrix<double, len_med_filter, 1> temp = _data_buffer_med.row(i);
			// 	size_t n = v.size() / 2;
			// 	nth_element(v.begin(), v.begin()+n, v.end());
			// 	return v[n];
			// 	_data_buffer_fir.col(filter_order - 1)[i] = ;
			// }
			_data_buffer_fir.col(filter_order-1) = sample;
			_data_filtered = _data_buffer_fir*_filter_coeffs;
		}
		++_count_samples;
	}

	// get filtered output
	const SampleFd& filteredSample() const {
		return _data_filtered;
	}


public:
	// filter coefficients. 2nd order filter. so 3 co-efficients
	// last filter co-efficient corresponds to most recent sample
	CoeffFd _filter_coeffs;

	// median filter window length.
	uint _len_med_filter;

	// number of data samples processed
	uint _count_samples;

	// sample dt
	double _dt;

	// // filter buffer for the median filter
	// MedBufferFd _data_buffer_med;

	// filter buffer for the FIR filter
	FirBufferFd _data_buffer_fir;

	// filter output
	SampleFd _data_filtered;
};

#endif //FORCE_FILTER_H
