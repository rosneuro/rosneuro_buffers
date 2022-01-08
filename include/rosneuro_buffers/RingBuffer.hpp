#ifndef ROSNEURO_RINGBUFFER_HPP
#define ROSNEURO_RINGBUFFER_HPP

#include "rosneuro_buffers/Buffer.hpp"

namespace rosneuro {

template<typename T>
class RingBuffer : public Buffer<T> {

	public:
		RingBuffer(void) {};
		RingBuffer(const unsigned int nsamples, const unsigned int nchannels);
		~RingBuffer(void) {};

		bool configure(void);
		void add(const NeuroData<T>& frame);

};

template<typename T>
RingBuffer<T>::RingBuffer(const unsigned int nsamples, const unsigned int nchannels) {
	this->data_.resize(nsamples, nchannels);
	this->data_.fill(static_cast<T>(NAN));
}

template<typename T>
bool RingBuffer<T>::configure(void) {

	unsigned int nsamples;
	unsigned int nchannels;

	if (!Buffer<T>::getParam(std::string("nsamples"), nsamples)) {
    	ROS_ERROR("[Buffer] Cannot find param nsamples");
		return false;
	}
	
	if (!Buffer<T>::getParam(std::string("nchannels"), nchannels)) {
    	ROS_ERROR("[Buffer] Cannot find param nchannels");
		return false;
	}

	this->data_.resize(nsamples, nchannels);
	this->data_.fill(static_cast<T>(NAN));

	return true;

}

template<typename T>
void RingBuffer<T>::add(const NeuroData<T>& frame) {

	unsigned int ns_frame = frame.nsamples();
	unsigned int nc_frame = frame.nchannels();
	unsigned int ns_buffer = this->data_.rows();
	unsigned int nc_buffer = this->data_.cols();

	if (nc_frame != nc_buffer) {
		throw std::runtime_error("[Error] - Different number of channels between buffer and frame");
	}

	T* cdata = const_cast<T*>(frame.data());
	Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> eframe(cdata, ns_frame, nc_frame);

	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cbuffer(ns_buffer - ns_frame, nc_buffer);
	cbuffer = this->data_.bottomRows(ns_buffer - ns_frame);
	this->data_ << cbuffer, eframe;
}


}


#endif
