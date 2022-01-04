#ifndef ROSNEURO_RINGBUFFER_CPP
#define ROSNEURO_RINGBUFFER_CPP

namespace rosneuro {

template<typename T>
RingBuffer<T>::RingBuffer(const unsigned int nsamples, const unsigned int nchannels) : 
									Buffer<T>(nsamples, nchannels) {}


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
