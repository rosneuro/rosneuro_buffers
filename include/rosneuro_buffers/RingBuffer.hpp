#ifndef ROSNEURO_RINGBUFFER_HPP
#define ROSNEURO_RINGBUFFER_HPP

#include "rosneuro_buffers/Buffer.hpp"

namespace rosneuro {

template<typename T>
class RingBuffer : public Buffer<T> {

	public:
		RingBuffer(void);
		~RingBuffer(void) {};

		bool configure(void);
		bool add(const Eigen::Ref< const DynamicMatrix<T> >& in);
};

template<typename T>
RingBuffer<T>::RingBuffer(void) {}

template<typename T>
bool RingBuffer<T>::configure(void) {

	int size;

	if (!Buffer<T>::getParam(std::string("size"), size)) {
    	ROS_ERROR("[Buffer] Cannot find param size");
		return false;
	}

	this->resize(size, 1);
	
	return true;
}

template<typename T>
bool RingBuffer<T>::add(const Eigen::Ref< const DynamicMatrix<T> >& in) {

	if(this->is_configured_ == false) {
		ROS_ERROR("[%s] Buffer is not configured", this->name().c_str());
		return false;
	}

	if(this->is_set_ == false) {
		this->resize(this->rows(), in.cols());
		this->is_set_ = true;
	}

	DynamicMatrix<T> cbuffer = this->data_.bottomRows(this->data_.rows() - in.rows());
	this->data_ << cbuffer, in;

	return true;
}

}


#endif
