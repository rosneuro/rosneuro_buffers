#ifndef ROSNEURO_RINGBUFFER_HPP
#define ROSNEURO_RINGBUFFER_HPP

#include "rosneuro_buffers/Buffer.hpp"

namespace rosneuro {

template<typename T>
class RingBuffer : public Buffer<T> {

	public:
		RingBuffer(void);
		~RingBuffer(void) {};

		bool set(unsigned int nrows, unsigned int ncols);

		bool configure(void);
		bool add(const Eigen::Ref< const DynamicMatrix<T> >& in);
	
	private:
		unsigned int size_;
};

template<typename T>
RingBuffer<T>::RingBuffer(void) {}

template<typename T>
bool RingBuffer<T>::set(unsigned int nrows, unsigned int ncols) {
	this->resize(nrows, ncols);
	this->is_set_ = true;

	return true;
}

template<typename T>
bool RingBuffer<T>::configure(void) {


	if (!Buffer<T>::getParam(std::string("size"), this->size_)) {
    	ROS_ERROR("[Buffer] Cannot find param size");
		return false;
	}
	
	return true;
}

template<typename T>
bool RingBuffer<T>::add(const Eigen::Ref< const DynamicMatrix<T> >& in) {

	if(this->is_set_ == false) {
		this->is_set_ = this->set(this->size_, in.cols());
		ROS_WARN("[%s] First apply: the buffer is set ([%d %ld])", this->name().c_str(), this->rows(), in.cols());
	}

	if(this->cols() != in.cols() ) {
		this->is_set_ = this->set(this->rows(), in.cols());
		ROS_WARN("[%s] Buffer and incoming data has different number of columns ([%d %d] vs. [%ld %ld])." 
				 "The buffer will be automatically resized.", this->name().c_str(), this->rows(), 
				  this->cols(), in.rows(), in.cols());
	}

	DynamicMatrix<T> cbuffer = this->data_.bottomRows(this->data_.rows() - in.rows());
	this->data_ << cbuffer, in;

	return true;
}

}


#endif
