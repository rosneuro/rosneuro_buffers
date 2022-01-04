#ifndef ROSNEURO_BUFFER_CPP
#define ROSNEURO_BUFFER_CPP

namespace rosneuro {

template<typename T>
Buffer<T>::Buffer(const unsigned int nsamples, const unsigned int nchannels) {
	this->data_.resize(nsamples, nchannels);
	this->data_.fill(NAN);
}

template<typename T>
bool Buffer<T>::isfull(void) {
	return !(this->data_.hasNaN());
}

template<typename T>
void Buffer<T>::get(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& data) {
	data = this->data_;
}


}


#endif
