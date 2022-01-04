#ifndef ROSNEURO_BUFFER_HPP
#define ROSNEURO_BUFFER_HPP

#include <Eigen/Dense>
#include "rosneuro_data/NeuroData.hpp"

namespace rosneuro {

template <typename T>
class Buffer {

	public:
		Buffer(const unsigned int nsamples, const unsigned int nchannels);
		virtual ~Buffer(void) {};

		Buffer(const Buffer&) = delete;
		Buffer& operator=(const Buffer&) = delete;

		virtual void add(const NeuroData<T>& frame) = 0;
		void get(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& data);

		bool isfull(void);

	public:
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data_;


	public: 
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}

#include "../src/Buffer.cpp"



#endif
