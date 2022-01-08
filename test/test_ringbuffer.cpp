#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_buffers/RingBuffer.hpp"

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "test_ringbuffer");
	
	constexpr unsigned int nsamples  = 2;
	constexpr unsigned int nchannels = 3;

	rosneuro::NeuroData<float> frame(nsamples, nchannels, "EEG");

	rosneuro::Buffer<float>* buffer = new rosneuro::RingBuffer<float>();
	if(buffer->configure("RingBufferCfgTest") == false) {
		ROS_ERROR("RingBuffer configuration failed");
		return false;
	}
	ROS_INFO("RingBuffer configuration succeeded");


	if(buffer->isfull() == false) 
		ROS_INFO("Buffer is not full");

	int niter = 0;
	ROS_INFO("Filling the buffer");
	try {

		do {
			std::cout<<"<<<<<<<<"<<std::endl;
			Eigen::MatrixXf nframe = Eigen::MatrixXf::Zero(2, 3);
			Eigen::MatrixXf ibuffer;

			for (auto i = 0; i<nframe.rows(); i++)
				nframe.row(i) = (niter*2 + i + 1)*Eigen::VectorXf::Constant(3, 1);

			std::copy(nframe.data(), nframe.data() + nframe.size(), frame.data());

			buffer->add(frame);
			buffer->get(ibuffer);
			std::cout<<ibuffer<<std::endl;
			niter++;
		} while (buffer->isfull() == false);
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
	}
	
	ROS_INFO("Buffer filled after %d iterations", niter);
	Eigen::MatrixXf ebuffer;
	buffer->get(ebuffer);
	std::cout<<ebuffer<<std::endl;


	ROS_INFO("Adding a new frame");
	Eigen::MatrixXf nframe = Eigen::MatrixXf::Zero(2, 3);

	for (auto i = 0; i<nframe.rows(); i++)
		nframe.row(i) = (niter*2 + i + 1)*Eigen::VectorXf::Constant(3, 1);

	std::copy(nframe.data(), nframe.data() + nframe.size(), frame.data());

	buffer->add(frame);

	buffer->get(ebuffer);
	std::cout<<ebuffer<<std::endl;

	delete buffer;
	return 0;

}


