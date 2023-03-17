#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include "rosneuro_buffers/Buffer.hpp"
#include "rosneuro_buffers/RingBuffer.hpp"

template<typename T>
rosneuro::DynamicMatrix<T> generate_frame(int nrows, int ncols, int index) {

	rosneuro::DynamicMatrix<T> frame = rosneuro::DynamicMatrix<T>::Zero(nrows, ncols);

	for (auto i = 0; i<frame.rows(); i++)
		frame.row(i) = (index*nrows + i + 1)*rosneuro::DynamicMatrix<T>::Constant(nrows, ncols, 1);

	return frame;
}


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "test_ringbuffer");
	
	constexpr unsigned int frame_rows = 3;
	constexpr unsigned int frame_cols = 10;

	rosneuro::Buffer<float>* buffer = new rosneuro::RingBuffer<float>();
	if(buffer->configure("RingBufferCfgTest") == false) {
		ROS_ERROR("[%s] Configuration failed", buffer->name().c_str());
		return false;
	}
	ROS_INFO("[%s] Configuration succeeded", buffer->name().c_str());


	if(buffer->isfull() == false) 
		ROS_INFO("[%s] Buffer is not full", buffer->name().c_str());

	int niter = 0;
	ROS_INFO("[%s] Filling the buffer", buffer->name().c_str());
	rosneuro::DynamicMatrix<float> frame;
	rosneuro::DynamicMatrix<float> output;

	try {

		do {
			std::cout<<"<<<<<<<<"<<std::endl;
			frame = generate_frame<float>(frame_rows, frame_cols, niter);

			buffer->add(frame);
			output = buffer->get();
			std::cout<<output<<std::endl;
			niter++;
		} while (buffer->isfull() == false);
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
	}
	
	ROS_INFO("[%s] Buffer filled after %d iterations", buffer->name().c_str(), niter);
	output = buffer->get();
	std::cout<<output<<std::endl;


	ROS_INFO("[%s] Adding a new frame", buffer->name().c_str());
	frame = generate_frame<float>(frame_rows, frame_cols, niter);

	buffer->add(frame);

	output = buffer->get();
	std::cout<<output<<std::endl;

	delete buffer;
	return 0;

}


