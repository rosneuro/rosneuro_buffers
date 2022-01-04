#include <iostream>
#include <algorithm>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_buffers/RingBuffer.hpp"

int main(int argc, char** argv) {

	int niter = 0;
	rosneuro::RingBuffer<float> buffer(10, 3);
	rosneuro::NeuroData<float> frame(2, 3, "EEG");

	if(buffer.isfull() == false)
		std::cout<<"Buffer is not full"<<std::endl;

	std::cout<<"Filling the buffer"<<std::endl;
	try {

		do {
			std::cout<<"<<<<<<<<"<<std::endl;
			Eigen::MatrixXf nframe = Eigen::MatrixXf::Zero(2, 3);

			for (auto i = 0; i<nframe.rows(); i++)
				nframe.row(i) = (niter*2 + i + 1)*Eigen::VectorXf::Constant(3, 1);

			std::copy(nframe.data(), nframe.data() + nframe.size(), frame.data());

			buffer.add(frame);
			std::cout<<buffer.data_<<std::endl;
			niter++;
		} while (buffer.isfull() == false);
	} catch (std::runtime_error& e) {
		std::cout<<e.what()<<std::endl;
	}
	
	std::cout<<"Buffer filled after "<<niter<<" iterations"<<std::endl;
	Eigen::MatrixXf ebuffer;
	buffer.get(ebuffer);
	std::cout<<ebuffer<<std::endl;


	std::cout<<"New frame"<<std::endl;
	Eigen::MatrixXf nframe = Eigen::MatrixXf::Zero(2, 3);

	for (auto i = 0; i<nframe.rows(); i++)
		nframe.row(i) = (niter*2 + i + 1)*Eigen::VectorXf::Constant(3, 1);

	std::copy(nframe.data(), nframe.data() + nframe.size(), frame.data());

	buffer.add(frame);

	buffer.get(ebuffer);
	std::cout<<ebuffer<<std::endl;


	return 0;

}


