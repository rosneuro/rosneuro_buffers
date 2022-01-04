#ifndef ROSNEURO_RINGBUFFER_HPP
#define ROSNEURO_RINGBUFFER_HPP

#include "rosneuro_buffers/Buffer.hpp"

namespace rosneuro {

template<typename T>
class RingBuffer : public Buffer<T> {

	public:
		RingBuffer(const unsigned int nsamples, const unsigned int nchannels);
		~RingBuffer(void) {};

		void add(const NeuroData<T>& frame);

};

}

#include "../src/RingBuffer.cpp"

#endif
