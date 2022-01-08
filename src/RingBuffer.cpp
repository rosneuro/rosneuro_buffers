#include "rosneuro_buffers/RingBuffer.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::RingBuffer<int>,    rosneuro::Buffer<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::RingBuffer<float>,  rosneuro::Buffer<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::RingBuffer<double>, rosneuro::Buffer<double>)
