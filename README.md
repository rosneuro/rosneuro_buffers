# ROSNeuro Buffers package
The package provides a generic interface to implement buffers of NeuroData. Different types of buffers can be independently developed and dynamically loaded through the interface. Currently, the package provides a plugin to instanciate a RingBuffer.

## Requirements
rosneuro_buffers has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.02 LTS Focal Fossa** and **ROS Noetic**

rosneuro_buffers depends on:
- [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page)

rosneuro_buffers depends on the following @rosneuro packages:
- [rosneuro/rosneuro_msgs](https://github.com/rosneuro/rosneuro_msgs) 
- [rosneuro/rosneuro_data](https://github.com/rosneuro/rosneuro_data) 

## Usage
Once instanciated, a buffer plugin creates a buffer of **NeuroData** with the provided type **T** and size *nsamples*x*nchannels*. The buffer is initially filled with *nan* values. The function member *bool add(...)* allows to update the buffer with new *NeuroData* and the function member *bool isfull(void)* verifies when the buffer is full. The buffer is updated with new *NeuroData* according to the replacement policy implemented in the provided plugin.
In the case of **rosneuro::RingBuffer** plugin, the data replacement is performed according to a *FIFO* policy. 

The buffer can be configured in the following way:
- Explicit configuration in the code
- Through nameserver configuration

**Explicit configuration of the buffer**
```cpp
#include <ros/ros.h>
#include "rosneuro_buffers/RingBuffer.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "ringbuffer");
  
  constexpr unsigned int nsamples  = 10;
	constexpr unsigned int nchannels = 3;
  
  rosneuro::Buffer<float>* buffer = new rosneuro::RingBuffer<float>(nsamples, nchannels);
  // ...
  // ...
  delete buffer;
  return 0;
}
```

**Nameserver configuration**
File myringbuffer.cpp with the buffer implementation:
```cpp
#include <ros/ros.h>
#include "rosneuro_buffers/RingBuffer.hpp"

int main(int argc, char** argv) {

  ros::init(argc, argv, "ringbuffer");
  
  rosneuro::Buffer<float>* buffer = new rosneuro::RingBuffer<float>();
	if(buffer->configure("RingBufferCfgTest") == false) {
		ROS_ERROR("RingBuffer configuration failed");
		return false;
	}
	ROS_INFO("RingBuffer configuration succeeded");
  // ...
  // ...
  delete buffer;
  return 0;
}
```
File myringbuffer.yaml with the yaml configuration:
```yalm
RingBufferCfgTest:
  name: myringbuffer
  type: RingBufferFloat
  params: 
    nsamples: 10
    nchannels: 3
```
File myringbuffer.launch with the launcher (where *$MYPACKAGE* is the path where the yaml file is):
```xml
<launch>
  <rosparam command="load" file="$MYPACKAGE/myringbuffer.yaml"/>
  <node name="myringbuffer" pkg="rosneuro_buffers" type="myringbuffer" output="screen"/>
</launch>
```
### RingBuffer templates
The RingBuffer provides plugin for three type of NeuroData: *int*, *float*, and *double*. The different plugin can be loaded by providing the following types in the yaml configuration:
- RingBufferInt (for *rosneuro::NeuroData\<int\>*)
- RingBufferFloat (for *rosneuro::NeuroData\<float\>*)
- RingBufferDouble (for *rosneuro::NeuroData\<double\>*)
