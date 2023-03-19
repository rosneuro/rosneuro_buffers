# ROSNeuro Buffers package
The package provides a generic interface to implement buffers. Different types of buffers can be independently developed and dynamically loaded through the interface. Currently, the package provides a plugin to instanciate a RingBuffer.

## Requirements
rosneuro_buffers has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.05 LTS Focal Fossa** and **ROS Noetic**

rosneuro_buffers depends on:
- [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page) 

## Usage
Once instanciated, a buffer plugin creates a buffer with the provided type **T** and *size*. The buffer is initially filled with *nan* values. The function member *bool add(const DynamicMatrix\<T\>& in)* allows to update the buffer with new data and the function member *bool isfull(void)* verifies when the buffer is full. The buffer is updated with new data according to the replacement policy implemented in the provided plugin.
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
  
  constexpr int nsamples  = 10;
  constexpr int nchannels = 3;
  
  rosneuro::Buffer<float>* buffer = new rosneuro::RingBuffer<float>();
  
  buffer->set(nsamples, nchannels);
  
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
    size: 10
```
File myringbuffer.launch with the launcher (where *$MYPACKAGE* is the path where the yaml file is):
```xml
<launch>
  <rosparam command="load" file="$MYPACKAGE/myringbuffer.yaml"/>
  <node name="myringbuffer" pkg="rosneuro_buffers" type="myringbuffer" output="screen"/>
</launch>
```
If the buffer is configured by *bool configure(void)* function (as in the case of YAML configuration), the number of channels is not required. The number of channels will be automatically deduced during the first call of the function *bool add(const DynamicMatrix\<T\>& in)*.


### RingBuffer templates
The RingBuffer provides plugin for three type of Eigen data: *int*, *float*, and *double*. The different plugin can be loaded by providing the following types in the yaml configuration:
- RingBufferInt (for *Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>*)
- RingBufferFloat (for *Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>*)
- RingBufferDouble (for *Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>*)

## How to implement a custom rosneuro::Buffer plugin
In the following section, we describe how to implement a custom plugin based on *rosneuro::Buffer* class. We assume that the plugin is implemente in a new package named **mybuffers_package** and the plugin is named **MyBuffer**. Furthermore we assume that the *mybuffers_package* is structured as follows:
```
mybuffers_package/
	|- include/mybuffers_package/MyBuffer.hpp
	|- src/
	   |- MyBuffer.cpp
	   |- mybuffer.cpp
	|- CMakeLists.txt
	|- package.xml
	|- plugin_mybuffer.xml
```
The *MyBuffer* class derives from the *rosneuro::Buffer* base class and it requires to implement the two pure virtual function members *void add(const DynamicMatrix\<T\>& in)* and *bool configure(void)*. Here an example of the implementation of the class in the *include/mybuffers_package/MyBuffer.hpp* file:
```cpp
#ifndef ROSNEURO_MYBUFFER_HPP
#define ROSNEURO_MYBUFFER_HPP

#include "rosneuro_buffers/Buffer.hpp"

namespace rosneuro {

template<typename T>
class MyBuffer : public Buffer<T> {

	public:
		MyBuffer(void) {};
		~MyBuffer(void) {};

		bool configure(void);
		void add(const DynamicMatrix<T>& in);

};

template<typename T>
bool MyBuffer<T>::configure(void) {

	if (!Buffer<T>::getParam(std::string("size"), this->size_)) {
    	ROS_ERROR("[MyBuffer] Cannot find param nsamples");
		return false;
	}

	return true;
}

template<typename T>
void MyBuffer<T>::add(const DynamicMatrix<T>& in) {

	// Specific implementation of MyBuffer
}

}

#endif
```
Notice that the function member *MyBuffer\<T\>::configure(void)* is automatically called inside the method *rosneuro::Buffer\<T\>::configure(const std::string& name)*. Therefore, in order to execute the function member *MyBuffer\<T\>::configure(void)* is required to call the function *bool configure(const std::string& name)* in the executable with argument the name of the YAML structure.

In *src/MyBuffer.cpp*, we just add the plugin macros:
```cpp
#include "rosneuro_buffers/MyBuffer.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::MyBuffer<int>,    rosneuro::Buffer<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::MyBuffer<float>,  rosneuro::Buffer<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::MyBuffer<double>, rosneuro::Buffer<double>)
```
In the *CMakeLists.txt* we need to provide the rules to compile the plugin library:
```
cmake_minimum_required(VERSION 2.8.3)
project(mybuffers_package)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 roslib
			 std_msgs
			 pluginlib)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}_mybuffer
  CATKIN_DEPENDS
    roslib
  	roscpp
	std_msgs
	pluginlib
  DEPENDS
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME}_mybuffer src/MyBuffer.cpp)
target_link_libraries(${PROJECT_NAME}_mybuffer ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_mybuffer ${catkin_EXPORTED_TARGETS})

#################
## Executables ##
#################

add_executable(mybuffer src/mybuffer.cpp)
target_link_libraries(mybuffer ${PROJECT_NAME}_mybuffer)
```
In the *package.xml* we need to add the dependency to the pluginlib package and to export our mybuffer plugin:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>mybuffers_package</name>
  <version>0.0.1</version>
  <description>My buffers package</description>
  <author email="luca.tonin@unipd.it">Luca Tonin</author>
  <maintainer email="luca.tonin@unipd.it">Luca Tonin</maintainer>

  <license>GPLv3</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>message_generation</depend>

  <exec_depend>message_runtime</exec_depend>
  <depend>eigen</depend>
  <depend>roslib</depend>
  <depend>rosconsole</depend>
  <build_depend>pluginlib</build_depend>
  <exec_depend>pluginlib</exec_depend>

  <export>
    <rosneuro_buffers plugin="${prefix}/plugin_mybuffer.xml" />
  </export>
  

</package>
```
Finally, we need to add the description of the plugin in the *plugin_mybuffer.xml* file:
```xml
<class_libraries>
  <library path="lib/libmybuffers_package">
    <class name="mybuffers_package/MyBufferDouble" type="rosneuro::MyBuffer<double>"
        base_class_type="rosneuro::Buffer<double>">
      <description>MyBuffer with doubles</description>
    </class>
    
	<class name="mybuffers_package/MyBufferFloat" type="rosneuro::MyBuffer<float>"
        base_class_type="rosneuro::Buffer<float>">
      <description>MyBuffer with floats</description>
    </class>
    
    <class name="mybuffers_package/MyBufferInt" type="rosneuro::MyBuffer<int>"
        base_class_type="rosneuro::Buffer<int>">
      <description>MyBuffer with ints</description>
    </class>
  </library>
</class_libraries> 
```

