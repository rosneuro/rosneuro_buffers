cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_buffers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 roslib
			 std_msgs
			 pluginlib
			 rosneuro_data
			 rosneuro_msgs)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}_ringbuffer
  CATKIN_DEPENDS
    roslib
  	roscpp
	std_msgs
	pluginlib
	rosneuro_data
	rosneuro_msgs

  DEPENDS
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
					include)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME}_ringbuffer src/RingBuffer.cpp)
target_link_libraries(${PROJECT_NAME}_ringbuffer ${Eigen3_LIBRARIES} 
												 ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME}_ringbuffer ${catkin_EXPORTED_TARGETS})

#################
## Executables ##
#################

add_executable(test_ringbuffer test/test_ringbuffer.cpp)
target_link_libraries(test_ringbuffer ${PROJECT_NAME}_ringbuffer ${CATKIN_LIBRARIES})

#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}_ringbuffer
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_ringbuffer.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
