cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_buffers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 roslib
			 std_msgs
			 pluginlib)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
find_package(GTest REQUIRED)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
    roslib
  	roscpp
	std_msgs
	pluginlib

  DEPENDS
)

catkin_add_gtest(test_buffer test/test_buffer.cpp)
target_link_libraries(test_buffer ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(test_buffer ${PROJECT_NAME} ${catkin_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS} gmock pthread)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
					include/${PROJECT_NAME}/)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME} src/Buffer.cpp)

target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES} 
									  ${catkin_LIBRARIES})

#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


