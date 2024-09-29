cmake_minimum_required(VERSION 3.0.2)
project(slam_note)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# 包含自己的cmake
include(cmake/packages.cmake)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  # cv_bridge
  
  # pcl library
  pcl_ros

  # external
  # ndt_omp
  livox_ros_driver

  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
# find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
# find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL 
  # GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

include_directories(
	include
  src
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
	# ${GTSAM_INCLUDE_DIR}
)



# file(GLOB common_lib "lib/*.cpp")

add_executable(app_node 
  src/app/slam_app.cc
  src/system/SystemManager.cc
  src/system/ConfigParams.cc
)
# add_dependencies(system_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(app_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  gtsam
  glog
)
