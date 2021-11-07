cmake_minimum_required(VERSION 2.8.3)
project(filter_extract_feature)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  livox_ros_driver
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs livox_ros_driver
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(filter_node src/filter_node.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(extract_feature_node src/extract_feature_node.cpp)
target_link_libraries(extract_feature_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})



