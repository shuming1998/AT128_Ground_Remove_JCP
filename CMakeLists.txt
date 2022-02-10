cmake_minimum_required(VERSION 2.8.3)
project(at128_jpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE -O2)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
)
find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES at128_jpc
 CATKIN_DEPENDS cv_bridge image_transport pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(at128_jpc src/at128_jpc.cpp) 
target_link_libraries(at128_jpc ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_INCLUDE_DIRS}) 
add_dependencies(at128_jpc beginner_tutorials_generate_messages_cpp)

