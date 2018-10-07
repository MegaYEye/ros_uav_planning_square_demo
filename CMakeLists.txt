cmake_minimum_required(VERSION 2.8.3)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
mavros
roscpp
rospy
std_msgs
nav_msgs
visualization_msgs
tf
)
catkin_package()
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(hole_node src/hole_node.cpp)
target_link_libraries(hole_node ${catkin_LIBRARIES})

