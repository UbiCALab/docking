cmake_minimum_required(VERSION 2.8.3)
project(sergi)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(hello hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})
