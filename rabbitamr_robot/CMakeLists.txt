cmake_minimum_required(VERSION 2.8.3)
project(rabbitamr_robot)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)