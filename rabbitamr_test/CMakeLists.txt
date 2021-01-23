cmake_minimum_required(VERSION 3.0.2)
project(rabbitamr_test)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  move_base_msgs
  roscpp
  actionlib
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(three_landmarks src/three_landmarks.cpp)
target_link_libraries(three_landmarks ${catkin_LIBRARIES})
