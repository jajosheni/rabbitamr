cmake_minimum_required(VERSION 3.0.2)
project(rabbitamr_cartographer)
 
find_package(catkin REQUIRED COMPONENTS
  cartographer_ros
  map_server
)  
 
catkin_package( 
)
 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
   