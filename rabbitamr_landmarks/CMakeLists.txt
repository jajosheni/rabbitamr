cmake_minimum_required(VERSION 3.0.2)
project(rabbitamr_landmarks)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  move_base_msgs
)
    
add_action_files(
  FILES
  Landmark.action
)
 
generate_messages(
  DEPENDENCIES
  actionlib_msgs
) 

catkin_package( 
)
 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
) 

add_executable(set_landmark_server src/set_landmark_server.cpp)
add_executable(set_landmark_client src/set_landmark_client.cpp)  
add_dependencies(set_landmark_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(set_landmark_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_landmark_server  ${catkin_LIBRARIES})
target_link_libraries(set_landmark_client  ${catkin_LIBRARIES} )
  