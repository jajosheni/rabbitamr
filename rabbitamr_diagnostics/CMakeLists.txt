cmake_minimum_required(VERSION 3.0.2)
project(rabbitamr_diagnostics)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  )


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(diagnostics_node ../rabbitamr_diagnostics/src/rabbitamr_diagnostics.cpp)
add_dependencies(diagnostics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  diagnostics_node
  ${catkin_LIBRARIES}
)

add_executable(rabbitamr_connections ../rabbitamr_diagnostics/src/rabbitamr_connections.cpp)
target_link_libraries(
  rabbitamr_connections
  ${catkin_LIBRARIES}
)
