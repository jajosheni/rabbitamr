cmake_minimum_required(VERSION 2.8.3)
project(rabbitamr_base)


find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    roscpp
    )

catkin_package(
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/local/include/modbus
    modbus
)

add_library(rabbitamr_hw_node
    src/rabbitamr_hw_node.cpp
    include/rabbitamr_base/rabbitamr_hw_node.h
    include/rabbitamr_base/WheelJointState.h
)

add_executable(rabbitamr_hardware_interface
    src/rabbitamr_hardware_interface.cpp
)

target_link_libraries(
    rabbitamr_hardware_interface
    rabbitamr_hw_node
    ${catkin_LIBRARIES}
    modbus
    )


add_executable(modbus_con src/modbus_con.cpp)

target_link_libraries(
    modbus_con
    ${catkin_LIBRARIES}
    modbus
)
