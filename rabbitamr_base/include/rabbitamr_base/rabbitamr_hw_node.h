//
// Created by sheni on 02/06/2020.
//

#ifndef ROS_WORKSPACE_RABBITAMR_HW_NODE_H
#define ROS_WORKSPACE_RABBITAMR_HW_NODE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "WheelJointState.h"
#include <ros/ros.h>
#include <modbus/modbus.h>
#include <iostream>

class rabbitamr_hw_node : public hardware_interface::RobotHW {
public:
    rabbitamr_hw_node();

    virtual ~rabbitamr_hw_node() = default;

    void load_params();

    void initEncoders();

    bool modbusConnect();

    void read(ros::Duration &period);

    void write(ros::Duration &period);

private:
    std::string node_name;
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

    WheelJointState left_wheel;
    WheelJointState right_wheel;
    ros::NodeHandle nodeHandle;

    modbus_t *mb;
    std::string plc_ip_connection;
    double wheel_radius;
    double milivolt_to_m_per_second;
    double encoder_ticks_per_rotation;

    uint16_t write_regs[2];
    uint16_t read_regs[8];

    int failed_request_count;
};


#endif //ROS_WORKSPACE_RABBITAMR_HW_NODE_H
