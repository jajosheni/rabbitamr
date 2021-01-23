//
// Created by sheni on 02/06/2020.
//

#include "rabbitamr_base/rabbitamr_hw_node.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

rabbitamr_hw_node::rabbitamr_hw_node() {

    this->load_params();
    this->initEncoders();

    this->left_wheel.init("left_wheel_joint");
    this->right_wheel.init("right_wheel_joint");

    hardware_interface::JointStateHandle left_state_handle(
        this->left_wheel.joint_name,
        &this->left_wheel.pose,
        &this->left_wheel.velocity,
        &this->left_wheel.effort
    );

    hardware_interface::JointStateHandle right_state_handle(
        this->right_wheel.joint_name,
        &this->right_wheel.pose,
        &this->right_wheel.velocity,
        &this->right_wheel.effort
    );

    joint_state_interface.registerHandle(left_state_handle);
    joint_state_interface.registerHandle(right_state_handle);

    hardware_interface::JointHandle left_handle(
        joint_state_interface.getHandle(this->left_wheel.joint_name), &this->left_wheel.command);

    hardware_interface::JointHandle right_handle(
        joint_state_interface.getHandle(this->right_wheel.joint_name), &this->right_wheel.command);

    position_joint_interface.registerHandle(left_handle);
    position_joint_interface.registerHandle(right_handle);
    velocity_joint_interface.registerHandle(left_handle);
    velocity_joint_interface.registerHandle(right_handle);
    effort_joint_interface.registerHandle(left_handle);
    effort_joint_interface.registerHandle(right_handle);
    //<1>
    registerInterface(&joint_state_interface);
    registerInterface(&effort_joint_interface);
    registerInterface(&velocity_joint_interface);
    registerInterface(&position_joint_interface);
}

void rabbitamr_hw_node::load_params() {

    this->node_name = ros::this_node::getName();
    if (this->nodeHandle.hasParam("rabbitamr_diff_drive_controller/wheel_radius"))
        this->nodeHandle.getParam("rabbitamr_diff_drive_controller/wheel_radius", this->wheel_radius);
    else
        this->wheel_radius = 0.1;

    if (this->nodeHandle.hasParam("rabbitamr_diff_drive_controller/wheel_radius_multiplier")) {
        double wrm;
        this->nodeHandle.getParam("rabbitamr_diff_drive_controller/wheel_radius_multiplier", wrm);
        this->wheel_radius *= wrm;
    }

    if (this->nodeHandle.hasParam("rabbitamr_diff_drive_controller/encoder_ticks_per_rotation"))
        this->nodeHandle.getParam("rabbitamr_diff_drive_controller/encoder_ticks_per_rotation", this->encoder_ticks_per_rotation);
    else
        this->encoder_ticks_per_rotation = 2048.0;

    if (this->nodeHandle.hasParam("rabbitamr_diff_drive_controller/milivolt_to_m_per_second"))
        this->nodeHandle.getParam("rabbitamr_diff_drive_controller/milivolt_to_m_per_second", this->milivolt_to_m_per_second);
    else
        this->milivolt_to_m_per_second = 6600.0;

    if (this->nodeHandle.hasParam("rabbitamr_diff_drive_controller/plc_ip_connection"))
        this->nodeHandle.getParam("rabbitamr_diff_drive_controller/plc_ip_connection", this->plc_ip_connection);
    else
        this->plc_ip_connection = "192.168.5.200";

    ROS_INFO_STREAM(this->node_name << "\n\tParameters loaded : "
                                    << "\n\t\t wheel_radius: " << this->wheel_radius
                                    << "\n\t\t encoder_ticks_per_rotation: " << this->encoder_ticks_per_rotation
                                    << "\n\t\t milivolt_to_m_per_second: " << this->milivolt_to_m_per_second
                                    << "\n\t\t plc_ip_connection: " << this->plc_ip_connection
                                    << std::endl);
}

void rabbitamr_hw_node::initEncoders() {
    //<2>
    mb = modbus_new_tcp(this->plc_ip_connection.c_str(), 502);
    this->failed_request_count = 0;

    if (modbusConnect()) {
        modbus_read_input_registers(mb, 0, 8, this->read_regs);

        this->left_wheel.encoder = MODBUS_GET_INT64_FROM_INT16(this->read_regs, 0);
        this->right_wheel.encoder = MODBUS_GET_INT64_FROM_INT16(this->read_regs, 4);
        ROS_INFO_STREAM(this->node_name << "Connection established." << std::endl);
    } else {
        ROS_ERROR_STREAM(this->node_name
                             << ": Couldn't connect to PLC, exiting." << std::endl);
        exit(1);
    }
}

bool rabbitamr_hw_node::modbusConnect() {
    if (modbus_connect(mb) == -1) {
        ROS_INFO_STREAM(this->node_name
                            << ": Connection failed: " << modbus_strerror(errno) << std::endl);
        modbus_close(mb);
        return false;
    }
    return true;
}

void rabbitamr_hw_node::read(ros::Duration &period) {
//    this->left_wheel.pose += (this->left_wheel.command * period.toSec()); // rad
//    this->left_wheel.velocity = this->left_wheel.command; // rad/s <3>

    if (modbus_read_input_registers(mb, 0, 8, this->read_regs) == -1) {

        this->failed_request_count++;

        if (this->failed_request_count > 150) { // 150 loops / 50 Hz = 3 seconds
            if (modbusConnect()) {
                ROS_INFO_STREAM(this->node_name << ": Connection re-established in reading." << std::endl);
                this->failed_request_count = 0;
            }
        }
        return;
    }

    // get encoder increment since the last loop
    this->left_wheel.en_diff = MODBUS_GET_INT64_FROM_INT16(this->read_regs, 0) - this->left_wheel.encoder;
    this->right_wheel.en_diff = MODBUS_GET_INT64_FROM_INT16(this->read_regs, 4) - this->right_wheel.encoder;

    this->left_wheel.encoder = MODBUS_GET_INT64_FROM_INT16(this->read_regs, 0);
    this->right_wheel.encoder = MODBUS_GET_INT64_FROM_INT16(this->read_regs, 4);

    // encoders total in radians
    this->left_wheel.pose = (this->left_wheel.encoder / this->encoder_ticks_per_rotation) * 2 * M_PI;
    this->right_wheel.pose = (this->right_wheel.encoder / this->encoder_ticks_per_rotation) * 2 * M_PI;

    if (period.toSec() > 0) {
        // angluar velocities of wheels (rad/s)
        this->left_wheel.velocity =
            ((double(this->left_wheel.en_diff) / this->encoder_ticks_per_rotation) * 2 * M_PI) / period.toSec();

        this->right_wheel.velocity =
            ((double(this->right_wheel.en_diff) / this->encoder_ticks_per_rotation) * 2 * M_PI) / period.toSec();
    }

}

void rabbitamr_hw_node::write(ros::Duration &period) {
    // convert to from rad/s to mv <4>
    int left_enc = int(this->milivolt_to_m_per_second *
                       this->left_wheel.command * this->wheel_radius);
    int right_enc = int(this->milivolt_to_m_per_second *
                        this->right_wheel.command * this->wheel_radius);

    this->write_regs[0] = left_enc;
    this->write_regs[1] = right_enc;

    if (modbus_write_registers(mb, 0, 2, write_regs) == -1) {

        this->failed_request_count++;

        if (this->failed_request_count > 150) { // 150 loops / 50 Hz = 3 seconds
            if (modbusConnect()) {
                ROS_INFO_STREAM(this->node_name << ": Connection re-established in writing." << std::endl);
                this->failed_request_count = 0;
            }
        }
    }
}


