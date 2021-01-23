//
// Created by sheni on 3/12/20.
//

#include "rabbitamr_base/modbus_con.h"
#include <modbus/modbus.h>
#include "ros/ros.h"

std::string node_name = "modbus_con";

int main(int argc, char **argv) {

    ros::init(argc, argv, "modbus_diagnostic");
    ros::NodeHandle n;
    ros::Rate rate(10);

    std::string ip;

    if (n.hasParam("rabbitamr_diff_drive_controller/plc_ip_connection"))
        n.getParam("rabbitamr_diff_drive_controller/plc_ip_connection", ip);
    else
        ip = "192.168.5.200";

    mb = modbus_new_tcp(ip.c_str(), 502);
    if (modbusConnect()) {
        ROS_INFO_STREAM(node_name << ": PLC Connection established." << std::endl);
    } else {
        ROS_ERROR_STREAM(node_name << ": Couldn't connect to PLC, exiting." << std::endl);
        exit(1);
    }

    int write_registers, read_registers;
    int timeoutCounter = 0;
    int coil_addr = 1;
    bool value = false;
    uint8_t received;

    //run only once each start and reset encoders
    write_registers = modbus_write_bit(mb, 2, true);
    sleep(1);
    write_registers = modbus_write_bit(mb, 2, false);
    if (write_registers == -1)
        ROS_ERROR("Encoders did NOT successfully reset");
    else
        ROS_INFO("Encoders reset");

    while (ros::ok()) {

        write_registers = modbus_write_bit(mb, coil_addr, value);
        read_registers = modbus_read_bits(mb, coil_addr, 1, &received);

        if (write_registers == -1 || read_registers == -1) { // <1>
            timeoutCounter++;
            if (timeoutCounter > 10) { // 10 loops / 10 Hz = 1 second
                if (modbusConnect()) {
                    timeoutCounter = 0;
                }
            }
        }

        value = !value;
        rate.sleep();
    }

    modbus_close(mb);
    modbus_free(mb);
}


bool modbusConnect() {
    if (modbus_connect(mb) == -1) {
        ROS_WARN_STREAM(node_name << ": Connection failed - " << modbus_strerror(errno) << std::endl);
        modbus_close(mb);
        return false;
    }
    return true;
}
