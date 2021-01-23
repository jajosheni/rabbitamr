//
// Created by sheni on 7/8/20.
//

#include "ros/ros.h"
#include <iostream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include "std_msgs/String.h"

std::string exec(const char *cmd) {
    std::array<char, 256> buffer{};
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rabbitamr_connections");
    ros::NodeHandle nh;

//    ros::Publisher publisher;
//    publisher = nh.advertise<std_msgs::String>("connected_devices", 10);

    ros::Rate loop_rate(1); // every 1 second

    std::string d435i = "8086:0b3a Intel Corp.";
    std::string wifi = "Access Point: FC:4A:E9:73:F2:9E";
    std::string output;

    while (ros::ok()) {
        output = exec("lsusb");

        if (output.find(d435i) != std::string::npos)
            std::cout << "intel realsense d435i is connected" << std::endl;
        else
            std::cout << "intel realsense d435i is not connected" << std::endl;

        output = exec("iwconfig");

        if (output.find(wifi) != std::string::npos)
            std::cout << "wifi ACCESSPOINTNAME is connected" << std::endl;
        else
            std::cout << "wifi ACCESSPOINTNAME is not connected" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
