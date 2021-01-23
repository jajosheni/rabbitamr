//
// Created by sheni on 02/06/2020.
//

#ifndef ROS_WORKSPACE_WHEELJOINTSTATE_H
#define ROS_WORKSPACE_WHEELJOINTSTATE_H

#include <iostream>

class WheelJointState {
public:

    double command;
    double pose;
    double velocity;
    double effort;
    std::string joint_name;

    int64_t encoder;
    int64_t en_diff;

    void init(const std::string &jointName) {
        command = 0.0;
        pose = 0.0;
        velocity = 0.0;
        effort = 0.0;
        joint_name = jointName;
        encoder = 0;
        en_diff = 0;
    }

    std::string toString() const{
        std::ostringstream ss;
        ss << this->joint_name << "\n" <<
            "\tcommand: " << this->command <<
            "\tpose: " << this->pose <<
            "\tvelocity: " << this->velocity <<
            "\tencoder: " << this->encoder <<
            "\ten_diff: " << this->en_diff << std::endl;

        return ss.str();
    }
};


#endif //ROS_WORKSPACE_WHEELJOINTSTATE_H
