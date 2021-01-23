//
// Created by sheni on 2/21/20.
//

#include "rabbitamr_base/rabbitamr_hw_node.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rabbitamr_hardware_interface_node");
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    int freq = 20;
    if (nh.hasParam("rabbitamr_joint_state_controller/publish_rate"))
        nh.getParam("rabbitamr_joint_state_controller/publish_rate", freq);

    ros::Rate rate(freq);

    rabbitamr_hw_node rabbitAMR; // <1>
    controller_manager::ControllerManager cm(&rabbitAMR, nh); // <2>

    ros::AsyncSpinner spinner(4, &queue); // <3>
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Time time_now;
    ros::Duration period;

    while (ros::ok())
    {
        time_now = ros::Time::now();
        period = time_now - prev_time;
        prev_time = time_now;

        rabbitAMR.read(period); // <4>
        cm.update(time_now, period); // <5>
        rabbitAMR.write(period); // <6>

        rate.sleep();
    }

    spinner.stop();

    return 0;
}
