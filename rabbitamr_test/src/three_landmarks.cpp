//
// Created by sheni on 7/3/20.
//

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <utility>

class Landmark {
public:
    Landmark(float x, float y, float w, float z, std::string goal_name) :
            _x{x}, _y{y}, _w{w}, _z{z}, _goal_name{std::move(goal_name)} {}

    float _x;
    float _y;
    float _w;
    float _z;
    std::string _goal_name;
};


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool sendAGoal(const Landmark &landmark, MoveBaseClient &ac) {
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = landmark._x;
    goal.target_pose.pose.position.y = landmark._y;
    goal.target_pose.pose.orientation.w = landmark._w;
    goal.target_pose.pose.orientation.z = landmark._z;

    ROS_INFO_STREAM("Sending " << landmark._goal_name << std::endl);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM(landmark._goal_name << " reached");
        return true;
    } else
        ROS_INFO("The base failed to reach desired point");

    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "loop_navigation_goals");
    ros::NodeHandle n;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Rate r(10);

    std::vector<Landmark> realFactory;
    realFactory.emplace_back((float) -3.00, (float) -2.05, (float) 0.22, (float) 0.97, "goal1");
    realFactory.emplace_back((float) -4.69, (float)  5.42, (float) 0.68, (float) -0.72, "goal2");
    realFactory.emplace_back((float) 0.055, (float) 1.284, (float) 0.99, (float) -0.02, "goal3");

    std::vector<Landmark> simFactory;
    simFactory.emplace_back((float) -0.99, (float) -2.01, (float) 0.998, (float) 0.056, "simulation goal1");
    simFactory.emplace_back((float) 1.36, (float)  2.85, (float) 0.086, (float) -0.996, "simulation goal2");
    simFactory.emplace_back((float) 3.78, (float) -0.53, (float) 0.02, (float) 0.999, "simulation goal3");

    while (ros::ok()) {

        for(const Landmark &landmark : realFactory){
            if(!sendAGoal(landmark, ac))
                break;
            ros::Duration(10.0).sleep();
        }

        r.sleep();
    }

    return 0;
}
