//
// Created by sheni on 10/06/2020.
//

#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rabbitamr_landmarks/LandmarkAction.h>
#include <actionlib/server/simple_action_server.h>

#include <unistd.h>
#include <pwd.h>

class LandmarkAction { // <1>
protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<rabbitamr_landmarks::LandmarkAction> actionServer;
    std::string action_name_;

    rabbitamr_landmarks::LandmarkActionFeedback feedback_;
    rabbitamr_landmarks::LandmarkResult result_;

    geometry_msgs::PoseWithCovarianceStampedConstPtr msg;

    char *fullpath;
    ros::NodeHandle nodeHandle;

public:

    LandmarkAction(const std::string &name) :
            actionServer(nh_, name, boost::bind(&LandmarkAction::executeCB, this, _1), false),
            action_name_(name) {
        this->actionServer.start();

        const char *homedir;
        if ((homedir = getenv("HOME")) == nullptr)
            homedir = getpwuid(getuid())->pw_dir;

        char otherdir[] = "/catkin_ws/src/rabbitamr/rabbitamr_landmarks/config/landmarks.yaml"; // <2>
        this->fullpath = new char[std::strlen(homedir) + std::strlen(otherdir) + 1];
        std::strcpy(this->fullpath, homedir);
        std::strcat(this->fullpath, otherdir);
    }

    ~LandmarkAction() = default;

    void executeCB(const rabbitamr_landmarks::LandmarkGoalConstPtr &goal) { // <3>
        bool success = true;
        this->msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", this->nh_,
                                                                                         ros::Duration(29.0)); //<4>
        ROS_INFO_STREAM("station_name: " << goal->station_name << std::endl);

        if (this->actionServer.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("%s: Preempted", this->action_name_.c_str());
            // set the action state to preempted
            this->actionServer.setPreempted();
            success = false;
        }

        std::string names, p, str;

        if (this->nodeHandle.hasParam("/defined_landmarks/station_names"))
            this->nodeHandle.getParam("/defined_landmarks/station_names", names); //<5>

        if(!names.empty())
            names.append(" ");

        names.append(goal->station_name);
        this->nodeHandle.setParam("/defined_landmarks/station_names", names);

        p = "/defined_landmarks/";
        p.append(goal->station_name); // "/defined_landmarks/yeni_istasyon"


        str = p; //  "/defined_landmarks/yeni_istasyon"
        str.append("/category"); //  "/defined_landmarks/yeni_istasyon/category"
        this->nodeHandle.setParam(str, goal->category);

        str = p;
        str.append("/x");//  "/defined_landmarks/yeni_istasyon/x"
        this->nodeHandle.setParam(str, this->msg->pose.pose.position.x);

        str = p;
        str.append("/y");
        this->nodeHandle.setParam(str, this->msg->pose.pose.position.y);

        str = p;
        str.append("/w");
        this->nodeHandle.setParam(str, this->msg->pose.pose.orientation.w);

        str = p;
        str.append("/z");
        this->nodeHandle.setParam(str, this->msg->pose.pose.orientation.z);

        str = "rosparam dump ";
        str.append(fullpath);
        str.append(" /defined_landmarks");

        system(str.c_str()); // <6>

        this->feedback_.feedback.time_elapsed = ros::Duration(1.0);

        if (success) {
            result_.saved_landmark = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            actionServer.setSucceeded(result_);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_landmark_server");

    LandmarkAction set_landmark_action("set_landmark");
    ros::spin();

    return 0;
}