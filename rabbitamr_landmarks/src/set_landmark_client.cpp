//
// Created by sheni on 10/06/2020.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rabbitamr_landmarks/LandmarkAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<rabbitamr_landmarks::LandmarkAction> LandmarkClient;

class Landmark {
public:
    Landmark() : _x{0.0}, _y{0.0}, _w{0.0}, _z{0.0}, _station_name{"station_name"}, _category{"category"} {}

    Landmark(float x, float y, float w, float z, std::string station_name, std::string category) :
            _x{x}, _y{y}, _w{w}, _z{z}, _station_name{std::move(station_name)}, _category{std::move(category)} {}

    float _x;
    float _y;
    float _w;
    float _z;
    std::string _station_name;
    std::string _category;
};

class ModelA {
protected:
    std::vector<Landmark> landmarks;
    ros::NodeHandle nodeHandle;
    std::string nameSpace;
    geometry_msgs::PoseWithCovarianceStampedConstPtr msg;

public:
    ModelA() {
        this->nameSpace = "/defined_landmarks/";
    }

    static void showMenu() { // <4>
        std::cout << "\n" << "MENU\n" <<
                  "Type the number and press enter: \n" <<
                  "1. Reload Landmarks\n" <<
                  "2. List Landmarks\n" <<
                  "3. Save a Landmark\n" <<
                  "4. Start a Load Unload Operation\n" <<
                  "5. Show this menu\n" <<
                  "6. Exit" << std::endl;
    }

    void loadLandmarks() { // <5>
        this->landmarks.clear();

        std::string names, p;
        p = this->nameSpace;
        p.append("station_names"); // "/defined_landmarks/load1"

        if (this->nodeHandle.hasParam(p))
            this->nodeHandle.getParam(p, names);
        else
            return;

        p = this->nameSpace;
        std::istringstream ss(names);
        std::string _p[4] = {"/x", "/y", "/w", "/z"};
        double _v[4];
        do {
            Landmark lm;
            ss >> lm._station_name;
            if (lm._station_name == "station_name")
                continue;

            for (int i = 0; i < 4; i++) {
                std::string str = p;
                str.append(lm._station_name);
                str.append(_p[i]); // defined_landmarks/load1/x
                if (this->nodeHandle.hasParam(str))
                    this->nodeHandle.getParam(str, _v[i]);
                else
                    _v[i] = 0.0;
            }

            lm._x = float(_v[0]);
            lm._y = float(_v[1]);
            lm._w = float(_v[2]);
            lm._z = float(_v[3]);

            std::string str = p;
            str.append(lm._station_name);
            str.append("/category");
            if (this->nodeHandle.hasParam(str))
                this->nodeHandle.getParam(str, lm._category);
            else
                lm._category = "category";

            this->landmarks.push_back(lm);
        } while (ss);
    }

    void listLandmarks(const std::string &filter) { // <6>
        this->loadLandmarks();
        std::cout << "Station\t\tX\t\tY\t\tW\t\tZ\t\tCategory" << std::endl;

        for (int i = 0; i < this->landmarks.size(); i++) {
            if (filter == landmarks[i]._category || filter.empty())
                std::cout << i << ". " <<
                          landmarks[i]._station_name << "\t" <<
                          "x: " << landmarks[i]._x << "\t" <<
                          "y: " << landmarks[i]._y << "\t" <<
                          "w: " << landmarks[i]._w << "\t" <<
                          "z: " << landmarks[i]._z << "\t" <<
                          "c: " << landmarks[i]._category << std::endl;
        }
    }

    static void saveLandmark(LandmarkClient &land_ac) { // <7>

        rabbitamr_landmarks::LandmarkGoal goal;
        std::cout << "Enter station name and press enter: ";
        std::cin >> goal.station_name;
        for (auto &c : goal.station_name)
            c = tolower(c);

        std::cout << "Enter station category(load/unload/charge) and press enter: ";
        std::cin >> goal.category;
        for (auto &c : goal.category)
            c = tolower(c);

        land_ac.sendGoal(goal); //<2>

        bool finished_before_timeout = land_ac.waitForResult(ros::Duration(30.0)); // <3>

        if (finished_before_timeout) // <4>
        {
            actionlib::SimpleClientGoalState state = land_ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else
            ROS_INFO("Action did not finish before the time out.");
    }

    void offsetGoal(Landmark &landmark) {
        // listen for a message only
        this->msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", this->nodeHandle,
                                                                                         ros::Duration(5.0));

        // construct the equation
        double x1, y1, x2, y2, m, b;
        x1 = this->msg->pose.pose.position.x;
        y1 = this->msg->pose.pose.position.y;
        x2 = landmark._x;
        y2 = landmark._y;

        // m = (y2 − y1) / (x2 - x1)
        // y = mx + b

        m = (y2 - y1) / (x2 - x1);
        b = y1 - (m * x1);

        
    }

    bool sendAGoal(const Landmark &landmark, MoveBaseClient &move_ac, bool redirection) { // <8>
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        Landmark offsetPoint = landmark;
        if (redirection)
            this->offsetGoal(offsetPoint);

        goal.target_pose.pose.position.x = offsetPoint._x;
        goal.target_pose.pose.position.y = offsetPoint._y;
        goal.target_pose.pose.orientation.w = offsetPoint._w;
        goal.target_pose.pose.orientation.z = offsetPoint._z;

        ROS_INFO_STREAM("Sending " << landmark._station_name << std::endl);
        move_ac.sendGoal(goal);

        move_ac.waitForResult();

        if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (redirection){
                this->sendAGoal(landmark, move_ac, false);
                ROS_INFO_STREAM(landmark._station_name << " preparing...");
            } else ROS_INFO_STREAM(landmark._station_name << " reached.");

            return true;
        } else
            ROS_INFO("The base failed to reach desired point");

        return false;
    }

    void loadUnload(MoveBaseClient &move_ac) { // <9>
        std::string load, unload;

        this->listLandmarks("load");
        std::cout << "Please select a load landmark: " << std::endl;
        std::cin >> load;

        this->listLandmarks("unload");
        std::cout << "Please select an unload landmark: " << std::endl;
        std::cin >> unload;

        int l, u;
        try {
            l = std::stoi(load);
            u = std::stoi(unload);
        } catch (int e) {
            std::cout << "Please enter valid numbers" << std::endl;
            this->loadUnload(move_ac);
        }

        if (this->sendAGoal(this->landmarks[l], move_ac, true)) {
            std::cout << "Taking loads." << std::endl;
            ros::Duration(10).sleep(); // waitbutton
            if (this->sendAGoal(this->landmarks[u], move_ac, true)) {
                std::cout << "Unloading." << std::endl;
                ros::Duration(10).sleep();
            }
        }
        std::cout << "Done." << std::endl;
        ModelA::showMenu();
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_landmark");

    LandmarkClient land_ac("set_landmark", true); // <1>
    MoveBaseClient move_ac("move_base", true); // <2>

    ROS_INFO("Waiting for Landmark action server to start.");
    land_ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Landmark action server started.");

    ROS_INFO("Waiting for MoveBase action server to start.");
    move_ac.waitForServer(); //will wait for infinite time
    ROS_INFO("MoveBase action server started.");

    ros::Rate rate(10);

    ModelA modelA;
    ModelA::showMenu();
    std::string command;
    int cmd;

    while (ros::ok()) { // <3>
        std::cout << "\nEnter Command no: " << std::endl;
        std::cin >> command;
        try {
            cmd = std::stoi(command);
        } catch (int e) {
            std::cout << "Please enter a valid number" << std::endl;
            continue;
        }

        switch (cmd) {
            case 1:
                modelA.loadLandmarks();
                break;
            case 2:
                modelA.listLandmarks("");
                break;
            case 3:
                ModelA::saveLandmark(land_ac);
                break;
            case 4:
                modelA.loadUnload(move_ac);
                break;
            case 5:
                ModelA::showMenu();
                break;
            case 6:
                return 0;
            default:
                std::cout << "Please enter a number shown in menu." << std::endl;
                break;
        }
        rate.sleep();
    }

    return 0;
}

