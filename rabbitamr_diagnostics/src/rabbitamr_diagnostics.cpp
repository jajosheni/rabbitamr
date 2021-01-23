#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <string>

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus LDS_state;

int imuReceivedData[2]; // counter and percentage
int ldsReceivedData[2]; // counter and percentage

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, // <1>
                     std::string hardware_id) {
    diag->level = level;
    diag->name = name;
    diag->message = message;
    diag->hardware_id = hardware_id;
}

void setIMUDiagnosis(uint8_t level, std::string message) {
    setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "D435iIMU");
}

void setLDSDiagnosis(uint8_t level, std::string message) {
    setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "SICKLIDAR");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg) { // <2>
    imuReceivedData[0]++;
}

void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ldsReceivedData[0]++;
}

void msgPub(ros::Publisher& rabbitamr_diagnostics_pub) { // <3>
    diagnostic_msgs::DiagnosticArray rabbitamr_diagnostics;

    rabbitamr_diagnostics.header.stamp = ros::Time::now();

    rabbitamr_diagnostics.status.clear();
    rabbitamr_diagnostics.status.push_back(imu_state);
    rabbitamr_diagnostics.status.push_back(LDS_state);

    rabbitamr_diagnostics_pub.publish(rabbitamr_diagnostics);
}

void resetValues(const ros::TimerEvent&){ // <4>
    // calculate and save percentages to [1]
    imuReceivedData[1] = (imuReceivedData[0] * 100)/250; // imu hz rate is 250
    ldsReceivedData[1] = (ldsReceivedData[0] * 100)/24; // lds hz rate is 24
    
    imuReceivedData[0] = 0;
    ldsReceivedData[0] = 0;


    if (imuReceivedData[1] < 5) {
        setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::STALE, "Broken");
    } else if (imuReceivedData[1] > 5 && imuReceivedData[1] < 20) {
        setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "Packet Loss +80%");
    } else if (imuReceivedData[1] > 20 && imuReceivedData[1] < 50) { 
        setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Packet Loss 50-80%");
    } else if (imuReceivedData[1] > 50 && imuReceivedData[1] < 80 ) { 
        setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Packet Loss 20-50%");
    } else {
        setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Running");
    }

    if (ldsReceivedData[1] < 5) {
        setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::STALE, "Broken");
    } else if (ldsReceivedData[1] > 5 && ldsReceivedData[1] < 20) {
        setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "Packet Loss +80%");
    } else if (ldsReceivedData[1] > 20 && ldsReceivedData[1] < 50) { 
        setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Packet Loss 50-80%");
    } else if (ldsReceivedData[1] > 50 && ldsReceivedData[1] < 80 ) { 
        setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Packet Loss 20-50%");
    } else {
        setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Running");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rabbitamr_diagnostics");
    ros::NodeHandle nh;

    ros::Publisher rabbitamr_diagnostics_pub;
    rabbitamr_diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

    ros::Subscriber imu = nh.subscribe("camera/accel/sample", 500, imuMsgCallback);
    ros::Subscriber lds = nh.subscribe("rabbitamr/laser/scan", 50, LDSMsgCallback);

    ros::Timer resetTimer = nh.createTimer(ros::Duration(1), resetValues);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        msgPub(rabbitamr_diagnostics_pub);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

