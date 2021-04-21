#include "robot.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#define MESSAGE_QUEUE_SIZE 10

void Robot::laser_measurement_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> laser_measurements = scan->ranges;

    ROS_INFO_STREAM("90 graus:" << laser_measurements[90]);
    ROS_INFO_STREAM("180 graus:" << laser_measurements[180]);
    ROS_INFO_STREAM("0 graus:" << laser_measurements[0]);
}

Robot::Robot(std::string robot_suffix, int argc, char** argv) {
    ros::init(argc, argv, "robot_node" + robot_suffix);
    ros::NodeHandle node_hanle;
    std::string laser_topic = "/ugv" + robot_suffix + "/scan";
    laser_scan = node_hanle.subscribe<sensor_msgs::LaserScan>(laser_topic, MESSAGE_QUEUE_SIZE, &Robot::laser_measurement_cb, this);
    broadcaster = node_hanle.advertise<std_msgs::String>("broadcast", MESSAGE_QUEUE_SIZE);
}

Robot::~Robot() {
}
