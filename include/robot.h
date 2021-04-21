#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Robot {
   private:
    ros::Subscriber laser_scan;
    ros::Publisher broadcaster;

   public:
    Robot(std::string robot_suffix, int argc, char** argv);
    ~Robot();
    void laser_measurement_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
};
