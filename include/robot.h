#pragma once

#include <geometry_msgs/Pose2D.h>

#include <optional>

#include "nav_msgs/Odometry.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Robot {
   private:
    void laser_measurement_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
    void odometry_cb(const nav_msgs::Odometry::ConstPtr& scan);
    geometry_msgs::Pose2D compute_delta_odometry(geometry_msgs::Point point, geometry_msgs::Quaternion orientation);

    ros::Subscriber laser_scan;
    ros::Subscriber odometry;

    ros::Publisher broadcaster;

    geometry_msgs::Pose2D* previous_pose_2d = nullptr;

   public:
    ParticleFilter* particle_filter = nullptr;
    Robot(std::string robot_suffix, int argc, char** argv);
    ~Robot();
};
