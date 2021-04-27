#pragma once

#include <geometry_msgs/Pose2D.h>

#include <optional>

#include "math_utilities.h"
#include "nav_msgs/Odometry.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define POSITION_STD_ODOMETRY 0.1
#define ANGLE_STD_ODOMETRY 0.0002
#define LASER_SCAN_STD 40

enum ParticleType {
    ROBOT,
    PARTICLE,
};

class Robot {
   private:
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    std::vector<double> select_robot_measurements(const sensor_msgs::LaserScan::ConstPtr& scan_meters);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& scan);
    geometry_msgs::Pose2D compute_delta_pose(geometry_msgs::Point point, geometry_msgs::Quaternion orientation);

    uint8_t robot_index;
    ros::NodeHandle* node_handle;
    ros::Subscriber laser_scan;
    ros::Subscriber odometry;
    ros::Publisher broadcaster;
    geometry_msgs::Pose2D* previous_pose_2d = nullptr; // Not to be used, just for the robot simulation
    double current_angle; // Not to be used, just for the robot simulation
    std::vector<uint16_t> measurement_angles_degrees{300, 315, 330, 345, 0, 15, 30, 45, 60};

   public:
    Robot(uint8_t robot_index, int argc, char** argv);
    void broadcast_particles();
    ~Robot();
    ParticleFilter* particle_filter = nullptr;
};
