#pragma once

#include <geometry_msgs/Pose2D.h>

#include <optional>
#include <unordered_map>

#include "math_utilities.h"
#include "multi_robot_localization/InfoForDetector.h"
#include "nav_msgs/Odometry.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define POSITION_STD_ODOMETRY (10.0 / 10.0)
#define ANGLE_STD_ODOMETRY (0.5 / 10.0)
#define LASER_SCAN_STD (10.0 / 10.0)
#define LASER_MAX_RANGE_METERS 3
#define DETECTION_THRESHOLD_METERS 1

enum ParticleType {
    ROBOT,
    PARTICLE,
};

class Robot {
   private:
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void update_measurements(const sensor_msgs::LaserScan::ConstPtr& scan_meters);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& scan);
    geometry_msgs::Pose2D compute_delta_pose(geometry_msgs::Point point, geometry_msgs::Quaternion orientation);
    multi_robot_localization::particle get_robot_particle_to_publish();
    void detector_clbk(const multi_robot_localization::InfoForDetector::ConstPtr& _robot_info);
    bool has_detected(multi_robot_localization::InfoForDetector other_robot_info);

    uint8_t robot_index;
    ros::NodeHandle* node_handle;
    ros::Subscriber laser_scan;
    ros::Subscriber odometry;
    ros::Subscriber all_robots_info;
    ros::Publisher broadcaster;
    ros::Publisher infos_to_detector;

    geometry_msgs::Pose2D* previous_pose_2d = nullptr;  // Not to be used, just for the robot simulation
    double current_angle;                               // Not to be used, just for the robot simulation
    std::vector<uint16_t> measurement_angles_degrees{0, 45, 90, 135, 180, 225, 270, 315};
    std::vector<double> robot_measurements{0, 0, 0, 0, 0, 0, 0, 0};
    std::unordered_map<uint16_t, multi_robot_localization::InfoForDetector> robot_detections;

   public:
    Robot(uint8_t robot_index, int argc, char** argv);
    void broadcast_particles();
    ~Robot();
    ParticleFilter* particle_filter = nullptr;
};
