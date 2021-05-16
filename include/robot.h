#pragma once

#include <geometry_msgs/Pose2D.h>

#include <optional>
#include <unordered_map>

#include "math_utilities.h"
#include "multi_robot_localization/clusters.h"
#include "nav_msgs/Odometry.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define POSITION_STD_ODOMETRY (10.0 / 10.0)
#define ANGLE_STD_ODOMETRY (0.5 / 10.0)
#define LASER_SCAN_STD (10.0 / 10.0)
#define LASER_MAX_RANGE_METERS 3
#define DETECTION_THRESHOLD_METERS 2.5

enum ParticleType {
    ROBOT,
    PARTICLE,
    CLUSTER,
};

typedef struct _Detection {
    double distance;
    double angle;
    std::vector<Particle> clusters;
} Detection;

class Robot {
   private:
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void update_measurements(const sensor_msgs::LaserScan::ConstPtr& scan_meters);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& scan);
    geometry_msgs::Pose2D compute_delta_pose(geometry_msgs::Point point, geometry_msgs::Quaternion orientation);
    multi_robot_localization::particle get_robot_particle_to_publish();
    void detector_callback(const multi_robot_localization::clusters::ConstPtr& other_robot_clusters_ptr);
    bool has_detected(double x, double y, Detection& detection);
    void publush_clusters();
    void update_particle_filter_based_on_detections();

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
    std::unordered_map<uint16_t, Detection> robot_detections;

   public:
    Robot(uint8_t robot_index, int argc, char** argv);
    void broadcast_particles();
    ~Robot();
    ParticleFilter* particle_filter = nullptr;
};
