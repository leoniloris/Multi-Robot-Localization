#include "robot.h"

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#define MESSAGE_QUEUE_SIZE 10

void Robot::laser_measurement_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> laser_measurements = scan->ranges;
    ROS_DEBUG_STREAM("90 degrees:" << laser_measurements[90]);
}

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation) {
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    double _roll, _pitch, yaw;
    m.getRPY(_roll, _pitch, yaw);
    ROS_DEBUG_STREAM("robot angle:" << yaw * 180 / 3.14159265359);

    return yaw;
}

geometry_msgs::Pose2D Robot::compute_delta_odometry(geometry_msgs::Point point, geometry_msgs::Quaternion orientation) {
    ROS_INFO_STREAM("previous pose:" << previous_pose_2d->x << " " << previous_pose_2d->y << " " << previous_pose_2d->theta);

    double current_yaw = get_yaw_from_orientation(orientation);
    geometry_msgs::Pose2D current_pose_2d;
    geometry_msgs::Pose2D delta_pose_2d;
    current_pose_2d.x = point.x;
    current_pose_2d.y = point.y;
    current_pose_2d.theta = current_yaw;
    ROS_INFO_STREAM("current pose:" << current_pose_2d.x << " " << current_pose_2d.y << " " << current_pose_2d.theta);

    if (previous_pose_2d == nullptr) {
        previous_pose_2d = new geometry_msgs::Pose2D(current_pose_2d);
    }

    delta_pose_2d.x = current_pose_2d.x - previous_pose_2d->x;
    delta_pose_2d.y = current_pose_2d.y - previous_pose_2d->y;
    delta_pose_2d.theta = current_pose_2d.theta - previous_pose_2d->theta;

    *previous_pose_2d = current_pose_2d;
    return delta_pose_2d;
}

void Robot::odometry_cb(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Pose2D delta_pose_2d = compute_delta_odometry(odom->pose.pose.position, odom->pose.pose.orientation);
    ROS_INFO_STREAM("delta pose:" << delta_pose_2d.x << " " << delta_pose_2d.y << " " << delta_pose_2d.theta);
}

Robot::Robot(std::string robot_suffix, int argc, char** argv) {
    ros::init(argc, argv, "robot_node" + robot_suffix);
    ros::NodeHandle node_hanle;

    particle_filter = new ParticleFilter(100, 1415, 2026, 6.28318531);

    std::string laser_topic = "/ugv" + robot_suffix + "/scan";
    std::string odometry_topic = "/ugv" + robot_suffix + "/odom";

    laser_scan = node_hanle.subscribe<sensor_msgs::LaserScan>(laser_topic, MESSAGE_QUEUE_SIZE, &Robot::laser_measurement_cb, this);
    odometry = node_hanle.subscribe<nav_msgs::Odometry>(odometry_topic, MESSAGE_QUEUE_SIZE, &Robot::odometry_cb, this);

    broadcaster = node_hanle.advertise<std_msgs::String>("broadcast", MESSAGE_QUEUE_SIZE);
}

Robot::~Robot() {
    delete particle_filter;
    delete previous_pose_2d;
}
