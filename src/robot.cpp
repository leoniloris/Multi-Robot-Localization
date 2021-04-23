#include "robot.h"

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#include <string>

#include "multi_robot/particles.h"
#include "nav_msgs/Odometry.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#define PUBSUB_QUEUE_SIZE 10

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation);

void Robot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> laser_measurements = scan->ranges;
    ROS_DEBUG_STREAM("90 degrees:" << laser_measurements[90]);
}

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation) {
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    double _roll, _pitch, yaw;
    m.getRPY(_roll, _pitch, yaw);
    ROS_DEBUG_STREAM("robot angle:" << yaw * 180 / PI);

    return yaw;
}

void Robot::odometry_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::Pose2D delta_pose_2d = compute_delta_pose(odom->pose.pose.position, odom->pose.pose.orientation);
    ROS_DEBUG_STREAM("delta pose:" << delta_pose_2d.x << " " << delta_pose_2d.y << " " << delta_pose_2d.theta);
    {
        // TODO: estimate good odometry std's.
        const double std_x = 0.01;
        const double std_y = 0.01;
        const double std_angle = 0;
        particle_filter->move_particles(std_x, std_y, std_angle, delta_pose_2d.x, delta_pose_2d.y, delta_pose_2d.theta);
        //// particle_filter->measure();
        //// particle_filter->update_weights_from_measurements();
    }
    static int a = 0;
    if ((a++ % 10) == 0) {
        broadcast_particles();
    }
}

geometry_msgs::Pose2D Robot::compute_delta_pose(geometry_msgs::Point point, geometry_msgs::Quaternion orientation) {
    double current_yaw = get_yaw_from_orientation(orientation);
    geometry_msgs::Pose2D current_pose_2d;
    geometry_msgs::Pose2D delta_pose_2d;
    current_pose_2d.x = point.x;
    current_pose_2d.y = point.y;
    current_pose_2d.theta = current_yaw;

    if (previous_pose_2d == nullptr) {
        previous_pose_2d = new geometry_msgs::Pose2D(current_pose_2d);
    }

    delta_pose_2d.x = current_pose_2d.x - previous_pose_2d->x;
    delta_pose_2d.y = current_pose_2d.y - previous_pose_2d->y;
    delta_pose_2d.theta = current_pose_2d.theta - previous_pose_2d->theta;

    *previous_pose_2d = current_pose_2d;
    return delta_pose_2d;
}

Robot::Robot(uint8_t robot_index, int argc, char** argv) {
    this->robot_index = robot_index;
    std::string robot_suffix = std::to_string(robot_index);
    ros::init(argc, argv, "robot_node" + robot_suffix);
    ros::NodeHandle node_handle;
    particle_filter = new ParticleFilter(1);

    std::string laser_topic = "/ugv" + robot_suffix + "/scan";
    std::string odometry_topic = "/ugv" + robot_suffix + "/odom";

    laser_scan = node_handle.subscribe<sensor_msgs::LaserScan>(laser_topic, PUBSUB_QUEUE_SIZE, &Robot::laser_callback, this);
    odometry = node_handle.subscribe<nav_msgs::Odometry>(odometry_topic, PUBSUB_QUEUE_SIZE, &Robot::odometry_callback, this);
    broadcaster = node_handle.advertise<multi_robot::particles>("particles_broadcast", PUBSUB_QUEUE_SIZE);
}

void Robot::broadcast_particles() {
    // ROS_INFO_STREAM("broadcasting particles");
    multi_robot::particles particles;
    particles.robot_index = robot_index;
    particle_filter->encode_particles_to_publish(particles);
    ROS_INFO_STREAM("particle: x: " << particles.particles[0].x << " y: " << particles.particles[0].y << " angle: " << particles.particles[0].angle << " id: " << particles.particles[0].id);
    broadcaster.publish(particles);
}

Robot::~Robot() {
    delete particle_filter;
    delete previous_pose_2d;
}
