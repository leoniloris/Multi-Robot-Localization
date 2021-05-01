#include "robot.h"

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#include <string>

#include "multi_robot_localization/particles.h"
#include "nav_msgs/Odometry.h"
#include "occupancy_grid.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#define PUBSUB_QUEUE_SIZE 10
#define SENSOR_OFFSET_METERS 0.16

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation);

void Robot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_meters) {
    std::vector<double> robot_measurements = select_robot_measurements(scan_meters);
    particle_filter->estimate_measurements(meters_to_cells(SENSOR_OFFSET_METERS));
    particle_filter->update_weights_from_robot_measurements(robot_measurements);
    broadcast_particles();

    particle_filter->resample_particles();
}

std::vector<double> Robot::select_robot_measurements(const sensor_msgs::LaserScan::ConstPtr& scan_meters) {
    static const double laser_max_range_meters = meters_to_cells(LASER_MAX_RANGE_METERS);
    std::vector<double> selected_measurements;

    if (previous_pose_2d != nullptr) {
        for (auto measurement_angle_degrees : measurement_angles_degrees) {
            const double range = meters_to_cells(scan_meters->ranges[measurement_angle_degrees]);
            selected_measurements.push_back(range < laser_max_range_meters ? range : laser_max_range_meters);
        }
    }
    return selected_measurements;
}

void Robot::odometry_callback(const nav_msgs::Odometry::ConstPtr& odom_meters) {
    geometry_msgs::Pose2D delta_pose_2d_meters = compute_delta_pose(odom_meters->pose.pose.position, odom_meters->pose.pose.orientation);
    geometry_msgs::Pose2D delta_pose_2d_cells = meters_to_cells(delta_pose_2d_meters);

    double forward_movement = L2_DISTANCE(delta_pose_2d_cells.x, delta_pose_2d_cells.y);

    bool is_moving_forward = INNER_PRODUCT(sin(current_angle - PI / 2), -delta_pose_2d_cells.x, cos(current_angle - PI / 2), delta_pose_2d_cells.y) > 0;
    forward_movement = is_moving_forward ? forward_movement : -forward_movement;
    particle_filter->move_particles(forward_movement, delta_pose_2d_cells.theta);


    // printf("%f,%f,%f\n",meters_to_cells(*previous_pose_2d).x, meters_to_cells(*previous_pose_2d).y, previous_pose_2d->theta);
}

geometry_msgs::Pose2D Robot::compute_delta_pose(geometry_msgs::Point point, geometry_msgs::Quaternion orientation) {
    current_angle = get_yaw_from_orientation(orientation);
    geometry_msgs::Pose2D current_pose_2d;
    geometry_msgs::Pose2D delta_pose_2d;
    current_pose_2d.x = point.x;
    current_pose_2d.y = point.y;
    current_pose_2d.theta = current_angle;

    if (previous_pose_2d == nullptr) {
        previous_pose_2d = new geometry_msgs::Pose2D(current_pose_2d);
    }

    delta_pose_2d.x = current_pose_2d.x - previous_pose_2d->x;
    delta_pose_2d.y = current_pose_2d.y - previous_pose_2d->y;
    delta_pose_2d.theta = current_pose_2d.theta - previous_pose_2d->theta;

    *previous_pose_2d = current_pose_2d;
    return delta_pose_2d;
}

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation) {
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    double _roll, _pitch, yaw;
    m.getRPY(_roll, _pitch, yaw);
    ROS_DEBUG_STREAM("robot angle:" << yaw * 180 / PI);

    return yaw;
}

Robot::Robot(uint8_t robot_index, int argc, char** argv) {
    this->robot_index = robot_index;
    std::string robot_suffix = std::to_string(robot_index);
    ros::init(argc, argv, "robot_node" + robot_suffix);
    ros::NodeHandle node_handle;

    particle_filter = new ParticleFilter(2000, measurement_angles_degrees);

    std::string laser_topic = "/ugv" + robot_suffix + "/scan";
    std::string odometry_topic = "/ugv" + robot_suffix + "/odom";

    laser_scan = node_handle.subscribe<sensor_msgs::LaserScan>(laser_topic, PUBSUB_QUEUE_SIZE, &Robot::laser_callback, this);
    odometry = node_handle.subscribe<nav_msgs::Odometry>(odometry_topic, PUBSUB_QUEUE_SIZE, &Robot::odometry_callback, this);
    broadcaster = node_handle.advertise<multi_robot_localization::particles>("particles_broadcast", PUBSUB_QUEUE_SIZE);
}

void Robot::broadcast_particles() {
    static int a = 0;
    if ((a++ % 30) != 0) {
        return;
    }  // a little downsampling to test.

    printf("broadcasting particles\n");
    multi_robot_localization::particles particles;
    particles.robot_index = robot_index;
    particle_filter->encode_particles_to_publish(particles);

    multi_robot_localization::particle robot_particle;
    robot_particle.x = meters_to_cells(previous_pose_2d->x);
    robot_particle.y = meters_to_cells(previous_pose_2d->y);
    robot_particle.angle = current_angle;
    robot_particle.type = ROBOT;
    // robot_particle.id = p.id;
    // robot_particle.weight = p.weight;
    // for (auto measurement : robot measurements) {
    //     robot_particle.measurements.push_back(measurement);
    // }
    particles.particles.push_back(robot_particle);

    broadcaster.publish(particles);
}

Robot::~Robot() {
    delete particle_filter;
    delete previous_pose_2d;
}
