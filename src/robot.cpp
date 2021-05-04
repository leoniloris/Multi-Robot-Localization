#include "robot.h"

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#include <string>
#include <unordered_map>

#include "multi_robot_localization/InfoForDetector.h"
#include "multi_robot_localization/particles.h"
#include "nav_msgs/Odometry.h"
#include "occupancy_grid.h"
#include "particle_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#define PUBSUB_QUEUE_SIZE 10

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation);

void Robot::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_meters) {
    update_measurements(scan_meters);
    particle_filter->estimate_measurements();
    particle_filter->update_weights_from_robot_measurements(robot_measurements);

    //update_weights(); //Acessa o dicionario de robot detections e chama funcoes do particle_filters

    broadcast_particles();

    particle_filter->resample_particles();
}

//void Robot::update_weights() {
//Chama funcao
//}
void Robot::update_measurements(const sensor_msgs::LaserScan::ConstPtr& scan_meters) {
    static const double laser_max_range = meters_to_cells(LASER_MAX_RANGE_METERS);

    if (previous_pose_2d != nullptr) {
        for (uint16_t measurement_idx = 0; measurement_idx < robot_measurements.size(); measurement_idx++) {
            const uint16_t measurement_angle_degrees = measurement_angles_degrees[measurement_idx];
            const double distance = meters_to_cells(scan_meters->ranges[measurement_angle_degrees]);
            robot_measurements[measurement_idx] = distance < laser_max_range ? distance : laser_max_range;
        }
    }
}

void Robot::odometry_callback(const nav_msgs::Odometry::ConstPtr& odom_meters) {
    static geometry_msgs::Point position;
    position.x = meters_to_cells(odom_meters->pose.pose.position.x) + X_CENTER;
    position.y = meters_to_cells(odom_meters->pose.pose.position.y) + Y_CENTER;
    geometry_msgs::Pose2D delta_pose_2d = compute_delta_pose(position, odom_meters->pose.pose.orientation);

    double forward_movement = L2_DISTANCE(delta_pose_2d.x, delta_pose_2d.y);
    bool is_moving_forward = INNER_PRODUCT(sin(current_angle - PI / 2), -delta_pose_2d.x, cos(current_angle - PI / 2), delta_pose_2d.y) > 0;
    forward_movement = is_moving_forward ? forward_movement : -forward_movement;
    particle_filter->move_particles(forward_movement, delta_pose_2d.theta);

    multi_robot_localization::InfoForDetector my_info;
    my_info.id = robot_index;
    my_info.x = previous_pose_2d->x;
    my_info.y = previous_pose_2d->y;
    my_info.angle = previous_pose_2d->theta;
    //my_info.cluster.push_back(x, y, peso)

    infos_to_detector.publish(my_info);
}

void Robot::detector_clbk(const multi_robot_localization::InfoForDetector::ConstPtr& _robot_info) {
    multi_robot_localization::InfoForDetector robot_info;
    robot_info.id = _robot_info->id;
    robot_info.x = _robot_info->x;
    robot_info.y = _robot_info->y;
    robot_info.angle = _robot_info->angle;

    if ((robot_info.id != robot_index) && has_detected(robot_info)) {
        robot_detections[robot_info.id] = robot_info;
        ROS_INFO_STREAM("DETECTOU ROBO:" << robot_info.id);
    } else if ((robot_info.id != robot_index) && (robot_detections.find(robot_info.id) != robot_detections.end())) {
        robot_detections.erase(robot_info.id);
        ROS_INFO_STREAM("REMOVEU ROBO:" << robot_info.id);
    }
}

bool Robot::has_detected(multi_robot_localization::InfoForDetector other_robot_info) {
    //const bool is_path_free = particle_filter->is_path_free((double)previous_pose_2d->x, (double)previous_pose_2d->y, (double)other_robot_info.x, (double)other_robot_info.y);

    const double robots_distance = L2_DISTANCE((previous_pose_2d->x - other_robot_info.x), (previous_pose_2d->y - other_robot_info.y));
    return (robots_distance < meters_to_cells(DETECTION_THRESHOLD_METERS));  // && is_path_free;
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

    particle_filter = new ParticleFilter(N_PARTICLES, measurement_angles_degrees);

    std::string laser_topic = "/ugv" + robot_suffix + "/scan";
    std::string odometry_topic = "/ugv" + robot_suffix + "/odom";

    laser_scan = node_handle.subscribe<sensor_msgs::LaserScan>(laser_topic, PUBSUB_QUEUE_SIZE, &Robot::laser_callback, this);
    odometry = node_handle.subscribe<nav_msgs::Odometry>(odometry_topic, PUBSUB_QUEUE_SIZE, &Robot::odometry_callback, this);
    broadcaster = node_handle.advertise<multi_robot_localization::particles>("particles_broadcast", PUBSUB_QUEUE_SIZE);

    all_robots_info = node_handle.subscribe<multi_robot_localization::InfoForDetector>("detector_t", PUBSUB_QUEUE_SIZE, &Robot::detector_clbk, this);
    infos_to_detector = node_handle.advertise<multi_robot_localization::InfoForDetector>("detector_t", PUBSUB_QUEUE_SIZE);

    assert(measurement_angles_degrees.size() == robot_measurements.size());
}

void Robot::broadcast_particles() {
    static uint16_t downsample_idx = 0;
    if ((downsample_idx++ % 4) != 0) {
        return;
    }
    multi_robot_localization::particles particles;
    particles.robot_index = robot_index;
    particle_filter->encode_particles_to_publish(particles);

    multi_robot_localization::particle robot_particle = get_robot_particle_to_publish();
    particles.particles.push_back(robot_particle);

    printf("broadcasting %ld particles.\n", particles.particles.size());
    broadcaster.publish(particles);
}

multi_robot_localization::particle Robot::get_robot_particle_to_publish() {
    multi_robot_localization::particle robot_particle;
    robot_particle.x = previous_pose_2d->x;
    robot_particle.y = previous_pose_2d->y;
    robot_particle.angle = current_angle;
    robot_particle.type = ROBOT;
    //// Robot does not need to set weight for now
    // robot_particle.weight = p.weight;
    robot_particle.id = 0xfff0 + robot_index;
    for (auto measurement : robot_measurements) {
        robot_particle.measurements.push_back(measurement);
    }
    return robot_particle;
}

Robot::~Robot() {
    delete particle_filter;
    delete previous_pose_2d;
}
