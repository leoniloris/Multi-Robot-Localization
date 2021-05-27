#include "robot.h"

#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#include <string>
#include <unordered_map>

#include "minibatch_kmeans.h"
#include "multi_robot_localization/clusters.h"
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

    // the following order ought to be respected, since a few operations rely on the result of others
    particle_filter->estimate_measurements();
    particle_filter->update_weights_from_robot_measurements(robot_measurements);
    update_particle_filter_based_on_detections();
    broadcast_particles();
    particle_filter->resample_particles();

    if (previous_pose_2d.x != UNITIALIZED) {
        double has_detection = (robot_detections.empty() ? 0.0 : 1.0);
        particle_filter->save_state(robot_index, previous_pose_2d.x, previous_pose_2d.y, previous_pose_2d.theta, has_detection);
    }
}

void Robot::update_particle_filter_based_on_detections() {
    for (const auto& other_robot_id_and_detection : robot_detections) {
        particle_filter->update_weights_based_on_detection(
            other_robot_id_and_detection.second.clusters,
            other_robot_id_and_detection.second.distance,
            other_robot_id_and_detection.second.angle);
    }
}

void Robot::update_measurements(const sensor_msgs::LaserScan::ConstPtr& scan_meters) {
    static const double laser_max_range = meters_to_cells(LASER_MAX_RANGE_METERS);

    if (previous_pose_2d.x != UNITIALIZED) {
        for (uint16_t measurement_idx = 0; measurement_idx < robot_measurements.size(); measurement_idx++) {
            const uint16_t measurement_angle_degrees = measurement_angles_degrees[measurement_idx];
            const double distance = meters_to_cells(scan_meters->ranges[measurement_angle_degrees]);
            robot_measurements[measurement_idx] = distance < laser_max_range ? distance : laser_max_range;
        }
    }
}

void Robot::publush_clusters() {
    const vector<Particle>* clusters = kmeans_get_clusters();
    multi_robot_localization::clusters clusters_to_publish;
    clusters_to_publish.origin_robot_index = robot_index;
    clusters_to_publish.origin_robot_x = previous_pose_2d.x;
    clusters_to_publish.origin_robot_y = previous_pose_2d.y;
    clusters_to_publish.origin_robot_angle = current_angle;
    for (auto cluster : (*clusters)) {
        multi_robot_localization::particle cluster_to_publish;
        cluster_to_publish.x = cluster.x;
        cluster_to_publish.y = cluster.y;
        cluster_to_publish.angle = cluster.angle;
        cluster_to_publish.weight = cluster.weight;
        cluster_to_publish.type = CLUSTER;
        clusters_to_publish.clusters.push_back(cluster_to_publish);
    }

    infos_to_detector.publish(clusters_to_publish);
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

    publush_clusters();
}

void Robot::detector_callback(const multi_robot_localization::clusters::ConstPtr& other_robot_clusters_ptr) {
    const bool not_me = other_robot_clusters_ptr->origin_robot_index != robot_index;
    const double x = other_robot_clusters_ptr->origin_robot_x, y = other_robot_clusters_ptr->origin_robot_y;
    const bool robot_already_detected = robot_detections.find(other_robot_clusters_ptr->origin_robot_index) != robot_detections.end();
    Detection detection;

    if (not_me && has_detected(x, y, detection)) {
        for (auto other_robot_cluster : other_robot_clusters_ptr->clusters) {
            Particle cluster_particle;
            cluster_particle.x = other_robot_cluster.x;
            cluster_particle.y = other_robot_cluster.y;
            cluster_particle.angle = other_robot_cluster.angle;
            cluster_particle.weight = other_robot_cluster.weight;

            detection.clusters.push_back(cluster_particle);
        }
        //robot_detections[other_robot_clusters_ptr->origin_robot_index] = detection;
        //printf("Robot %d detected/updated.\n", other_robot_clusters_ptr->origin_robot_index);
    } else if (not_me && robot_already_detected) {
        //robot_detections.erase(other_robot_clusters_ptr->origin_robot_index);
        //printf("Robot %d not being detected anymore.\n", other_robot_clusters_ptr->origin_robot_index);
    }
}

bool Robot::has_detected(double x, double y, Detection& detection) {
    const bool is_path_free = particle_filter->is_path_free(previous_pose_2d.x, previous_pose_2d.y, x, y);
    const double dx = (previous_pose_2d.x - x);
    const double dy = (previous_pose_2d.y - y);
    const double robots_distance = L2_DISTANCE(dx, dy);

    detection.distance = robots_distance;
    detection.angle = atan(dy / (dx + EPS));

    const double random_percentage = (double)rand() / RAND_MAX;
    const double distance_likelihood = GAUSSIAN_LIKELIHOOD(0, meters_to_cells(DETECTION_THRESHOLD_METERS), robots_distance);
    const bool probably_detected = distance_likelihood > random_percentage;
    return probably_detected && is_path_free;
}

geometry_msgs::Pose2D Robot::compute_delta_pose(geometry_msgs::Point point, geometry_msgs::Quaternion orientation) {
    current_angle = get_yaw_from_orientation(orientation);
    geometry_msgs::Pose2D current_pose_2d;
    geometry_msgs::Pose2D delta_pose_2d;
    current_pose_2d.x = point.x;
    current_pose_2d.y = point.y;
    current_pose_2d.theta = current_angle;

    if (previous_pose_2d.x == UNITIALIZED) {
        previous_pose_2d.x = current_pose_2d.x;
        previous_pose_2d.y = current_pose_2d.y;
        previous_pose_2d.theta = current_pose_2d.theta;
    }

    delta_pose_2d.x = current_pose_2d.x - previous_pose_2d.x;
    delta_pose_2d.y = current_pose_2d.y - previous_pose_2d.y;
    delta_pose_2d.theta = current_pose_2d.theta - previous_pose_2d.theta;

    previous_pose_2d = current_pose_2d;
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

    particle_filter = new ParticleFilter(measurement_angles_degrees);
    previous_pose_2d.x = UNITIALIZED;
    previous_pose_2d.y = UNITIALIZED;
    previous_pose_2d.theta = UNITIALIZED;

    std::string laser_topic = "/ugv" + robot_suffix + "/scan";
    std::string odometry_topic = "/ugv" + robot_suffix + "/odom";

    laser_scan = node_handle.subscribe<sensor_msgs::LaserScan>(laser_topic, PUBSUB_QUEUE_SIZE, &Robot::laser_callback, this);
    odometry = node_handle.subscribe<nav_msgs::Odometry>(odometry_topic, PUBSUB_QUEUE_SIZE, &Robot::odometry_callback, this);
    broadcaster = node_handle.advertise<multi_robot_localization::particles>("plot_info_broadcast", PUBSUB_QUEUE_SIZE);

    all_robots_info = node_handle.subscribe<multi_robot_localization::clusters>("robot_inf_broadcast", PUBSUB_QUEUE_SIZE, &Robot::detector_callback, this);
    infos_to_detector = node_handle.advertise<multi_robot_localization::clusters>("robot_inf_broadcast", PUBSUB_QUEUE_SIZE);

    assert(measurement_angles_degrees.size() == robot_measurements.size());
}

void Robot::broadcast_particles() {
    static uint16_t downsample_idx = 0;
    if ((downsample_idx++ % 10) != 0) {
        return;
    }

    particle_filter->encoded_particles.robot_index = robot_index;
    multi_robot_localization::particle robot_particle = get_robot_particle_to_publish();
    particle_filter->encoded_particles.particles[ROBOT_PARTICLE_IDX] = robot_particle;

    printf("broadcasting %ld particles.\n", particle_filter->encoded_particles.particles.size());
    broadcaster.publish(particle_filter->encoded_particles);
}

multi_robot_localization::particle Robot::get_robot_particle_to_publish() {
    multi_robot_localization::particle robot_particle;
    robot_particle.x = previous_pose_2d.x;
    robot_particle.y = previous_pose_2d.y;
    robot_particle.angle = current_angle;
    robot_particle.type = ROBOT;
    //// Robot does not need to set weight for now
    // robot_particle.weight = p.weight;
    for (auto measurement : robot_measurements) {
        robot_particle.measurements.push_back(measurement);
    }
    return robot_particle;
}

Robot::~Robot() {
    delete particle_filter;
}
