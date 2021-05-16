#include "particle_filter.h"

#include <cassert>
#include <fstream>
#include <random>

#include "math_utilities.h"
#include "minibatch_kmeans.h"
#include "multi_robot_localization/particle.h"
#include "multi_robot_localization/particles.h"
#include "robot.h"
#include "ros/ros.h"

using namespace std;

static random_device rd;
static mt19937 random_number_generator;

bool ParticleFilter::is_path_free(double x_begin, double y_begin, double x_end, double y_end) {
    return occupancy_grid->is_path_free(x_begin, y_begin, x_end, y_end);
}

Particle create_particle(double height, double width, uint16_t measurement_size) {
    // static uniform_real_distribution<double> distribution_x(45, 45 + 0.1);
    // static uniform_real_distribution<double> distribution_y(80, 80 + 0.1);
    // static uniform_real_distribution<double> distribution_angle(0/2, 0/2 +0.0001);
    static uniform_real_distribution<double> distribution_x(0, height-1.0);
    static uniform_real_distribution<double> distribution_y(0, width-1.0);
    static uniform_real_distribution<double> distribution_angle(0, 2 * PI);
    Particle p = Particle{};
    p.x = distribution_x(random_number_generator);
    p.y = distribution_y(random_number_generator);
    p.angle = distribution_angle(random_number_generator);
    p.weight = 1;
    for (uint16_t _i = 0; _i < measurement_size; _i++) {
        p.measurements.push_back(numeric_limits<double>::infinity());
    }
    return p;
}

Particle ParticleFilter::create_particle_in_free_cell(double height, double width, uint16_t measurement_size) {
    Particle p = Particle{};

    bool is_cell_occupied = true;
    while (is_cell_occupied) {
        p = create_particle(height, width, measurement_size);
        is_cell_occupied = occupancy_grid->is_cell_occupied(p.x, p.y);
    }

    return p;
}

ParticleFilter::ParticleFilter(vector<uint16_t>& angles_degrees)
    : particles(N_PARTICLES), resampled_particles(N_PARTICLES) {
    occupancy_grid = new OccupancyGrid();

    for (auto angle_degree : angles_degrees) {
        measurement_angles_degrees.push_back(angle_degree);
    }

    random_number_generator = mt19937(rd());

    for (uint16_t particle_idx = 0; particle_idx < N_PARTICLES; particle_idx++) {
        Particle p = create_particle(occupancy_grid->height_cells(), occupancy_grid->width_cells(), measurement_angles_degrees.size());
        particles[particle_idx] = p;
        ROS_INFO_STREAM("creating particle: x: " << p.x << " y: " << p.y << " angle: " << p.angle);
    }

    encoded_particles.particles.clear();

    for (uint16_t idx = 0; idx < N_PARTICLES_TO_PUBLISH; idx++) {
        multi_robot_localization::particle encoded_particle;
        encoded_particles.particles.push_back(encoded_particle);
    }

    kmeans_init_clusteers(occupancy_grid->height_cells(), occupancy_grid->width_cells());
}

void ParticleFilter::move_particles(double forward_movement, double delta_angle) {
    for (auto& p : particles) {
        move_particle(p, forward_movement, delta_angle);
    }
}

void ParticleFilter::move_particle(Particle& particle, double forward_movement, double delta_angle) {
    normal_distribution<double> distribution_angle{0, ANGLE_STD_ODOMETRY};
    const double new_angle = particle.angle + delta_angle + fmod(distribution_angle(random_number_generator), 2 * PI);

    normal_distribution<double> distribution_position{0, POSITION_STD_ODOMETRY};
    const double noisy_forward_movement = forward_movement + distribution_position(random_number_generator);
    const double new_x = particle.x + cos(particle.angle) * noisy_forward_movement;
    const double new_y = particle.y + sin(particle.angle) * noisy_forward_movement;

    if (!occupancy_grid->is_path_free(particle.x, particle.y, new_x, new_y)) {
        //// TODO: We can try to just put the weights to 0 and update the particle
        //// But right here right now, we're just strongly decreasing the weight of
        //// the particle and not updating it because it might be the case that the
        //// particle just didn't sampled well.
        particle.weight *= 0.3;
        return;
    }

    particle.x = new_x;
    particle.y = new_y;
    particle.angle = new_angle;
}

void ParticleFilter::estimate_measurements() {
    static const double laser_max_range = meters_to_cells(LASER_MAX_RANGE_METERS);

    for (auto& p : particles) {
        for (uint16_t measurement_idx = 0; measurement_idx < measurement_angles_degrees.size(); measurement_idx++) {
            const double measurement_angle = p.angle + ((measurement_angles_degrees[measurement_idx] * PI) / 180.0);
            const double distance = occupancy_grid->distance_until_obstacle(p.x, p.y, measurement_angle);  // + robot_sensor_offset;

            p.measurements[measurement_idx] = distance < laser_max_range ? distance : laser_max_range;
        }
    }
}

void ParticleFilter::update_weights_from_robot_measurements(const vector<double>& robot_measurements) {
    if (robot_measurements.size() != particles[0].measurements.size()) {
        printf("particles should have the same measurement (size) as the robot. %ld != %ld\n",
               robot_measurements.size(), particles[0].measurements.size());
        return;
    }

    for (auto& p : particles) {
        p.weight = 0;
        for (uint16_t measurement_idx = 0; measurement_idx < p.measurements.size(); measurement_idx++) {
            const double robot_measurement = robot_measurements[measurement_idx];
            const double particle_measurement = p.measurements[measurement_idx];
            const double likelihood = GAUSSIAN_LIKELIHOOD(robot_measurement, LASER_SCAN_STD, particle_measurement);
            // Test different forms of aggregation. I'll use mean so the weight won't go to zero when 0 likelyhood is found
            p.weight += (likelihood / p.measurements.size());
        }
    }
}

void ParticleFilter::resample_particles() {
    static random_device rd;
    vector<double> weights;
    for (auto p : this->particles) {
        weights.push_back(p.weight);
    }
    discrete_distribution<int> distribution{weights.begin(), weights.end()};

    // Resample particles, keeping some roaming
    for (uint16_t particle_idx = 0; particle_idx < N_PARTICLES; particle_idx++) {
        Particle p = particles[distribution(rd)];

        if (particle_idx > N_ROAMING_PARTICLES) {
            resampled_particles[particle_idx] = p;
        } else {
            resampled_particles[particle_idx] = create_particle_in_free_cell(
                occupancy_grid->height_cells(),
                occupancy_grid->width_cells(),
                measurement_angles_degrees.size());
        }

        if (particle_idx < KMEANS_BATCH_SIZE) {
            kmeans_fill_batch_with(p, particle_idx);
        }
    }

    // Uniform sample particles to be encoded and plotted.
    static multi_robot_localization::particle encoded_particle;
    for (uint16_t particle_idx = 0; particle_idx < (ROBOT_PARTICLE_IDX); particle_idx++) {
        Particle p = resampled_particles[particle_idx * (N_PARTICLES / (ROBOT_PARTICLE_IDX))];
        encoded_particle.x = p.x;
        encoded_particle.y = p.y;
        encoded_particle.angle = p.angle;
        encoded_particle.weight = p.weight;
        encoded_particle.type = PARTICLE;
        encoded_particles.particles[particle_idx] = encoded_particle;
    }

    kmeans_assign_nearest_cluster_to_particles();
    kmeans_update_cluster_center();

    const vector<Particle>* clusters = kmeans_get_clusters();
    for (uint16_t cluster_id = 0; cluster_id < N_CLUSTERS; cluster_id++) {
        encoded_particle.x = (*clusters)[cluster_id].x;
        encoded_particle.y = (*clusters)[cluster_id].y;
        encoded_particle.weight = (*clusters)[cluster_id].weight;
        encoded_particle.type = CLUSTER;
        encoded_particles.particles[cluster_id + CLUSTER_PARTICLE_FIRST_IDX] = encoded_particle;
    }

    particles = resampled_particles;
}

void ParticleFilter::update_weights_based_on_detection(const vector<Particle> other_robot_clusters, const double measured_distance, const double measured_angle) {
    // Bigger standard deviation since a cluster has a huge diversity
    static const double detection_laser_std = LASER_SCAN_STD * 10.0;
    static const double detection_angle_std = ANGLE_STD_ODOMETRY * 10.0;

    // assign particles to nearest clusters
    for (auto& particle : particles) {
        kmeans_assign_nearest_cluster_to_particle(particle);
    }

    // update particles weights using likelihood of measurement
    const vector<Particle>* my_clusters = kmeans_get_clusters();
    vector<double> weight_gain_for_clusters(N_CLUSTERS, EPS);
    double sum_weight_gains = 0;
    for (uint16_t my_cluster_id = 0; my_cluster_id < N_CLUSTERS; my_cluster_id++) {
        auto my_cluster = (*my_clusters)[my_cluster_id];
        double my_cluster_weight_gain = 0;

        for (auto other_robot_cluster : other_robot_clusters) {
            // TODO: function for this.. it's just a pain to create functions on classes (on c++).
            const double dx = (my_cluster.x - other_robot_cluster.x);
            const double dy = (my_cluster.y - other_robot_cluster.y);
            const double cluster_distance = L2_DISTANCE(dx, dy);
            const double cluster_angle = atan(dy / (dx + EPS));

            const double likelihood_of_distance = GAUSSIAN_LIKELIHOOD(measured_distance, detection_laser_std, cluster_distance);
            const double likelihood_of_angle = PERIODIC_GAUSSIAN_LIKELIHOOD(measured_angle, detection_angle_std, cluster_angle, 2.0 * PI);

            my_cluster_weight_gain += my_cluster.weight * likelihood_of_distance * likelihood_of_angle;
        }

        weight_gain_for_clusters[my_cluster_id] = my_cluster_weight_gain;
        sum_weight_gains += my_cluster_weight_gain;
    }
    for (auto& w : weight_gain_for_clusters) {
        w /= sum_weight_gains;
    }
    for (auto& particle : particles) {
        particle.weight *= weight_gain_for_clusters[particle.cluster_id];
    }
}

//------------------------------------------------------------//
// For experiments
//------------------------------------------------------------//

static vector<vector<double>> saved_locations;

void ParticleFilter::save_state(uint16_t robot_id, double robot_x, double robot_y, double robot_angle) {
    static uint32_t new_rows_counter = 0;

    // append robot location and clusters locations to saved_locations
    vector<double> locations{robot_x, robot_y, robot_angle};
    const vector<Particle>* clusters = kmeans_get_clusters();
    for (auto cluster : (*clusters)) {
        locations.push_back(cluster.x);
        locations.push_back(cluster.y);
        locations.push_back(cluster.angle);
        locations.push_back(cluster.weight);
    }

    saved_locations.push_back(locations);

    // in a slower rate, writes to file and clear saved_locations
    const string path = string(getenv("HOME")) +
                        string("/catkin_ws/src/multi_robot_localization/") +
                        string("robot") +
                        to_string(robot_id) +
                        string("with_hard_detection_trial_") +
                        string(getenv("TRIAL")) +
                        string(".csv");
    if ((new_rows_counter++) % 10 == 0) {
        ofstream outfile;
        outfile.open(path, ios_base::app);

        for (auto saved_location : saved_locations) {
            ostringstream oss;
            copy(saved_location.begin(), saved_location.end() - 1, ostream_iterator<double>(oss, ","));
            oss << saved_location.back();

            outfile << oss.str() << endl;
        }

        saved_locations.clear();
    }
}