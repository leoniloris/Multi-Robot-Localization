#include "particle_filter.h"

#include <cassert>
#include <random>

#include "math_utilities.h"
#include "multi_robot_localization/particle.h"
#include "multi_robot_localization/particles.h"
#include "robot.h"
#include "ros/ros.h"

using namespace std;

#define EPS 0.00000001

ParticleFilter::ParticleFilter(uint16_t number_of_particles, std::vector<uint16_t>& angles_degrees) {
    const string home_folder = string(getenv("HOME"));
    const string grid_path = string("/catkin_ws/src/multi_robot_localization/occupancy_grid/base_occupancy_grid.csv");

    occupancy_grid = new OccupancyGrid(home_folder + grid_path);
    n_particles = number_of_particles;

    for (auto angle_degree : angles_degrees) {
        measurement_angles_degrees.push_back(angle_degree);
    }

    random_device rd;
    random_number_generator = mt19937(rd());

    // uniform_real_distribution<double> distribution_x(490, 490 + 1);
    // uniform_real_distribution<double> distribution_y(1366 + 29.999, 1366 + 30);
    // uniform_real_distribution<double> distribution_angle(180 * PI / 180, 181 * PI / 180);
    uniform_real_distribution<double> distribution_x(0, (double)occupancy_grid->height_cells());
    uniform_real_distribution<double> distribution_y(0, (double)occupancy_grid->width_cells());
    uniform_real_distribution<double> distribution_angle(0, 2 * PI);

    for (uint16_t particle_idx = 0; particle_idx < n_particles; particle_idx++) {
        Particle p = Particle{};
        p.id = particle_idx;
        p.x = distribution_x(random_number_generator);
        p.y = distribution_y(random_number_generator);
        p.angle = distribution_angle(random_number_generator);
        p.weight = 1;
        for (auto _ : measurement_angles_degrees) {
            p.measurements.push_back(numeric_limits<double>::infinity());
        }
        particles.push_back(p);
        ROS_INFO_STREAM("creating particle: x: " << p.x << " y: " << p.y << " angle: " << p.angle << " id: " << p.id);
    }
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
    const double new_x = particle.x + (-1 * sin(particle.angle)) * noisy_forward_movement;
    const double new_y = particle.y + cos(particle.angle) * noisy_forward_movement;

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

void ParticleFilter::encode_particles_to_publish(multi_robot_localization::particles& encoded_particles) {
    encoded_particles.particles.clear();
    for (auto& p : particles) {
        multi_robot_localization::particle encoded_particle;
        encoded_particle.x = p.x;
        encoded_particle.y = p.y;
        encoded_particle.angle = p.angle;
        encoded_particle.weight = p.weight;
        encoded_particle.id = p.id;
        encoded_particle.type = PARTICLE;
        for (auto measurement : p.measurements) {
            encoded_particle.measurements.push_back(measurement);
        }
        encoded_particles.particles.push_back(encoded_particle);
    }
}

void ParticleFilter::estimate_measurements(double robot_sensor_offset) {
    for (auto& p : particles) {
        for (uint16_t measurement_idx = 0; measurement_idx < measurement_angles_degrees.size(); measurement_idx++) {
            const double measurement_angle = p.angle + ((measurement_angles_degrees[measurement_idx] * PI) / 180.0);
            p.measurements[measurement_idx] = occupancy_grid->distance_until_obstacle(p.x, p.y, measurement_angle);  // + robot_sensor_offset;
        }
    }
}

void ParticleFilter::update_weights_from_robot_measurements(const std::vector<double>& robot_measurements) {
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
    static uint16_t aa = 0;
    printf("resampling\n");

    vector<Particle> new_particles;
    vector<double> weights;

    for (auto p : this->particles) {
        weights.push_back(p.weight);
    }

    discrete_distribution<int> distribution{weights.begin(), weights.end()};
    static random_device rd;
    for (uint16_t particle_idx = 0; particle_idx < particles.size(); particle_idx++) {
        Particle p = particles[distribution(rd)];
        p.id = particle_idx;
        p.id = ;
        new_particles.push_back(p);
    }

    particles = new_particles;
}