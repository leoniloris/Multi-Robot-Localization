#include "particle_filter.h"

#include <random>

#include "math_utilities.h"
#include "multi_robot/particle.h"
#include "multi_robot/particles.h"
#include "robot.h"
#include "ros/ros.h"

using namespace std;

#define MEASUREMENT_STD 0.1
#define EPS 0.00000001

ParticleFilter::ParticleFilter(uint16_t number_of_particles) {
    const string home_folder = string(getenv("HOME"));
    const string grid_path = string("/catkin_ws/src/multi_robot/occupancy_grid/base_occupancy_grid.csv");

    occupancy_grid = new OccupancyGrid(home_folder + grid_path);
    n_particles = number_of_particles;

    random_device rd;
    random_number_generator = mt19937(rd());

    uniform_real_distribution<double> distribution_x(0, (double)occupancy_grid->height_cells());
    uniform_real_distribution<double> distribution_y(0, (double)occupancy_grid->width_cells());
    uniform_real_distribution<double> distribution_angle(0, 2 * PI);
    for (uint16_t i = 0; i < n_particles; i++) {
        Particle p = Particle{};
        p.id = i;
        p.x = distribution_x(random_number_generator);
        p.y = distribution_y(random_number_generator);
        p.angle = distribution_angle(random_number_generator);
        p.weight = 1;
        p.measurement = numeric_limits<double>::infinity();
        particles.push_back(p);
        ROS_INFO_STREAM("creating particle: x: " << p.x << " y: " << p.y << " angle: " << p.angle << " id: " << p.id);
    }
}

void ParticleFilter::move_particles(double delta_x, double delta_y, double delta_angle) {
    for (auto& p : particles) {
        move_particle(p, delta_x, delta_y, delta_angle);
    }
}

void ParticleFilter::move_particle(Particle& particle, double delta_x, double delta_y, double delta_angle) {
    const double displacement = L2_DISTANCE(delta_x, delta_y);

    const double delta_x_with_angular_movement = (displacement / (delta_angle+EPS)) * (sin(particle.angle + delta_angle) - sin(particle.angle));
    const double delta_y_with_angular_movement = (displacement / (delta_angle+EPS)) * (cos(particle.angle) - cos(particle.angle + delta_angle));

    normal_distribution<double> distribution_x{particle.x + delta_x_with_angular_movement, X_STD_ODOMETRY};
    normal_distribution<double> distribution_y{particle.y + delta_y_with_angular_movement, Y_STD_ODOMETRY};
    normal_distribution<double> distribution_angle{particle.angle + delta_angle, ANGLE_STD_ODOMETRY};

    const double new_x = distribution_x(random_number_generator);
    const double new_y = distribution_y(random_number_generator);
    const double new_angle = fmod(distribution_angle(random_number_generator), 2 * PI);  // wrap 360 degrees

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

void ParticleFilter::encode_particles_to_publish(multi_robot::particles& encoded_particles) {
    encoded_particles.particles.clear();
    for (auto& p : particles) {
        multi_robot::particle encoded_particle;
        encoded_particle.x = p.x;
        encoded_particle.y = p.y;
        encoded_particle.angle = p.angle;
        encoded_particle.weight = p.weight;
        encoded_particle.id = p.id;
        encoded_particle.type = PARTICLE;
        encoded_particle.measurement = p.measurement;
        encoded_particles.particles.push_back(encoded_particle);
    }
}

void ParticleFilter::estimate_measurements() {
    // Laser-based measurement.
    normal_distribution<double> distribution{0, MEASUREMENT_STD};

    for (auto& p : particles) {
        const double measurement_noise = distribution(random_number_generator);
        p.measurement = occupancy_grid->distance_until_obstacle(p.x, p.y, p.angle);// + measurement_noise;
        printf("p.measurement %f\n", p.measurement);
    }
}
