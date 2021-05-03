#pragma once

#include <stdint.h>

#include <random>
#include <vector>

#include "multi_robot_localization/particles.h"
#include "occupancy_grid.h"
#include "ros/ros.h"

#define PI 3.141592653589793238

#define N_PARTICLES 10000
#define N_PARTICLES_TO_PUBLISH (N_PARTICLES/100)

typedef struct _Particle {
    double x;
    double y;
    double angle;
    double weight;
    uint16_t id;
    std::vector<double> measurements;
} Particle;

class ParticleFilter {
    void move_particle(Particle& particle, double forward_movement, double delta_angle);

    std::mt19937 random_number_generator;
    uint16_t n_particles;
    OccupancyGrid* occupancy_grid;
    std::vector<Particle> particles;
    std::vector<uint16_t> measurement_angles_degrees;

    void shuffle_random_indexes();
    std::vector<uint16_t> random_indexes; // speeding up random sampling

   public:
    ParticleFilter(uint16_t number_of_particles, std::vector<uint16_t>& measurement_angles_degrees);
    bool is_path_free(double x_begin, double y_begin, double x_end, double y_end);
    void move_particles(double forward_movement, double delta_angle);
    void encode_particles_to_publish(multi_robot_localization::particles& particles);
    void estimate_measurements();
    void update_weights_from_robot_measurements(const std::vector<double>& robot_measurements);
    void resample_particles();
    ~ParticleFilter() { delete occupancy_grid; };
};
