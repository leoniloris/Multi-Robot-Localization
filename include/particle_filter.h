#pragma once

#include <stdint.h>

#include <random>
#include <vector>

#include "multi_robot_localization/particles.h"
#include "occupancy_grid.h"
#include "ros/ros.h"

#define PI 3.141592653589793238

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


   public:
    ParticleFilter(uint16_t number_of_particles, std::vector<uint16_t>& measurement_angles_degrees) ;
    void move_particles(double forward_movement, double delta_angle);
    void encode_particles_to_publish(multi_robot_localization::particles& particles);
    void estimate_measurements(double robot_sensor_offset);
    void update_weights_from_robot_measurements(const std::vector<double>& robot_measurements);
    void resample_particles();
    ~ParticleFilter() { delete occupancy_grid; };
};
