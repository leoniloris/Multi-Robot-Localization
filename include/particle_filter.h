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
    double measurement;
} Particle;

class ParticleFilter {
    void move_particle(Particle& particle, double delta_x, double delta_y, double delta_angle);

    std::mt19937 random_number_generator;
    uint16_t n_particles;
    OccupancyGrid* occupancy_grid;
    std::vector<Particle> particles;


   public:
    ParticleFilter(uint16_t number_of_particles);
    void move_particles(double delta_x, double delta_y, double delta_angle);
    void encode_particles_to_publish(multi_robot_localization::particles& particles);
    void estimate_measurements();
    ~ParticleFilter() { delete occupancy_grid; };
};
