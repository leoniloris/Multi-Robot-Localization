#pragma once

#include <stdint.h>

#include <random>
#include <vector>

#include "occupancy_grid.h"
#include "ros/ros.h"

#define PI 3.141592653589793238

typedef struct _Particle {
    double x;
    double y;
    double angle;
    double weight;
    uint16_t id;
} Particle;

class ParticleFilter {
    std::vector<Particle> particles;
    std::mt19937 random_number_generator;
    uint16_t n_particles;
    OccupancyGrid* occupancy_grid;

    void move_particle(Particle* particle, double std_x, double std_y, double std_angle, double delta_x, double delta_y, double delta_angle);

   public:
    ParticleFilter(uint16_t number_of_particles, double max_x, double max_y, double max_angle);
    void move_particles(double std_x, double std_y, double std_angle, double delta_x, double delta_y, double delta_angle);
    ~ParticleFilter(){ delete occupancy_grid; };
};
