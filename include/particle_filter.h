#pragma once

#include <stdint.h>

#include <random>
#include <vector>

#include "multi_robot_localization/particles.h"
#include "occupancy_grid.h"
#include "ros/ros.h"

#define PI 3.141592653589793238

#define N_CLUSTERS (4)
#define N_PARTICLES (10000)
#define N_ROAMING_PARTICLES (N_PARTICLES / 10)

#define N_PARTICLES_TO_PUBLISH ((N_PARTICLES / 50) + 1 + N_CLUSTERS)
#define ROBOT_PARTICLE_IDX ((N_PARTICLES / 50))
#define CLUSTER_PARTICLE_FIRST_IDX (ROBOT_PARTICLE_IDX + 1)

typedef struct _Particle {
    double x;
    double y;
    double angle;
    double weight;
    uint16_t id;
    std::vector<double> measurements;
} Particle;

Particle create_particle(double height, double width, uint16_t measurement_size);

class ParticleFilter {
    void move_particle(Particle& particle, double forward_movement, double delta_angle);

    std::vector<Particle> particles;
    std::vector<Particle> resampled_particles;

    OccupancyGrid* occupancy_grid;
    std::vector<uint16_t> measurement_angles_degrees;

   public:
    multi_robot_localization::particles encoded_particles;
    ParticleFilter(std::vector<uint16_t>& measurement_angles_degrees);
    bool is_path_free(double x_begin, double y_begin, double x_end, double y_end);
    Particle create_particle_in_free_cell(double height, double width, uint16_t measurement_size);
    void move_particles(double forward_movement, double delta_angle);
    void estimate_measurements();
    void update_weights_from_robot_measurements(const std::vector<double>& robot_measurements);
    void resample_particles();
    ~ParticleFilter() { delete occupancy_grid; };
};
