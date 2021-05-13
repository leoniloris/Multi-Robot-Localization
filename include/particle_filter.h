#pragma once

#include <stdint.h>

#include <random>
#include <vector>

#include "multi_robot_localization/particles.h"
#include "occupancy_grid.h"
#include "ros/ros.h"

using namespace std;

#define PI 3.141592653589793238

#define N_CLUSTERS (8)
#define N_PARTICLES (10000)
#define N_ROAMING_PARTICLES (N_PARTICLES / 10)

#define N_PARTICLES_TO_PUBLISH ((N_PARTICLES / 50) + 1 + N_CLUSTERS)
#define ROBOT_PARTICLE_IDX ((N_PARTICLES / 50))
#define CLUSTER_PARTICLE_FIRST_IDX (ROBOT_PARTICLE_IDX + 1)

typedef uint16_t ClusterId;

typedef struct _Particle {
    double x;
    double y;
    double angle;
    double weight;
    ClusterId cluster_id;
    vector<double> measurements;
} Particle;

Particle create_particle(double height, double width, uint16_t measurement_size);

class ParticleFilter {
    void move_particle(Particle& particle, double forward_movement, double delta_angle);

    vector<Particle> particles;
    vector<Particle> resampled_particles;

    OccupancyGrid* occupancy_grid;
    vector<uint16_t> measurement_angles_degrees;

   public:
    ParticleFilter(vector<uint16_t>& measurement_angles_degrees);
    bool is_path_free(double x_begin, double y_begin, double x_end, double y_end);
    Particle create_particle_in_free_cell(double height, double width, uint16_t measurement_size);
    void move_particles(double forward_movement, double delta_angle);
    void estimate_measurements();
    void update_weights_from_robot_measurements(const vector<double>& robot_measurements);
    void resample_particles();
    void update_weights_based_on_detection(const vector<Particle> other_robot_clusters, const double measured_distance, const double measured_angle);
    ~ParticleFilter() { delete occupancy_grid; };

    multi_robot_localization::particles encoded_particles;
};
