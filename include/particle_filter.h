#pragma once

#include <vector>

typedef struct _Particle {
    double x;
    double y;
    double theta;
    double weight;
    int id;
} Particle;

class ParticleFilter {
    int n_particles;
    std::vector<Particle> particles;

   public:
    ParticleFilter(int n_particles, double std_x, double std_y, double std_theta, double max_x, double max_y, double max_theta);
    ~ParticleFilter(){};
};
