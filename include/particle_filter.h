#pragma once

#include <vector>

typedef struct _Particle {
    double x;
    double y;
    double angle;
    double weight;
    int id;
} Particle;

class ParticleFilter {
    int n_particles;
    std::vector<Particle> particles;

   public:
    ParticleFilter(int n_particles, double max_x, double max_y, double max_angle);
    void move_particles(double std_x, double std_y, double std_angle, double delta_x, double delta_y, double delta_angle);
    ~ParticleFilter(){};
};
