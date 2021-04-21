#include "particle_filter.h"

#include <random>

#include "ros/ros.h"

ParticleFilter::ParticleFilter(int n_particles, double std_x, double std_y, double std_theta, double max_x, double max_y, double max_theta) {
    particles.resize(n_particles);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution_x(0, max_x);
    std::uniform_real_distribution<double> distribution_y(0, max_y);
    std::uniform_real_distribution<double> distribution_theta(0, max_theta);

    for (int i = 0; i < n_particles; ++i) {
        Particle p = Particle{};
        p.id = i;
        p.x = distribution_x(generator);
        p.y = distribution_y(generator);
        p.theta = distribution_theta(generator);
        p.weight = 1;
        particles[i] = p;
    }
}
