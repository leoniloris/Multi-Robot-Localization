#include "particle_filter.h"

#include <cmath>
#include <random>

#include "ros/ros.h"

#define PI 3.141592653589793238

static void move_particle(Particle* particle,
                          double std_x, double std_y, double std_angle,
                          double delta_x, double delta_y, double delta_angle,
                          std::default_random_engine* gen) {
    std::normal_distribution<double> distribution_x(particle->x + delta_x, std_x);
    std::normal_distribution<double> distribution_y(particle->y + delta_y, std_y);
    std::normal_distribution<double> distribution_angle(particle->angle + delta_angle, std_angle);

    particle->x = distribution_x(*gen);
    particle->y = distribution_y(*gen);
    particle->angle = distribution_angle(*gen);
    particle->angle = std::fmod(particle->angle, 2 * PI);  // wrap 360 degrees
}

ParticleFilter::ParticleFilter(int n_particles, double max_x, double max_y, double max_angle) {
    particles.resize(n_particles);

    std::default_random_engine gen;
    std::uniform_real_distribution<double> distribution_x(0, max_x);
    std::uniform_real_distribution<double> distribution_y(0, max_y);
    std::uniform_real_distribution<double> distribution_angle(0, max_angle);

    for (int i = 0; i < n_particles; ++i) {
        Particle p = Particle{};
        p.id = i;
        p.x = distribution_x(gen);
        p.y = distribution_y(gen);
        p.angle = distribution_angle(gen);
        p.weight = 1;
        particles[i] = p;
    }
}

void ParticleFilter::move_particles(double std_x, double std_y, double std_angle, double delta_x, double delta_y, double delta_angle) {
    std::default_random_engine gen;

    for (int i = 0; i < n_particles; ++i) {
        Particle* p = &particles[i];
        move_particle(p, std_x, std_y, std_angle, delta_x, delta_y, delta_angle, &gen);
    }
}
