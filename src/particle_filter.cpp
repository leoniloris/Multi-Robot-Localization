#include "particle_filter.h"

#include <cmath>
#include <random>

#include "ros/ros.h"

ParticleFilter::ParticleFilter(uint16_t n_particles, double max_x, double max_y, double max_angle) {
    this->n_particles = n_particles;
    particles.resize(n_particles);

    std::random_device rd;
    random_number_generator = std::mt19937(rd());

    std::uniform_real_distribution<double> distribution_x(0, max_x);
    std::uniform_real_distribution<double> distribution_y(0, max_y);
    std::uniform_real_distribution<double> distribution_angle(0, max_angle);

    for (uint16_t i = 0; i < n_particles; i++) {
        Particle p = Particle{};
        p.id = i;
        p.x = distribution_x(random_number_generator);
        p.y = distribution_y(random_number_generator);
        p.angle = distribution_angle(random_number_generator);
        p.weight = 1;
        particles[i] = p;
        ROS_INFO_STREAM("creating particle: x: " << p.x << " y: " << p.y << " angle: " << p.angle << " id: " << p.id);
    }
}

void ParticleFilter::move_particles(double std_x, double std_y, double std_angle, double delta_x, double delta_y, double delta_angle) {
    for (uint16_t i = 0; i < n_particles; i++) {
        Particle* p = &particles[i];
        move_particle(p, std_x, std_y, std_angle, delta_x, delta_y, delta_angle);
    }
}

void ParticleFilter::move_particle(Particle* particle,
                                   double std_x, double std_y, double std_angle,
                                   double delta_x, double delta_y, double delta_angle) {
    std::normal_distribution<double> distribution_x{particle->x + delta_x, std_x};
    std::normal_distribution<double> distribution_y{particle->y + delta_y, std_y};
    std::normal_distribution<double> distribution_angle{particle->angle + delta_angle, std_angle};

    particle->x = distribution_x(random_number_generator);
    particle->y = distribution_y(random_number_generator);
    particle->angle = distribution_angle(random_number_generator);
    particle->angle = std::fmod(particle->angle, 2 * PI);  // wrap 360 degrees
}
