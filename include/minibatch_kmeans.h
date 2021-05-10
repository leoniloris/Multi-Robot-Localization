#pragma once

#include "multi_robot_localization/particles.h"
#include "particle_filter.h"

using namespace std;

#define KMEANS_BATCH_SIZE (N_PARTICLES / 10)

void kmeans_fill_batch_with(Particle p, uint16_t at);
void kmeans_assign_nearest_cluster_to_particle(Particle& particle);
void kmeans_init_clusteers(double height, double width);
void kmeans_assign_nearest_cluster_to_particles();
void kmeans_update_cluster_center();
const vector<Particle> kmeans_get_clusters();
