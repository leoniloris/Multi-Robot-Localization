#pragma once

#include "particle_filter.h"

#define BATCH_SIZE (N_PARTICLES / 20)
#define N_CLUSTERS (4)

void kmeans_fill_batch_with(Particle p, uint16_t at);
void kmeans_init_clusteers(double height, double width);
