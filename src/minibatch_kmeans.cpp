#include "minibatch_kmeans.h"

#include "particle_filter.h"

using namespace std;

// select the number of clusters
// randomly select 3 data points to be our cluster
// measure distance from each point to a given cluster, and assign a point to the nearest cluster
// compute the mean of each cluster based on the points it belongs to
// check if the sum of the variance of all clusters have

typedef uint16_t ClusterIdx;

static vector<tuple<ClusterIdx, Particle>> particles_batch_to_cluster(BATCH_SIZE);
static vector<Particle> clusters(N_CLUSTERS);

void kmeans_fill_batch_with(Particle p, uint16_t at) {
    tuple<ClusterIdx, Particle> particle_to_cluster(0xffff, p);
    particles_batch_to_cluster[at] = particle_to_cluster;
}

void kmeans_init_clusteers(double height, double width) {
    for (auto& c : clusters) {
        c = create_particle(height, width, 0);
    }
}

void kmeans_assign_nearest_cluster_to_particles(double height, double width) {
    for (auto& p : particles_batch_to_cluster) {
        // c = create_particle(height, width, 0);
    }
}