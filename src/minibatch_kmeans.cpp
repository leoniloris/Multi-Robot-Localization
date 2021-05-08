#include "minibatch_kmeans.h"

#include "math_utilities.h"
#include "multi_robot_localization/particles.h"
#include "particle_filter.h"

using namespace std;

// select the number of clusters
// randomly select 3 data points to be our cluster
// measure distance from each point to a given cluster, and assign a point to the nearest cluster
// compute the mean of each cluster based on the points it belongs to
// check if the sum of the variance of all clusters have

#define GET_CLUSTER_ID(cluster_particle) get<0>(cluster_particle)
#define GET_PARTICLE(cluster_particle) get<1>(cluster_particle)

typedef uint16_t ClusterIdx;
typedef tuple<ClusterIdx, Particle> ClusterParticle;

static vector<ClusterParticle> cluster_particles(KMEANS_BATCH_SIZE);
static vector<Particle> clusters(N_CLUSTERS);

void kmeans_fill_batch_with(Particle p, uint16_t at) {
    tuple<ClusterIdx, Particle> particle_to_cluster(0xffff, p);
    cluster_particles[at] = particle_to_cluster;
}

void kmeans_init_clusteers(double height, double width) {
    for (auto& c : clusters) {
        c = create_particle(height, width, 0);
    }
}

static inline void assign_nearest_cluster_to_cluster_particle(ClusterParticle& cluster_particle) {
    double distance_to_nearest_cluster = numeric_limits<double>::infinity();
    for (ClusterIdx cluster_idx = 0; cluster_idx < N_CLUSTERS; cluster_idx++) {
        const Particle particle = GET_PARTICLE(cluster_particle);
        double distance_to_cluster = L2_DISTANCE((particle.x - clusters[cluster_idx].x), (particle.y - clusters[cluster_idx].y));
        if (distance_to_cluster < distance_to_nearest_cluster) {
            distance_to_nearest_cluster = distance_to_cluster;
            GET_CLUSTER_ID(cluster_particle) = cluster_idx;
        }
    }
}

void kmeans_assign_nearest_cluster_to_particles() {
    for (auto& p : cluster_particles) {
        assign_nearest_cluster_to_cluster_particle(p);
    }
}

void kmeans_update_cluster_center() {
    // Think about: the cluster here considers only the (x,y), byut actually, the angle should be clustered as well
    vector<Particle> new_clusters(N_CLUSTERS);
    for (auto& cluster : new_clusters) {
        cluster.x = 0;
        cluster.y = 0;
        cluster.weight = 0;
    }

    for (auto cluster_particle : cluster_particles) {
        const ClusterIdx cluster_idx = GET_CLUSTER_ID(cluster_particle);
        const Particle particle = GET_PARTICLE(cluster_particle);
        new_clusters[cluster_idx].x += (particle.x * particle.weight);
        new_clusters[cluster_idx].y += (particle.y * particle.weight);
        new_clusters[cluster_idx].weight += particle.weight;
    }

    for (auto& cluster : new_clusters) {
        cluster.x /= cluster.weight;
        cluster.y /= cluster.weight;
    }
    clusters = new_clusters;
}

const vector<Particle> kmeans_get_clusters() { return clusters; }
