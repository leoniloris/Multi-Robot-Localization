#include "minibatch_kmeans.h"

#include "math_utilities.h"
#include "multi_robot_localization/particles.h"
#include "particle_filter.h"

using namespace std;

// select the number of clusters
// randomly select 3 data points to be our cluster
// measure distance from each point to a given cluster, and assign a point to the nearest cluster
// compute the mean of each cluster based on the points it belongs to

static vector<Particle> cluster_particles(KMEANS_BATCH_SIZE);
static vector<Particle> clusters(N_CLUSTERS);

void kmeans_fill_batch_with(Particle p, uint16_t at) {
    cluster_particles[at] = p;
}

void kmeans_init_clusteers(double height, double width) {
    for (auto& c : clusters) {
        c = create_particle(height, width, 0);
    }
}

void kmeans_assign_nearest_cluster_to_particle(Particle& particle) {
    double distance_to_nearest_cluster = numeric_limits<double>::infinity();
    for (ClusterId cluster_id = 0; cluster_id < N_CLUSTERS; cluster_id++) {
        double distance_to_cluster = L2_DISTANCE((particle.x - clusters[cluster_id].x), (particle.y - clusters[cluster_id].y));
        if (distance_to_cluster < distance_to_nearest_cluster) {
            distance_to_nearest_cluster = distance_to_cluster;
            particle.cluster_id = cluster_id;
        }
    }
}

void kmeans_assign_nearest_cluster_to_particles() {
    for (auto& p : cluster_particles) {
        kmeans_assign_nearest_cluster_to_particle(p);
    }
}

void kmeans_update_cluster_center() {
    // Think about: the cluster here considers only the (x,y), but actually, the angle should be clustered as well
    vector<Particle> new_clusters(N_CLUSTERS);
    for (auto& cluster : new_clusters) {
        cluster.x = 0;
        cluster.y = 0;
        cluster.weight = 0;
    }

    // Compute cluster position by weighted-averaging the particle positions
    for (auto cluster_particle : cluster_particles) {
        const ClusterId cluster_id = cluster_particle.cluster_id;
        new_clusters[cluster_id].x += (cluster_particle.x * cluster_particle.weight);
        new_clusters[cluster_id].y += (cluster_particle.y * cluster_particle.weight);
        new_clusters[cluster_id].angle += (cluster_particle.angle * cluster_particle.weight);
        new_clusters[cluster_id].weight += (cluster_particle.weight);
    }
    double sum_of_cluster_weights = 0;
    for (auto& cluster : new_clusters) {
        sum_of_cluster_weights += cluster.weight;
        cluster.x /= (cluster.weight + EPS);
        cluster.y /= (cluster.weight + EPS);
        cluster.angle /= (cluster.weight + EPS);
    }

    // Normalize cluster weights
    for (auto& cluster : new_clusters) {
        cluster.weight /= (sum_of_cluster_weights + EPS);
    }

    clusters = new_clusters;
}

const vector<Particle>* kmeans_get_clusters() { return (const vector<Particle>*)&clusters; }
