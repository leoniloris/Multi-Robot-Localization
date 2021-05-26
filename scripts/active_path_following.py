from multi_robot_localization.msg import clusters as ParticlesClusters
from dataclasses import dataclass, field
from geometry_msgs.msg import Twist

import path_following
import path_planner
import numpy as np
import rospy
import time
import sys
import re


def load_occupancy_grid():
    text = open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
    occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
    return np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")


OCCUPANCY_GRID = load_occupancy_grid()
N_MOST_CERTAIN_CLUSTERS = 3
N_ITERATIONS_WITH_CERTAIN_CLUSTERS = 10

robots_stuff = {}


@dataclass
class RobotStuff:
    actuator: rospy.Publisher
    path_follower: 'typing.Any' = field(default=None)
    clusters: 'typing.Any' = field(default={})
    previous_clusters_idxs: 'typing.Any' = field(default={})
    current_clusters_idxs: 'typing.Any' = field(default={})
    main_cluster_id: int = = field(default=0)



n_iterations_with_same_certain_clusters = 0
running_active_localization = False
odometry_last_run_at = time.time()
def clusters_recv_cb(msg):
    global robots_stuff, running_active_localization, n_iterations_with_same_certain_clusters
    if (time.time() - odometry_last_run_at) <= 0.3 and not running_active_localization:
        odometry_last_run_at = time.time()
        return

    robot_id = msg.robot_id
    robots_stuff[robot_id].current_clusters_idxs = set(np.argsort([
        c.weight for c in msg.clusters
    ])[:-N_MOST_CERTAIN_CLUSTERS:][::-1])

    if robots_stuff[robot_id].current_clusters_idxs == robots_stuff[robot_id].previous_clusters_idxs:
        n_iterations_with_same_certain_clusters += 1

    robots_stuff[robot_id].previous_clusters_idxs = robots_stuff[robot_id].current_clusters_idxs
    robots_stuff[robot_id].clusters = np.asarray(msg.clusters)[list(robots_stuff[robot_id].current_clusters_idxs)]

    running_active_localization = n_iterations_with_same_certain_clusters >= N_ITERATIONS_WITH_CERTAIN_CLUSTERS:
    if running_active_localization:
        n_iterations_with_same_certain_clusters = 0


paths_last_calculated_at = time.time()
def run_active_localization():
    # check if it's time to update  paths
    if (time.time() - paths_last_calculated_at) <= 5:
        return

    # for each robot select cluster with main_cluster_id'th largest weight
    robots_main_cluster = {
        robot_id: robot_stuff.clusters[robot_stuff.main_cluster_id]
        for robot_id, robot_stuff in robots_stuff.items()
    }
    # for each robot compute path to the largest-weighted cluster from someone else
    robots_begin_end = {}
    for robot_id, robot_main_clusters in robots_main_cluster.items():
        most_probable_cluster_other_robots = max(
            {r_id: r_main_cluster for (r_id, r_main_cluster) in robots_main_cluster.items() if r_id != robot_id},
            key=lambda c: c.weight)
        robots_begin_end[robot_id] = [
            (robot_main_clusters.x, robot_main_clusters.y),
            (most_probable_cluster_other_robots.x, most_probable_cluster_other_robots.y)
        ]

    # put each robot to follow that path
    robots_paths = {
        robot_id: path_planner.a_star(OCCUPANCY_GRID, robot_begin_end[0], robot_begin_end[1])
        for robot_id,robot_begin_end in robots_begin_end.items()
    }

    # for each robot: check if whether it's finished the path (stop experiment) or an obstacle was found (change main_cluster_id)
    #### when finished, we need to set running_active_localization = False


def main():
    global robots_stuff, running_active_localization
    robots_stuff = {
        int(robot_suffix): RobotStuff(rospy.Publisher(f'/ugv{str(robot_suffix)}/cmd_vel', Twist, queue_size=1))\
        for robot_suffix in sys.argv[1:]
    }
    rospy.init_node(f'active_path_following')
    sub = rospy.Subscriber('robot_inf_broadcast', ParticlesClusters, clusters_recv_cb)
    scheduler = rospy.Rate(20)

    while not rospy.is_shutdown():
        if running_active_localization:
            run_active_localization_state_machine()

        for robot_id, robot_stuff in robots_stuff.items():
            if robot_stuff.path_follower is not None:
                actuator.publish(robot_stuff.path_follower.control_msg)
        scheduler.sleep()



if __name__ == '__main__':
    main()
