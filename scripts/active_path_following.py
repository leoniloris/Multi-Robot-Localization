from multi_robot_localization.msg import clusters as ParticlesClusters
from dataclasses import dataclass, field
from geometry_msgs.msg import Twist
from typing import Any

import path_following
import path_planner
import numpy as np
import rospy
import time
import sys
import re
import os

def load_occupancy_grid():
    text = open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
    occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
    return np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")


OCCUPANCY_GRID = load_occupancy_grid()
N_MOST_CERTAIN_CLUSTERS = 3
N_ITERATIONS_WITH_CERTAIN_CLUSTERS = 10


@dataclass
class RobotStuff:
    actuator: rospy.Publisher
    path_follower: 'Any' = field(default=None)
    clusters: 'Any' = field(default_factory=dict)
    previous_most_certain_clusters_idxs: 'Any' = field(default_factory=dict)
    current_most_certain_clusters_idxs: 'Any' = field(default_factory=dict)
    main_cluster_id: int = field(default=0)
    laser_sub : 'Any' = field(default=None)

@dataclass
class State:
    n_iterations_with_same_certain_clusters: 'Any' = field(default=0)
    running_active_localization: 'Any' = field(default=False)
    odometry_last_run_at: 'Any' = field(default=time.time())
    paths_last_calculated_at: 'Any' = field(default=time.time())
    robots_stuff: Any = field(default_factory=dict)


def get_indexes_for_decreasing_order_of_cluster_weight(clusters):
    return set(np.argsort([
        c.weight for c in clusters
    ])[:-N_MOST_CERTAIN_CLUSTERS:][::-1])


def clusters_recv_cb(msg):
    global state
    should_run_odometry_for_active_localization = state.running_active_localization and state.robots_stuff[robot_id].path_follower is not None
    if should_run_odometry_for_active_localization:
        print("should_run_odometry_for_active_localization")
        state.robots_stuff[robot_id].path_follower.control_pose_from_setpoint(cluster_x, cluster_y, cluster_angle)

    if (time.time() - state.odometry_last_run_at) <= 0.3 and not state.running_active_localization:
        state.odometry_last_run_at = time.time()
        return

    print("1 ==========")
    state.running_active_localization = process_odometry(msg)
    if state.running_active_localization:
        state.n_iterations_with_same_certain_clusters = 0
        prepare_for_active_localization()


def process_odometry(msg):
    global state
    robot_id = msg.robot_id
    state.robots_stuff[robot_id].current_most_certain_clusters_idxs = get_indexes_for_decreasing_order_of_cluster_weight(msg.clusters)

    if state.robots_stuff[robot_id].current_most_certain_clusters_idxs == state.robots_stuff[robot_id].previous_most_certain_clusters_idxs:
        state.n_iterations_with_same_certain_clusters += 1

    state.robots_stuff[robot_id].previous_most_certain_clusters_idxs = state.robots_stuff[robot_id].current_most_certain_clusters_idxs
    state.robots_stuff[robot_id].clusters = np.asarray(msg.clusters)

    did_the_best_clusters_remain_the_same = state.n_iterations_with_same_certain_clusters >= N_ITERATIONS_WITH_CERTAIN_CLUSTERS
    return did_the_best_clusters_remain_the_same


def prepare_for_active_localization():
    global state
    # at first, attempt with most probable cluster. (this part assumes all the robots have already sent their clusters)
    for robot_id, robot_stuff in state.robots_stuff.items():
        robot_stuff.main_cluster_id = get_indexes_for_decreasing_order_of_cluster_weight(robot_stuff.clusters)[0]


def run_active_localization():
    global state
    # check if it's time to update  paths
    if (time.time() - state.paths_last_calculated_at) <= 4:# seconds
        return
    state.paths_last_calculated_at = time.time()

    # for each robot select cluster with main_cluster_id'th largest weight
    robots_main_cluster = {
        robot_id: robot_stuff.clusters[robot_stuff.main_cluster_id]
        for robot_id, robot_stuff in state.robots_stuff.items()
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
        robot_id: [path_following.Landmark(*l) for l in path_planner.a_star(OCCUPANCY_GRID, robot_begin_end[0], robot_begin_end[1])]
        for robot_id,robot_begin_end in robots_begin_end.items()
    }

    for robot_id, robot_stuff in state.robots_stuff.items():
        robot_stuff.path_follower = path_following.PathFollower(robots_paths[robot_id])
        robot_stuff.laser_sub = rospy.Subscriber(f'/ugv{str(robot_id)}/scan', LaserScan, robot_stuff.path_follower.clbk_laser)

    # for each robot: check whether it's finished the path (stop experiment) or an obstacle was found (change main_cluster_id for a given robot)
    #### when finished, we need to set running_active_localization = False and unregister callbacks, set None stuff to None


def main():
    global state
    state = State()
    rospy.init_node('active_path_following')
    state.robots_stuff = {
        int(robot_id): RobotStuff(rospy.Publisher(f'/ugv{str(robot_id)}/cmd_vel', Twist, queue_size=1))\
        for robot_id in sys.argv[1:]
    }

    _clusters_subscriber = rospy.Subscriber('robot_inf_broadcast', ParticlesClusters, clusters_recv_cb)
    print(">>>>================ ", _clusters_subscriber)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_control_messages()
        rate.sleep()


def publish_control_messages():
    global state
    for robot_id, robot_stuff in state.robots_stuff.items():
        if robot_stuff.path_follower is not None:
            actuator.publish(robot_stuff.path_follower.control_msg)


if __name__ == '__main__':
    main()
