from multi_robot_localization.msg import clusters as ParticlesClusters
from dataclasses import dataclass, field
from sensor_msgs.msg import LaserScan
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
N_ITERATIONS_WITH_CERTAIN_CLUSTERS = 30
RECOMPUTE_PATHS_PERIOD_S = 10


@dataclass
class RobotStuff:
    actuator: rospy.Publisher
    path_follower: 'Any' = field(default=None)
    clusters: 'Any' = field(default_factory=dict)
    previous_most_certain_clusters_ids: 'Any' = field(default_factory=list)
    current_most_certain_clusters_ids: 'Any' = field(default_factory=list)
    main_cluster_idx: int = field(default=0)
    laser_sub : 'Any' = field(default=None)

@dataclass
class State:
    n_iterations_with_same_certain_clusters: 'Any' = field(default=0)
    running_active_localization: 'Any' = field(default=False)
    odometry_last_run_at: 'Any' = field(default_factory=lambda :0)
    paths_last_calculated_at: 'Any' = field(default_factory=lambda :0)
    robots_stuff: Any = field(default_factory=dict)


def get_indexes_for_decreasing_order_of_cluster_weight(clusters):
    return list(np.argsort([
        c.weight for c in clusters
    ])[::-1][:N_MOST_CERTAIN_CLUSTERS])


def clusters_recv_cb(msg):
    global state
    robot_id = msg.origin_robot_index
    should_run_odometry_for_active_localization = state.running_active_localization and state.robots_stuff[robot_id].path_follower is not None
    if should_run_odometry_for_active_localization:

        cluster_id = state.robots_stuff[robot_id].current_most_certain_clusters_ids[state.robots_stuff[robot_id].main_cluster_idx]

        cluster = state.robots_stuff[robot_id].clusters[cluster_id]
        cluster_x, cluster_y, cluster_angle = cluster.x, cluster.y, cluster.angle
        state.robots_stuff[robot_id].path_follower.control_pose_from_setpoint(cluster_y, cluster_x, cluster_angle)

    ## do this debounce, but for each separate robot
    # if (time.time() - state.odometry_last_run_at) <= 0.4 or state.running_active_localization:
    #     return
    # state.odometry_last_run_at = time.time()

    state.running_active_localization = process_clusters_odometry(msg)
    if state.running_active_localization:
        state.n_iterations_with_same_certain_clusters = 0
        prepare_for_active_localization()


def process_clusters_odometry(msg):
    global state
    robot_id = msg.origin_robot_index
    state.robots_stuff[robot_id].current_most_certain_clusters_ids = get_indexes_for_decreasing_order_of_cluster_weight(msg.clusters)

    if state.robots_stuff[robot_id].current_most_certain_clusters_ids == state.robots_stuff[robot_id].previous_most_certain_clusters_ids:
        state.n_iterations_with_same_certain_clusters += 1
    else:
        state.n_iterations_with_same_certain_clusters = 0
    state.robots_stuff[robot_id].previous_most_certain_clusters_ids = state.robots_stuff[robot_id].current_most_certain_clusters_ids
    state.robots_stuff[robot_id].clusters = msg.clusters

    did_the_best_clusters_remain_the_same = state.n_iterations_with_same_certain_clusters >= N_ITERATIONS_WITH_CERTAIN_CLUSTERS
    return did_the_best_clusters_remain_the_same


def prepare_for_active_localization():
    global state
    # at first, attempt with most probable cluster. (this part assumes all the robots have already sent their clusters)
    for robot_id, robot_stuff in state.robots_stuff.items():
        if len(robot_stuff.clusters) == 0:
            print("trying to start active localization without some active robots!")
            state.running_active_localization = False
            return
        robot_stuff.main_cluster_idx = 0


def run_active_localization():
    global state

    # check if it's time to update  paths
    if (time.time() - state.paths_last_calculated_at) <= RECOMPUTE_PATHS_PERIOD_S:# seconds
        return
    state.paths_last_calculated_at = time.time()

    # change main_cluster_idx if required by wall colision
    for robot_id, robot_stuff in state.robots_stuff.items():
        if robot_stuff.path_follower is not None and robot_stuff.path_follower.path_is_probably_obstructed:
            robot_stuff.main_cluster_idx += 1
            if robot_stuff.laser_sub is not None:
                robot_stuff.laser_sub.unregister()
                del robot_stuff.path_follower
                del robot_stuff.laser_sub

    # for each robot select cluster with main_cluster_idx'th largest weight
    robots_main_cluster = {
        robot_id: robot_stuff.clusters[robot_stuff.current_most_certain_clusters_ids[robot_stuff.main_cluster_idx]]
        for robot_id, robot_stuff in state.robots_stuff.items()
    }

    # for each robot compute path to the largest-weighted cluster from someone else
    robots_begin_end = {}
    for robot_id, robot_main_clusters in robots_main_cluster.items():
        most_probable_cluster_other_robots = max(
            [r_main_cluster for (r_id, r_main_cluster) in robots_main_cluster.items() if r_id != robot_id],
            key=lambda c: c.weight)
        robots_begin_end[robot_id] = [
            [robot_main_clusters.x, robot_main_clusters.y],
            [most_probable_cluster_other_robots.x, most_probable_cluster_other_robots.y]
        ]

    # put each robot to follow that path
    robots_paths = {}
    print(robots_begin_end)
    for robot_id,robot_begin_end in robots_begin_end.items():
        print('robot_begin_end', robot_id, tuple(np.int_(robot_begin_end[0])), tuple(np.int_(robot_begin_end[1])))
        robots_paths[robot_id] = [
            path_following.Landmark(*l) for l in
            path_planner.a_star(OCCUPANCY_GRID,
                                tuple(np.int_(robot_begin_end[0])),
                                tuple(np.int_(robot_begin_end[1])))
            ]

    print(f'robots_paths {robots_paths}')

    for robot_id, robot_stuff in state.robots_stuff.items():
        robot_stuff.path_follower = path_following.PathFollower(robots_paths[robot_id])
        robot_stuff.laser_sub = rospy.Subscriber(f'/ugv{str(robot_id)}/scan', LaserScan, robot_stuff.path_follower.clbk_laser)

    print(robot_stuff.path_follower, robot_stuff.laser_sub)
    #### when finished, we need to set running_active_localization = False and unregister callbacks, set None stuff to None


def main():
    global state
    state = State()
    rospy.init_node('active_path_following_')
    state.robots_stuff = {
        int(robot_id): RobotStuff(rospy.Publisher(f'/ugv{str(robot_id)}/cmd_vel', Twist, queue_size=10))\
        for robot_id in sys.argv[1:]
    }

    _clusters_subscriber = rospy.Subscriber('robot_inf_broadcast', ParticlesClusters, clusters_recv_cb)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if state.running_active_localization:
            run_active_localization()
        publish_control_messages()
        rate.sleep()


def publish_control_messages():
    global state
    for robot_id, robot_stuff in state.robots_stuff.items():
        if robot_stuff.path_follower is not None:
            robot_stuff.actuator.publish(robot_stuff.path_follower.control_msg)


if __name__ == '__main__':
    main()
