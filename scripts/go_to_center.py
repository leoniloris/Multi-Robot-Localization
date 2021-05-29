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
N_MOST_CERTAIN_CLUSTERS = 2
N_ITERATIONS_WITH_CERTAIN_CLUSTERS = 30
RECOMPUTE_PATHS_PERIOD_S = 100000


@dataclass
class RobotStuff:
    actuator: rospy.Publisher
    x_y_angle_pra_roubar: 'Any'  = field(default=None)######## AAAAAAAAAA
    path_follower: 'Any' = field(default=None)
    clusters: 'Any' = field(default_factory=dict)
    previous_most_certain_clusters_ids: 'Any' = field(default_factory=list)
    current_most_certain_clusters_ids: 'Any' = field(default_factory=list)
    main_cluster_idx: int = field(default=0)
    laser_sub : 'Any' = field(default=None)
    best_cluster_weight : 'Any' = field(default=0)


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
    state.robots_stuff[robot_id].clusters = msg.clusters

    if state.running_active_localization:
        cluster_id = state.robots_stuff[robot_id].current_most_certain_clusters_ids[state.robots_stuff[robot_id].main_cluster_idx]
        cluster = max(state.robots_stuff[robot_id].clusters, key=lambda c:c.weight)
        cluster_x, cluster_y, cluster_angle = cluster.x, cluster.y, cluster.angle


        if state.robots_stuff[robot_id].path_follower is not None:
            angolinho = state.robots_stuff[robot_id].x_y_angle_pra_roubar[2]
            angolinho = np.fmod(angolinho+4*np.pi, 2*np.pi)
            # state.robots_stuff[robot_id].path_follower.control_pose_from_setpoint(cluster_x, cluster_y, angolinho)
            state.robots_stuff[robot_id].path_follower.control_pose_from_setpoint(cluster_x, cluster_y, cluster_angle)
        run_active_localization()
        # if int(robot_id) == 2:
            # print(f'cluster_x, cluster_y {cluster_x, cluster_y}')

    else:
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

    did_the_best_clusters_remain_the_same = state.n_iterations_with_same_certain_clusters >= N_ITERATIONS_WITH_CERTAIN_CLUSTERS

    robot_stuff = state.robots_stuff[robot_id]
    robot_stuff.best_cluster_weight = 0.9*robot_stuff.best_cluster_weight + 0.1*robot_stuff.clusters[robot_stuff.current_most_certain_clusters_ids[0]].weight
    has_some_certanty = all([robot_stuff.best_cluster_weight > 0.5 for robot_stuff in state.robots_stuff.values()])
    # print(state.robots_stuff[1].best_cluster_weight, state.robots_stuff[2].best_cluster_weight)

    ##### ROUBANDO
    a = True
    for robot_stuff in state.robots_stuff.values():
        if len(robot_stuff.previous_most_certain_clusters_ids) >1 and len(robot_stuff.clusters) > 1 and robot_stuff.x_y_angle_pra_roubar is not None:
            cluster_idx = robot_stuff.previous_most_certain_clusters_ids[robot_stuff.main_cluster_idx]
            cluster_xy = np.asarray([robot_stuff.clusters[cluster_idx].x,robot_stuff.clusters[cluster_idx].y])
            real_xy = np.asarray(robot_stuff.x_y_angle_pra_roubar[:2])
            d = np.sqrt(((cluster_xy - real_xy)**2).sum())
            a = a and d < 10
    return a

    return did_the_best_clusters_remain_the_same and has_some_certanty

def prepare_for_active_localization():
    global state, wall_follower_processes
    # at first, attempt with most probable cluster. (this part assumes all the robots have already sent their clusters)
    for robot_id, robot_stuff in state.robots_stuff.items():
        if len(robot_stuff.clusters) == 0:
            print("TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS! TRYING TO START ACTIVE LOCALIZATION WITHOUT SOME ACTIVE ROBOTS!")
            state.running_active_localization = False
            return
        robot_stuff.main_cluster_idx = 0
    for p in wall_follower_processes.values(): p.kill()


def run_active_localization():
    global state

    # check if it's time to update  paths
    if (time.time() - state.paths_last_calculated_at) <= RECOMPUTE_PATHS_PERIOD_S:# seconds
        return
    state.paths_last_calculated_at = time.time()

    # # change main_cluster_idx if required by wall colision
    # for robot_id, robot_stuff in state.robots_stuff.items():
    #     if robot_stuff.path_follower is not None and robot_stuff.path_follower.path_is_probably_obstructed:
    #         print("HIT THE WALL")
    #         robot_stuff.main_cluster_idx += 1
    #         if robot_stuff.laser_sub is not None:
    #             robot_stuff.laser_sub.unregister()
    #             del robot_stuff.path_follower
    #             del robot_stuff.laser_sub

    # for each robot select cluster with main_cluster_idx'th largest weight
    robots_main_cluster = {
        robot_id: robot_stuff.clusters[robot_stuff.current_most_certain_clusters_ids[robot_stuff.main_cluster_idx]]
        for robot_id, robot_stuff in state.robots_stuff.items()
    }

    # path
    robots_paths = {}
    for robot_id, robot_main_clusters in robots_main_cluster.items():
        print(f'{robot_id} robot_main_clusters {robot_main_clusters}')
        robots_paths[robot_id] = [
            path_following.Landmark(*l) for l in
            path_planner.a_star(OCCUPANCY_GRID, (int(robot_main_clusters.x), int(robot_main_clusters.y)), (69, 59))
        ]
    print(f'robots_paths {robots_paths}')

    for robot_id, robot_stuff in state.robots_stuff.items():
        robot_stuff.path_follower = path_following.PathFollower(robots_paths[robot_id])
        robot_stuff.laser_sub = rospy.Subscriber(f'/ugv{str(robot_id)}/scan', LaserScan, robot_stuff.path_follower.clbk_laser)

    print(robot_stuff.path_follower, robot_stuff.laser_sub)
    #### when finished, we need to set running_active_localization = False and unregister callbacks, set None stuff to None

wall_follower_processes = {}
def run_wall_follower(robot_ids):
    import subprocess
    global wall_follower_processes
    for robot_id in robot_ids:
        wall_follower_processes[robot_id] = subprocess.Popen(['python3',  f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/scripts/wall_following.py', f'{robot_id}'])


def main():
    run_wall_follower(sys.argv[1:])
    global state
    state = State()
    rospy.init_node('active_path_following_')
    state.robots_stuff = {
        int(robot_id): RobotStuff(rospy.Publisher(f'/ugv{str(robot_id)}/cmd_vel', Twist, queue_size=10))\
        for robot_id in sys.argv[1:]
    }
    from nav_msgs.msg import Odometry
    from tf import transformations

    def pra_roubar1(m):
        global state
        x = m.pose.pose.position.x * 10.0 + 69.0
        y = m.pose.pose.position.y * 10.0 + 59.0
        angle = transformations.euler_from_quaternion([
            m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w
        ])[2]

        state.robots_stuff[1].x_y_angle_pra_roubar = (x,y,angle)
    def pra_roubar2(m):
        global state
        x = m.pose.pose.position.x * 10.0 + 69.0
        y = m.pose.pose.position.y * 10.0 + 59.0
        angle = transformations.euler_from_quaternion([
            m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w
        ])[2]

        state.robots_stuff[2].x_y_angle_pra_roubar = (x,y,angle)
    rospy.Subscriber(f'/ugv1/odom', Odometry, pra_roubar1)
    rospy.Subscriber(f'/ugv2/odom', Odometry, pra_roubar2)

    _clusters_subscriber = rospy.Subscriber('robot_inf_broadcast', ParticlesClusters, clusters_recv_cb)
    rate = rospy.Rate(25)

    while not rospy.is_shutdown():
        publish_control_messages()
        rate.sleep()


def publish_control_messages():
    global state
    # if state.robots_stuff[1].actuator is not None and state.robots_stuff[1].path_follower.control_msg is not None:
    #     state.robots_stuff[1].actuator.publish(state.robots_stuff[1].path_follower.control_msg)
    for robot_id, robot_stuff in state.robots_stuff.items():
        if robot_stuff.path_follower is not None:
            robot_stuff.actuator.publish(robot_stuff.path_follower.control_msg)


if __name__ == '__main__':
    main()
