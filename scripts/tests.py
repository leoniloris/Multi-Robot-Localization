import path_following
import numpy as np
import rospy
import sys
import os
import re

from geometry_msgs.msg import Twist
from scipy.ndimage import convolve
from nav_msgs.msg import Odometry

def follow_path_from_file():
    def get_trajectory_from_file(robot_suffix):
        with open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/trajectories/a_star_{robot_suffix}.txt') as f:
            return [tuple(map(float, i.split(','))) for i in f]

    robot_suffix = sys.argv[2]
    rospy.init_node(f'path_following_{robot_suffix}')
    trajectory = get_trajectory_from_file(robot_suffix)
    path_landmarks = [path_following.Landmark(*l) for l in trajectory]

    path_follower = path_following.PathFollower(path_landmarks)
    actuator = rospy.Publisher('/ugv' + str(robot_suffix) + '/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/odom', Odometry, path_follower.clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        actuator.publish(path_follower.control_msg)
        rate.sleep()


def follow_a_star_path():
    def get_crammed_occupancy_grid(min_wall_distance):
        occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read())[0]
        occupancy_grid = np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")
        cramming_kernel = np.ones((min_wall_distance*2, min_wall_distance*2))
        return np.clip(convolve(occupancy_grid, cramming_kernel), a_min=0, a_max=1)
    robot_suffix, min_wall_distance, x1, y1, x2, y2 = sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7])
    rospy.init_node(f'path_following_{robot_suffix}')
    print(f'min_wall_distance={min_wall_distance}, start={(x1, y1)}, end={(x2, y2)}')
    map_bigger_walls = get_crammed_occupancy_grid(min_wall_distance)
    path_landmarks = [path_following.Landmark(*l) for l in path_planner.a_star(map_bigger_walls, (x1, y1), (x2, y2))]

    path_follower = path_following.PathFollower(path_landmarks)
    actuator = rospy.Publisher('/ugv' + str(robot_suffix) + '/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/odom', Odometry, path_follower.clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        actuator.publish(path_follower.control_msg)
        rate.sleep()


if __name__ == '__main__':
    test_to_run = sys.argv[1]
    # exec(test_to_run)
    follow_path_from_file()
