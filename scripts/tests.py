import path_following
import numpy as np
import rospy
import sys
import os
import re

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def follow_path_from_file():
    def get_occupancy_grid():
        occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read())[0]
        return np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")


    def get_trajectory_from_file(robot_suffix):
        with open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/trajectories/a_star_{robot_suffix}.txt') as f:
            return [tuple(map(float, i.split(','))) for i in f]

    robot_suffix = sys.argv[2]
    rospy.init_node(f'path_following_{robot_suffix}')
    trajectory = get_trajectory_from_file(robot_suffix)
    path_landmarks = [path_following.Landmark(*l) for l in trajectory]

    import matplotlib.pyplot as plt
    import seaborn as sns
    fig = plt.figure()
    fig.suptitle(f'{robot_suffix}', fontsize=20)
    sns.heatmap(get_occupancy_grid(5))
    plt.scatter(list(zip(*trajectory))[1], list(zip(*trajectory))[0])
    plt.pause(5)


    path_follower = path_following.PathFollower(path_landmarks)
    actuator = rospy.Publisher('/ugv' + str(robot_suffix) + '/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/odom', Odometry, path_follower.clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        actuator.publish(path_follower.control_msg)
        rate.sleep()


def follow_a_star_path():
    def get_occupancy_grid():
        occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read())[0]
        return np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")
    robot_suffix, x1, y1, x2, y2 = sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6])
    rospy.init_node(f'path_following_{robot_suffix}')
    print(f'start={(x1, y1)}, end={(x2, y2)}')
    occupancy_grid = get_occupancy_grid()
    path_landmarks = [path_following.Landmark(*l) for l in path_planner.a_star(occupancy_grid, (x1, y1), (x2, y2))]

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
