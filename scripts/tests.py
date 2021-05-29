import path_following

import numpy as np
import rospy
import sys
import os
import re

from path_following import Landmark
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def follow_path_from_file():
    def get_occupancy_grid():
        occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read())[0]
        return np.loadtxt(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}', delimiter=",")

    def get_trajectory_from_file(robot_suffix):
        with open(f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/trajectories/a_star_{robot_suffix}_obstacles.txt') as f:
            return [tuple(map(float, i.split(','))) for i in f]

    robot_suffix = sys.argv[2]
    rospy.init_node(f'path_following_{robot_suffix}')
    # trajectory = get_trajectory_from_file(robot_suffix)
    # path_landmarks = [Landmark(*l) for l in trajectory]

    path_landmarks = [Landmark(x=41.0, y=6.0, checked=False), Landmark(x=41.5, y=6.5, checked=False), Landmark(x=41.666666666666664, y=7.0, checked=False), Landmark(x=41.75, y=7.5, checked=False), Landmark(x=41.8, y=9.0, checked=False), Landmark(x=42.0, y=10.0, checked=False), Landmark(x=42.142857142857146, y=11.0, checked=False), Landmark(x=42.125, y=11.5, checked=False), Landmark(x=42.0, y=12.25, checked=False), Landmark(x=41.8, y=12.4, checked=False), Landmark(x=41.54545454545455, y=12.5, checked=False), Landmark(x=41.2, y=12.571428571428571, checked=False), Landmark(x=40.7, y=12.5, checked=False), Landmark(x=40.1, y=12.5, checked=False), Landmark(x=40.1, y=12.5, checked=False), Landmark(x=40.375, y=12.5, checked=False), Landmark(x=40.375, y=12.5, checked=False), Landmark(x=40.375, y=12.5, checked=False), Landmark(x=39.5, y=12.75, checked=False), Landmark(x=38.5, y=13.0, checked=False), Landmark(x=37.625, y=13.25, checked=False), Landmark(x=36.875, y=13.625, checked=False), Landmark(x=36.125, y=13.833333333333334, checked=False), Landmark(x=35.5, y=14.5, checked=False), Landmark(x=35.125, y=15.5, checked=False), Landmark(x=35.1, y=16.5, checked=False), Landmark(x=34.8, y=17.5, checked=False), Landmark(x=34.92857142857143, y=18.5, checked=False), Landmark(x=35.0, y=19.5, checked=False), Landmark(x=35.27777777777778, y=20.5, checked=False), Landmark(x=35.125, y=21.5, checked=False), Landmark(x=35.142857142857146, y=22.5, checked=False), Landmark(x=35.1, y=23.5, checked=False), Landmark(x=35.4, y=24.5, checked=False), Landmark(x=35.5, y=25.5, checked=False), Landmark(x=36.125, y=26.333333333333332, checked=False), Landmark(x=36.75, y=27.166666666666668, checked=False), Landmark(x=37.5, y=27.833333333333332, checked=False), Landmark(x=38.5, y=28.5, checked=False), Landmark(x=38.9, y=29.166666666666668, checked=False), Landmark(x=39.3, y=29.875, checked=False), Landmark(x=39.5, y=30.375, checked=False), Landmark(x=40.0, y=30.75, checked=False), Landmark(x=39.75, y=31.0, checked=False), Landmark(x=39.25, y=31.083333333333332, checked=False), Landmark(x=38.5, y=31.142857142857142, checked=False), Landmark(x=37.5, y=31.25, checked=False), Landmark(x=36.5, y=31.22222222222222, checked=False), Landmark(x=35.5, y=31.38888888888889, checked=False), Landmark(x=34.5, y=31.555555555555557, checked=False), Landmark(x=33.5, y=31.72222222222222, checked=False), Landmark(x=32.5, y=31.88888888888889, checked=False), Landmark(x=31.5, y=32.05555555555556, checked=False), Landmark(x=30.5, y=32.0625, checked=False), Landmark(x=29.5, y=31.916666666666668, checked=False), Landmark(x=28.5, y=32.0, checked=False), Landmark(x=27.5, y=32.4, checked=False), Landmark(x=26.666666666666668, y=32.5, checked=False), Landmark(x=26.0, y=33.0, checked=False), Landmark(x=25.75, y=33.666666666666664, checked=False), Landmark(x=25.375, y=34.5, checked=False), Landmark(x=25.3, y=35.5, checked=False), Landmark(x=25.25, y=36.5, checked=False), Landmark(x=25.214285714285715, y=37.5, checked=False), Landmark(x=25.375, y=38.5, checked=False), Landmark(x=25.4375, y=39.5, checked=False), Landmark(x=25.428571428571427, y=40.5, checked=False), Landmark(x=25.5, y=41.5, checked=False), Landmark(x=25.6, y=42.5, checked=False), Landmark(x=25.75, y=43.5, checked=False), Landmark(x=26.25, y=44.5, checked=False), Landmark(x=26.666666666666668, y=45.333333333333336, checked=False), Landmark(x=27.5, y=46.0, checked=False), Landmark(x=28.5, y=46.25, checked=False), Landmark(x=29.5, y=46.625, checked=False), Landmark(x=30.5, y=47.0, checked=False), Landmark(x=31.5, y=47.2, checked=False), Landmark(x=32.5, y=47.333333333333336, checked=False), Landmark(x=33.5, y=47.5, checked=False), Landmark(x=34.5, y=47.6875, checked=False), Landmark(x=35.5, y=47.833333333333336, checked=False), Landmark(x=36.5, y=48.05555555555556, checked=False), Landmark(x=37.5, y=48.22222222222222, checked=False), Landmark(x=38.5, y=48.388888888888886, checked=False), Landmark(x=39.5, y=48.55555555555556, checked=False), Landmark(x=40.5, y=48.72222222222222, checked=False), Landmark(x=41.5, y=48.94444444444444, checked=False), Landmark(x=42.5, y=49.166666666666664, checked=False), Landmark(x=43.5, y=49.333333333333336, checked=False), Landmark(x=44.5, y=49.5, checked=False), Landmark(x=45.5, y=49.72222222222222, checked=False), Landmark(x=46.5, y=49.94444444444444, checked=False), Landmark(x=47.5, y=50.166666666666664, checked=False), Landmark(x=48.5, y=50.388888888888886, checked=False), Landmark(x=49.5, y=50.611111111111114, checked=False), Landmark(x=50.5, y=50.8125, checked=False), Landmark(x=51.5, y=51.0625, checked=False), Landmark(x=52.5, y=51.3125, checked=False), Landmark(x=53.5, y=51.5, checked=False), Landmark(x=54.5, y=51.8125, checked=False), Landmark(x=55.5, y=52.07142857142857, checked=False), Landmark(x=56.5, y=52.357142857142854, checked=False), Landmark(x=57.5, y=52.583333333333336, checked=False), Landmark(x=58.5, y=52.916666666666664, checked=False), Landmark(x=59.5, y=53.2, checked=False), Landmark(x=60.5, y=53.5, checked=False), Landmark(x=61.5, y=53.9, checked=False), Landmark(x=62.5, y=54.4, checked=False), Landmark(x=63.5, y=54.9, checked=False), Landmark(x=64.5, y=55.5, checked=False), Landmark(x=65.5, y=55.77777777777778, checked=False), Landmark(x=66.5, y=56.0, checked=False), Landmark(x=67.0, y=56.0, checked=False), Landmark(x=67.0, y=56.285714285714285, checked=False), Landmark(x=67.5, y=56.666666666666664, checked=False)] if str(robot_suffix) == '1' else\
                     [Landmark(x=97.0, y=106.0, checked=False), Landmark(x=96.5, y=105.5, checked=False), Landmark(x=96.0, y=105.0, checked=False), Landmark(x=95.0, y=104.0, checked=False), Landmark(x=93.5, y=102.5, checked=False), Landmark(x=92.5, y=101.5, checked=False), Landmark(x=92.0, y=101.0, checked=False), Landmark(x=91.0, y=100.0, checked=False), Landmark(x=90.5, y=99.5, checked=False), Landmark(x=90.0, y=99.0, checked=False), Landmark(x=89.0, y=98.0, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=88.5, y=97.5, checked=False), Landmark(x=87.5, y=96.5, checked=False), Landmark(x=86.5, y=95.5, checked=False), Landmark(x=85.5, y=94.5, checked=False), Landmark(x=84.5, y=93.5, checked=False), Landmark(x=83.5, y=92.5, checked=False), Landmark(x=82.5, y=91.5, checked=False), Landmark(x=81.5, y=90.5, checked=False), Landmark(x=80.5, y=89.5, checked=False), Landmark(x=80.25, y=88.5, checked=False), Landmark(x=80.0, y=87.66666666666667, checked=False), Landmark(x=80.0, y=87.0, checked=False), Landmark(x=80.25, y=86.75, checked=False), Landmark(x=80.75, y=86.375, checked=False), Landmark(x=81.5, y=86.3, checked=False), Landmark(x=82.5, y=86.25, checked=False), Landmark(x=83.5, y=86.21428571428571, checked=False), Landmark(x=84.5, y=86.33333333333333, checked=False), Landmark(x=85.5, y=86.16666666666667, checked=False), Landmark(x=86.5, y=86.05555555555556, checked=False), Landmark(x=87.5, y=86.0, checked=False), Landmark(x=88.5, y=86.0, checked=False), Landmark(x=89.5, y=86.0, checked=False), Landmark(x=90.5, y=86.0, checked=False), Landmark(x=91.5, y=86.0, checked=False), Landmark(x=92.5, y=86.0, checked=False), Landmark(x=93.5, y=86.0, checked=False), Landmark(x=94.5, y=86.0, checked=False), Landmark(x=95.5, y=86.0, checked=False), Landmark(x=96.5, y=86.0, checked=False), Landmark(x=97.5, y=86.0, checked=False), Landmark(x=98.5, y=86.0, checked=False), Landmark(x=99.5, y=86.0, checked=False), Landmark(x=100.5, y=86.0, checked=False), Landmark(x=101.5, y=86.0, checked=False), Landmark(x=102.5, y=86.0, checked=False), Landmark(x=103.5, y=86.0, checked=False), Landmark(x=104.5, y=85.94444444444444, checked=False), Landmark(x=105.5, y=85.83333333333333, checked=False), Landmark(x=106.5, y=85.66666666666667, checked=False), Landmark(x=107.5, y=85.44444444444444, checked=False), Landmark(x=108.5, y=85.57142857142857, checked=False), Landmark(x=109.5, y=85.5, checked=False), Landmark(x=110.5, y=85.4, checked=False), Landmark(x=111.33333333333333, y=85.25, checked=False), Landmark(x=112.0, y=84.75, checked=False), Landmark(x=112.25, y=84.33333333333333, checked=False), Landmark(x=112.625, y=83.5, checked=False), Landmark(x=112.7, y=82.5, checked=False), Landmark(x=112.75, y=81.5, checked=False), Landmark(x=112.78571428571429, y=80.5, checked=False), Landmark(x=112.625, y=79.5, checked=False), Landmark(x=112.5625, y=78.5, checked=False), Landmark(x=112.57142857142857, y=77.5, checked=False), Landmark(x=112.5, y=76.5, checked=False), Landmark(x=112.4, y=75.5, checked=False), Landmark(x=112.25, y=74.5, checked=False), Landmark(x=111.75, y=73.5, checked=False), Landmark(x=111.33333333333333, y=72.5, checked=False), Landmark(x=110.5, y=71.5, checked=False), Landmark(x=109.5, y=70.5, checked=False), Landmark(x=108.5, y=69.5, checked=False), Landmark(x=107.5, y=68.5, checked=False), Landmark(x=106.5, y=67.5, checked=False), Landmark(x=105.5, y=66.5, checked=False), Landmark(x=104.5, y=65.5, checked=False), Landmark(x=103.5, y=64.5, checked=False), Landmark(x=102.5, y=63.5, checked=False), Landmark(x=101.5, y=62.5, checked=False), Landmark(x=100.5, y=61.5, checked=False), Landmark(x=99.5, y=60.5, checked=False), Landmark(x=98.5, y=59.5, checked=False), Landmark(x=97.5, y=58.5, checked=False), Landmark(x=96.5, y=57.5, checked=False), Landmark(x=95.5, y=56.5, checked=False), Landmark(x=94.5, y=55.5, checked=False), Landmark(x=93.5, y=54.666666666666664, checked=False), Landmark(x=92.5, y=54.25, checked=False), Landmark(x=91.5, y=53.75, checked=False), Landmark(x=90.5, y=53.375, checked=False), Landmark(x=89.5, y=53.5, checked=False), Landmark(x=88.5, y=53.583333333333336, checked=False), Landmark(x=87.5, y=53.9375, checked=False), Landmark(x=86.5, y=54.05555555555556, checked=False), Landmark(x=85.5, y=54.0, checked=False), Landmark(x=84.5, y=54.05555555555556, checked=False), Landmark(x=83.5, y=54.166666666666664, checked=False), Landmark(x=82.5, y=54.333333333333336, checked=False), Landmark(x=81.5, y=54.55555555555556, checked=False), Landmark(x=80.5, y=54.75, checked=False), Landmark(x=79.5, y=55.0, checked=False), Landmark(x=78.5, y=55.25, checked=False), Landmark(x=77.5, y=55.42857142857143, checked=False), Landmark(x=76.5, y=55.714285714285715, checked=False), Landmark(x=75.5, y=56.07142857142857, checked=False), Landmark(x=74.5, y=56.07142857142857, checked=False), Landmark(x=73.5, y=56.23076923076923, checked=False), Landmark(x=72.5, y=56.333333333333336, checked=False), Landmark(x=71.5, y=56.45454545454545, checked=False), Landmark(x=71.0, y=56.45454545454545, checked=False), Landmark(x=71.0, y=56.6, checked=False), Landmark(x=70.5, y=56.77777777777778, checked=False)]
    trajectory = [(l.x,l.y) for l in path_landmarks]

    import matplotlib.pyplot as plt
    import seaborn as sns
    fig = plt.figure()
    fig.suptitle(f'{robot_suffix}', fontsize=20)
    sns.heatmap(get_occupancy_grid())
    plt.scatter(list(zip(*trajectory))[1], list(zip(*trajectory))[0])
    plt.pause(5)


    path_follower = path_following.PathFollower(path_landmarks)
    actuator = rospy.Publisher('/ugv' + str(robot_suffix) + '/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/odom', Odometry, path_follower.clbk_odometry)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/scan', LaserScan, path_follower.clbk_laser)

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
    path_landmarks = [Landmark(*l) for l in path_planner.a_star(occupancy_grid, (x1, y1), (x2, y2))]

    path_follower = path_following.PathFollower(path_landmarks)
    actuator = rospy.Publisher('/ugv' + str(robot_suffix) + '/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/odom', Odometry, path_follower.clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        actuator.publish(path_follower.control_msg)
        rate.sleep()


if __name__ == '__main__':
    test_to_run = sys.argv[1]
    # exec(test_to_run)
    follow_path_from_file()
