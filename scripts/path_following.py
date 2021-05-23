import numpy as np
import path_planner
import rospy
import sys
import os
import re

from dataclasses import dataclass, field
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from itertools import cycle


@dataclass
class Landmark:
    x: float
    y: float
    checked: bool = field(default=False)

    def position(self):
        return np.asarray((self.x, self.y))


LANDMARK_DETECTION_DISTANCE = 3
with open(os.environ["HOME"] + "/catkin_ws/src/multi_robot_localization/include/occupancy_grid.h", mode='r') as occupancy_grid_file:
    text = occupancy_grid_file.read()
    X_CENTER = float(re.findall(r'X_CENTER \((.*?)\)', text)[0])
    Y_CENTER = float(re.findall(r'Y_CENTER \((.*?)\)', text)[0])
    CELLS_PER_METER = float(re.findall(r'CELLS_PER_METER \((.*?)\)', text)[0])


class PathFollower:
    def __init__(self, path_landmarks):
        self.control_msg = Twist()
        self.path_landmarks = path_landmarks

    def control_angle(self, angle_error):
        global integral_err
        angle_error = np.fmod(angle_error+4*np.pi, 2*np.pi)

        if angle_error < np.pi:
            err = -angle_error
        else:
            err = (2*np.pi - angle_error)

        return (3 / np.pi) * err

    def clbk_odometry(self, msg):
        x = msg.pose.pose.position.x * CELLS_PER_METER + X_CENTER
        y = msg.pose.pose.position.y * CELLS_PER_METER + Y_CENTER
        angle = transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        ])[2]

        target = self.get_target(x, y)
        if target.checked:
            self.control_msg.angular.z = 0
            self.control_msg.linear.x = 0
            return

        position_error = target.position() - np.asarray([x, y])
        angle_error = angle - np.arctan2(*position_error[-1::-1])
        self.control_msg.angular.z = self.control_angle(angle_error)
        self.control_msg.linear.x = self.control_position(
            position_error, angle_error)
        self.clear_past_targets(x, y)

    def control_position(self, position_error, angle_error):
        angle_error = np.fmod(angle_error+4*np.pi, 2*np.pi)
        direction = 1 if angle_error < np.pi/2 or angle_error > 3*np.pi/2 else -1
        distance_error = np.clip(np.linalg.norm(position_error), 0.1, 1)
        return direction * 0.4 * distance_error

    def get_target(self, x, y):
        return next((path_landmark for path_landmark in self.path_landmarks if not path_landmark.checked), self.path_landmarks[0])

    def clear_past_targets(self, x, y):
        for path_landmark in self.path_landmarks:
            if np.linalg.norm(path_landmark.position() - np.asarray((x, y))) <= LANDMARK_DETECTION_DISTANCE:
                path_landmark.checked = True


def get_crammed_occupancy_grid(min_wall_distance):
    from scipy.ndimage import convolve
    import re
    text = open(
        f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/src/occupancy_grid.cpp', mode='r').read()
    occupancy_grid_file = re.findall(r'/occupancy_grid/(.*)"', text)[0]
    occupancy_grid = np.loadtxt(
        f'{os.environ["HOME"]}/catkin_ws/src/multi_robot_localization/occupancy_grid/{occupancy_grid_file}',
        delimiter=",")
    cramming_kernel = np.ones((min_wall_distance*2, min_wall_distance*2))
    return np.clip(convolve(occupancy_grid, cramming_kernel), a_min=0, a_max=1)


def main():
    robot_suffix, min_wall_distance, x1, y1, x2, y2 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6])
    rospy.init_node(f'path_following_{robot_suffix}')
    print(f'min_wall_distance={min_wall_distance}, start={(x1, y1)}, end={(x2, y2)}')
    map_bigger_walls = get_crammed_occupancy_grid(min_wall_distance)
    path_landmarks = [Landmark(*l) for l in path_planner.a_star(map_bigger_walls, (x1, y1), (x2, y2))]

    path_follower = PathFollower(path_landmarks)
    actuator = rospy.Publisher('/ugv' + str(robot_suffix) + '/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/ugv' + str(robot_suffix) + '/odom', Odometry, path_follower.clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        actuator.publish(path_follower.control_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
