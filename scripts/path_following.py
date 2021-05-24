import numpy as np
import path_planner
import rospy
import re
import os

from dataclasses import dataclass, field
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations


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
        angle_error = np.fmod(angle_error+4*np.pi, 2*np.pi)

        if angle_error < np.pi:
            err = -angle_error
        else:
            err = (2*np.pi - angle_error)

        return (6 / np.pi) * err

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
