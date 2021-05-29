import enum
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


with open(os.environ["HOME"] + "/catkin_ws/src/multi_robot_localization/include/occupancy_grid.h", mode='r') as occupancy_grid_file:
    text = occupancy_grid_file.read()
    X_CENTER = float(re.findall(r'X_CENTER \((.*?)\)', text)[0])
    Y_CENTER = float(re.findall(r'Y_CENTER \((.*?)\)', text)[0])
    CELLS_PER_METER = float(re.findall(r'CELLS_PER_METER \((.*?)\)', text)[0])


LANDMARK_DETECTION_DISTANCE = 0.8
AVOIDING_OBSTACLE_COUNTER_EXPIRITY = 12

class PathFollower:
    def __init__(self, path_landmarks):
        self.control_msg = Twist()
        self.path_landmarks = path_landmarks
        self.avoiding_obstacle = False
        self.avoiding_obstacle_counter = 0

        self.path_is_probably_obstructed = False

    def control_pose(self, position_error, angle_error):
        if self.avoiding_obstacle: return

        angle_error = np.fmod(angle_error+4*np.pi, 2*np.pi)
        angle_error_for_actuation = -angle_error if angle_error < np.pi else (2*np.pi - angle_error)
        angle_error_for_actuation = np.sign(angle_error_for_actuation) * np.clip(abs(angle_error_for_actuation), 0, 1)
        angle_actuation = (4 / np.pi) * angle_error_for_actuation

        direction = 1 if angle_error < np.pi/2 or angle_error > 3*np.pi/2 else -1
        distance_error = np.clip(np.linalg.norm(position_error), 0, 1)
        gain = 0.4 - 0.37*(np.clip(abs(angle_error_for_actuation), 0, 45*np.pi/180)/(45*np.pi/180) )
        distance_actuation = direction * gain * distance_error
        self.control_msg.angular.z, self.control_msg.linear.x = angle_actuation, distance_actuation

    def clbk_laser(self, msg):
        nearest_frontal_distance = min(msg.ranges[-60:] + msg.ranges[:60])
        should_avoid_obstacle = nearest_frontal_distance < 0.25


        if should_avoid_obstacle:
            scans = np.asarray(msg.ranges)
            scans[scans > 3] = 3
            scan_vectors = np.vstack([
                np.asarray([np.sin(angle*np.pi/180)*scans[angle],
                            np.cos(angle*np.pi/180)*scans[angle]])
                for angle in np.arange(360)
            ])
            normal = scan_vectors.mean(axis=0)

            self.avoiding_obstacle = True
            self.path_is_probably_obstructed = True
            if normal[0] < 0:
                # backup going right
                self.control_msg.angular.z = 0.5
                self.control_msg.linear.x = -0.2
            else:
                # back up going left
                self.control_msg.angular.z = 0.5
                self.control_msg.linear.x = -0.2


        should_go_past_obstacle = self.avoiding_obstacle and not should_avoid_obstacle
        if should_go_past_obstacle:
            self.control_msg.linear.x = 0.3
            self.control_msg.angular.z = 0
            self.avoiding_obstacle_counter += 1
            if self.avoiding_obstacle_counter >= AVOIDING_OBSTACLE_COUNTER_EXPIRITY:
                self.avoiding_obstacle = False
                self.avoiding_obstacle_counter = 0


    def clbk_odometry(self, msg):
        x = msg.pose.pose.position.x * CELLS_PER_METER + X_CENTER
        y = msg.pose.pose.position.y * CELLS_PER_METER + Y_CENTER
        angle = transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        ])[2]
        self.control_pose_from_setpoint(x, y, angle)


    def control_pose_from_setpoint(self, robot_x, robot_y, robot_angle):
        target = self.get_target()
        if target is None or target.checked:
            self.control_msg.angular.z = 0
            self.control_msg.linear.x = 0
            return

        position_error = target.position() - np.asarray([robot_x, robot_y])
        angle_error = robot_angle - np.arctan2(*position_error[-1::-1])
        self.control_pose(position_error, angle_error)

        self.clear_past_targets(robot_x, robot_y)


    def get_target(self):
        for path_landmark in self.path_landmarks:
            if not path_landmark.checked:
                return path_landmark

    def clear_past_targets(self, x, y):
        found_near_target = False
        for path_landmark in self.path_landmarks:
            is_near = np.linalg.norm(path_landmark.position() - np.asarray((x, y))) <= LANDMARK_DETECTION_DISTANCE
            if is_near and not path_landmark.checked:
                found_near_target = True
                path_landmark.checked = True
            elif not is_near and found_near_target:
                return
