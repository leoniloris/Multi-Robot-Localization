import numpy as np
import rospy
import os
import re

from dataclasses import dataclass, field
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from itertools import cycle
import path_planer


@dataclass
class Landmark:
    x: float
    y: float
    checked: bool = field(default=False)

    def position(self):
        return np.asarray((self.x, self.y))


LANDMARK_DETECTION_DISTANCE = 3
actuator = None
control_msg = Twist()

current_target_idx = 0
path_landmarks = [
    Landmark(*l) for l in [(10, 20), (25, 20), (25, 62), (115, 62), (115, 105), (130, 105)]
]


with open(os.environ["HOME"] + "/catkin_ws/src/multi_robot_localization/include/occupancy_grid.h", mode='r') as occupancy_grid_file:
    text = occupancy_grid_file.read()
    X_CENTER = float(re.findall(r'X_CENTER \((.*?)\)', text)[0])
    Y_CENTER = float(re.findall(r'Y_CENTER \((.*?)\)', text)[0])
    CELLS_PER_METER = float(re.findall(r'CELLS_PER_METER \((.*?)\)', text)[0])


def clbk_odometry(msg):
    global control_msg
    x = msg.pose.pose.position.x * CELLS_PER_METER + X_CENTER
    y = msg.pose.pose.position.y * CELLS_PER_METER + Y_CENTER
    angle = transformations.euler_from_quaternion([
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
    ])[2]

    target = get_target(x, y)
    if target.checked:
        control_msg.angular.z = 0
        control_msg.linear.x = 0
        return

    position_error = target.position() - np.asarray([x, y])
    angle_error = angle - np.arctan2(*position_error[-1::-1])
    control_msg.angular.z = control_angle(angle_error)
    control_msg.linear.x = control_position(position_error, angle_error)
    clear_past_targets(x, y)


# integral_err = 0
def control_angle(angle_error):
    global integral_err
    angle_error = np.fmod(angle_error+4*np.pi, 2*np.pi)

    if angle_error < np.pi:
        err = -angle_error
    else:
        err = (2*np.pi - angle_error)

    # integral error just messes things up
    # integral_err += err
    return (3 / np.pi) * err  # + 0.006*integral_err


def control_position(position_error, angle_error):
    angle_error = np.fmod(angle_error+4*np.pi, 2*np.pi)
    direction = 1 if angle_error < np.pi/2 or angle_error > 3*np.pi/2 else -1
    distance_error = np.clip(np.linalg.norm(position_error), 0.1, 1)
    return direction * 0.4 * distance_error


def get_target(x, y):
    return next((path_landmark for path_landmark in path_landmarks if not path_landmark.checked), path_landmarks[0])


def clear_past_targets(x, y):
    for path_landmark in path_landmarks:
        if np.linalg.norm(path_landmark.position() - np.asarray((x, y))) <= LANDMARK_DETECTION_DISTANCE:
            path_landmark.checked = True


def get_crammed_occupancy_grid():
    from scipy.ndimage import convolve
    occupancy_grid =\
        np.loadtxt(os.environ["HOME"] +
                   "/catkin_ws/src/multi_robot_localization/occupancy_grid/rooms_small.csv", delimiter=",")
    cramming_kernel = np.ones((5, 5))
    return np.clip(convolve(occupancy_grid, cramming_kernel), a_min=0, a_max=1)


def main():
    import sys
    robot_suffix = sys.argv[1]
    global actuator, crammed_occupancy_grid
    crammed_occupancy_grid = get_crammed_occupancy_grid()
    rospy.init_node(f'path_follower_{robot_suffix}')
    odometry_topic = '/ugv' + str(robot_suffix) + '/odom'
    vel_topic = '/ugv' + str(robot_suffix) + '/cmd_vel'
    print(vel_topic)

    actuator = rospy.Publisher(vel_topic, Twist, queue_size=1)
    sub = rospy.Subscriber(odometry_topic, Odometry, clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        actuator.publish(control_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
