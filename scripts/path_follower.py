import numpy as np
import rospy
import os
import re
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf import transformations
from nav_msgs.msg import Odometry

LANDMARK_DETECTION_DISTANCE = 3
with open(os.environ["HOME"] + "/catkin_ws/src/multi_robot_localization/include/occupancy_grid.h", mode='r') as occupancy_grid:
    text = occupancy_grid.read()
    X_CENTER = float(re.findall(r'X_CENTER \((.*?)\)', text)[0])
    Y_CENTER = float(re.findall(r'Y_CENTER \((.*?)\)', text)[0])
    CELLS_PER_METER = float(re.findall(r'CELLS_PER_METER \((.*?)\)', text)[0])

actuator_publisher = None
path_landmarks = [(10, 20), (25, 20), (25, 62),
                  (115, 62), (115, 105), (130, 105)]


def clbk_odometry(msg):
    x = msg.pose.pose.position.x * CELLS_PER_METER + X_CENTER
    y = msg.pose.pose.position.y * CELLS_PER_METER + Y_CENTER
    angle = transformations.euler_from_quaternion([
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    # 1- check if I can clear a landmark (and then reduce speed a little)
    # 2- compute error until the next landmark
    # 3- change angle aiming to reach the next (reduce speed a little)
    # 4- go to the next landmark full speed
    # 5- go back to '1'

    control_msg = Twist()
    control_msg.linear.x = 0.2
    control_msg.angular.z = -0.4

    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", x, y, angle*180/np.pi)


def main():
    import sys
    robot_suffix = sys.argv[1]
    global actuator_publisher

    rospy.init_node(f'path_follower_{robot_suffix}')
    odometry_topic = '/ugv' + str(robot_suffix) + '/odom'
    vel_topic = '/ugv' + str(robot_suffix) + '/cmd_vel'
    print(vel_topic)

    actuator_publisher = rospy.Publisher(vel_topic, Twist, queue_size=1)
    sub = rospy.Subscriber(odometry_topic, Odometry, clbk_odometry)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
