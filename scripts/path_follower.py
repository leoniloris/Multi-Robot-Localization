# import ros stuff
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (Twist, Quaternion)
from tf import transformations
from nav_msgs.msg import Odometry

actuator_publisher = None
path = [(10, 20), (25, 20), (25, 62), (115, 62), (115, 105), (130, 105)]


def clbk_odometry(msg):
    angle = transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", angle*180/np.pi)


def main():
    import sys
    robot_suffix = sys.argv[1]
    global actuator_publisher

    rospy.init_node(f'path_follower_{robot_suffix}')
    laser_topic = '/ugv' + str(robot_suffix) + '/scan'
    odometry_topic = '/ugv' + str(robot_suffix) + '/odom'
    vel_topic = '/ugv' + str(robot_suffix) + '/cmd_vel'
    print(laser_topic)
    print(vel_topic)

    actuator_publisher = rospy.Publisher(vel_topic, Twist, queue_size=1)

    sub = rospy.Subscriber(odometry_topic, Odometry, clbk_odometry)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
