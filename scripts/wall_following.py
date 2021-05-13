#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import math

pub_ = None
angles = np.arange(-180, 180)

mvmt_msg = Twist()


def clbk_laser(msg):
    global pub_, mvmt_msg
    scans = np.asarray(msg.ranges)
    scans[scans > 0.4] = 0.4

    scan_vectors = np.vstack([
        np.asarray([np.sin(angle*np.pi/180)*scans[angle],
                   np.cos(angle*np.pi/180)*scans[angle]])
        for angle in angles
    ])

    # follow in a -90 degrees difference from walls's normal vector (from robot lasers)
    wall_normal = scan_vectors.mean(axis=0)
    rotation = -(np.pi*0.5)
    rotated_wall_normal = wall_normal @ np.asarray([
        [np.cos(rotation), -np.sin(rotation)],
        [np.sin(rotation), np.cos(rotation)]
    ])


    mvmt_msg.linear.x = 0.2  # np.linalg.norm(rotated_wall_normal)
    rotated_wall_normal[0] = 0 if abs(rotated_wall_normal[0]) < 0.01 else rotated_wall_normal[0]
    mvmt_angle = np.arctan2(-rotated_wall_normal[0], rotated_wall_normal[1] )
    print(mvmt_angle*180/np.pi)
    mvmt_msg.angular.z = mvmt_angle


def main():
    global pub_
    global mvmt_msg

    rospy.init_node('reading_laser')

    robot_suffix = 1
    laser_topic = '/ugv' + str(robot_suffix) + '/scan'
    vel_topic = '/ugv' + str(robot_suffix) + '/cmd_vel'

    pub_ = rospy.Publisher(vel_topic, Twist, queue_size=1)
    sub = rospy.Subscriber(laser_topic, LaserScan, clbk_laser)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        pub_.publish(mvmt_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
