#! /usr/bin/env python

# import ros stuff
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'back up going right',
    4: 'back up going left',
}


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(msg.ranges[-90], 10),  # -90 Right
        'fright': min(msg.ranges[-45], 10),  # -45 Front right
        'front':  min(msg.ranges[0], 10), # 0 Front
        'fleft':  min(msg.ranges[45], 10),  # 45 Front Left
        'left':   min(msg.ranges[90], 10),  # 90 - Left
    }
    nearest_frontal_distance = min(msg.ranges[-60:] + msg.ranges[:60])
    will_colide = nearest_frontal_distance < 0.25
    if will_colide:
        scans = np.asarray(msg.ranges)
        scans[scans > 3] = 3
        scan_vectors = np.vstack([
            np.asarray([np.sin(angle*np.pi/180)*scans[angle],
                        np.cos(angle*np.pi/180)*scans[angle]])
            for angle in np.arange(360)
        ])
        normal = scan_vectors.mean(axis=0)

        if normal[0] < 0:
            change_state(3)  # backup going right
        else:
            change_state(4)  # back up going left
    else:
        take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 2

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.4
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.4
    return msg


def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5

    return msg


def back_up(state):
    msg = Twist()
    msg.angular.z = 0.5 if state == 3 else -0.5
    msg.linear.x = -0.1
    return msg


def main():
    import sys
    robot_suffix = sys.argv[1]
    global pub_

    rospy.init_node(f'wall_following_{robot_suffix}')
    laser_topic = '/ugv' + str(robot_suffix) + '/scan'
    vel_topic = '/ugv' + str(robot_suffix) + '/cmd_vel'
    print(laser_topic)
    print(vel_topic)

    pub_ = rospy.Publisher(vel_topic, Twist, queue_size=10)

    sub = rospy.Subscriber(laser_topic, LaserScan, clbk_laser)

    rate = rospy.Rate(20)

    print('Wall follower - [%s]' % (state_))

    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_ == 3 or state_ == 4:
            msg = back_up(state_)
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
