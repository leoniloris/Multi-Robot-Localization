#! /usr/bin/env python
from multi_robot_localization.msg import particles as ParticlesMessageType
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_n')
pub_vel = rospy.Publisher('/ugv1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = 0.5

while not rospy.is_shutdown():
    pub_vel.publish(move)
    rate.sleep()
