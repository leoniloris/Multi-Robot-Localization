from multi_robot_localization.msg import clusters as ParticlesClusters
from dataclasses import dataclass, field
from geometry_msgs.msg import Twist

import path_following
import path_planner
import rospy
import sys

@dataclass
class RobotStuff:
    actuator: rospy.Publisher
    path_follower: 'typing.Any'  # path_following.PathFollower
    clusters: 'typing.Any'  # ParticlesClusters


def clusters_recv_cb(msg):
    print("-------------------------")
    print(msg.clusters[2], msg.clusters[0])


def main():
    robots_stuff = {
        robot_suffix: RobotStuff(
            rospy.Publisher(f'/ugv{str(robot_suffix)}/cmd_vel', Twist, queue_size=1),
            None, None
        ) for robot_suffix in sys.argv[1:]
    }

    rospy.init_node(f'active_path_following')
    sub = rospy.Subscriber('robot_inf_broadcast', ParticlesClusters, clusters_recv_cb)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for robot_id, robot_stuff in robots_stuff.items():
            if robot_stuff.path_follower is not None:
                actuator.publish(robot_stuff.path_follower.control_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
