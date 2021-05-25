from multi_robot_localization.msg import clusters as ParticlesClusters
from dataclasses import dataclass, field
from geometry_msgs.msg import Twist

import path_following
import path_planner
import rospy
import sys

N_MOST_CERTAIN_CLUSTERS = 2

robots_stuff = {}
debounce_counter = 0
def debounce():
    global debounce_counter
    debounce_counter += 1
    if debounce_counter < 5:
        return True
    debounce_counter = 0
    return False

@dataclass
class RobotStuff:
    actuator: rospy.Publisher
    path_follower: 'typing.Any'  # path_following.PathFollower
    clusters: 'typing.Any'  # ParticlesClusters


def clusters_recv_cb(msg):
    print("-------------------------")
    global robots_stuff
    if debounce(): return

    most_certain_clusters = sorted(msg.clusters, key=lambda c: c.weight)[-N_MOST_CERTAIN_CLUSTERS:]
    robots_stuff[msg.origin_robot_index].clusters = most_certain_clusters
    print(robots_stuff[msg.origin_robot_index].clusters)


def main():
    global robots_stuff
    robots_stuff = {
        int(robot_suffix): RobotStuff(
            rospy.Publisher(
                f'/ugv{str(robot_suffix)}/cmd_vel', Twist, queue_size=1),
            None, None
        ) for robot_suffix in sys.argv[1:]
    }

    print(robots_stuff)
    rospy.init_node(f'active_path_following')
    sub = rospy.Subscriber('robot_inf_broadcast',
                           ParticlesClusters, clusters_recv_cb)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for robot_id, robot_stuff in robots_stuff.items():
            if robot_stuff.path_follower is not None:
                actuator.publish(robot_stuff.path_follower.control_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
