#!/bin/bash
set -e

[ -z ${TRIAL+x} ] && echo "Needs to define the variable TRIAL" && exit 1

function kill_processes {
    sudo pkill -f robot_node && sudo pkill -f wall_following
}

trap kill_processes SIGINT

rosrun multi_robot_localization robot_node 1 & \
python3 src/multi_robot_localization/scripts/wall_following.py 1 & \
rosrun multi_robot_localization robot_node 2 & \
python3 src/multi_robot_localization/scripts/wall_following.py 2
