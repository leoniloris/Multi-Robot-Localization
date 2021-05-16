#!/bin/bash
set -e

function kill_processes {
    pkill -f robot_node
    pkill -f wall_following
    pkill -f gazebo
    pkill -f python3
    pkill -f noetic
}

function run_trial {
    roslaunch -v multi_robot_localization main.launch n_robots:=2 &
    sleep 5

    rosrun multi_robot_localization robot_node 1 & \
    python3 src/multi_robot_localization/scripts/wall_following.py 1 & \
    rosrun multi_robot_localization robot_node 2 & \
    python3 src/multi_robot_localization/scripts/wall_following.py 2 &
}

trap kill_processes SIGINT

for TRIAL in {1..10}; do
    echo "==========================================================="
    echo "================ Running trial ${TRIAL}... ================"
    echo "==========================================================="
    TRIAL=${TRIAL} run_trial
    sleep 30
    kill_processes
done
