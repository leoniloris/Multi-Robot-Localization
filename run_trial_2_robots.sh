#!/bin/bash
set -e

function kill_processes {
    sudo pkill -f robot_node
    sudo pkill -f wall_following
    sudo pkill -f gazebo
    sudo pkill -f python3
    sudo pkill -f noetic
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

for TRIAL in {1..5}; do
    echo "==========================================================="
    echo "================ Running trial ${TRIAL}... ================"
    echo "==========================================================="
    run_trial
    sleep 300
    kill_processes
done
