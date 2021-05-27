#!/bin/bash
set -e

function kill_processes {
    echo "Stopping processes..."
    sleep 2
    pkill -f robot_node
    pkill -f tests
    pkill -f gazebo
    pkill -f python3
    pkill -f noetic
}

function run_trial {
    roslaunch -v multi_robot_localization main.launch n_robots:=2 &
    sleep 4

    rosrun multi_robot_localization robot_node 1 & \
    python3 src/multi_robot_localization/scripts/tests.py follow_path_from_file 1 & \
    rosrun multi_robot_localization robot_node 2 & \
    python3 src/multi_robot_localization/scripts/tests.py follow_path_from_file 2 &
}

trap kill_processes SIGINT

for TRIAL in {18..50}; do
    echo "==========================================================="
    echo " Running trial ${TRIAL}... "
    echo "==========================================================="

    TRIAL=${TRIAL} run_trial

    sleep 390
    kill_processes
    sleep 4
done
