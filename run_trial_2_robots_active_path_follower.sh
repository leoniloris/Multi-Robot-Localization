#!/bin/bash
set -e

function kill_processes {
    echo "Stopping processes..."
    sleep 2
    pkill -f robot_node
    pkill -f active_path_following
    pkill -f multi_robot_localization
    pkill -f gazebo
    pkill -f python3
    pkill -f noetic
    pkill -f map_server
}

function run_trial {
    roslaunch -v multi_robot_localization main.launch n_robots:=2 &
    sleep 4

    python3 src/multi_robot_localization/scripts/active_path_following.py 1 2 & \
    rosrun multi_robot_localization robot_node 1 & \
    rosrun multi_robot_localization robot_node 2 & \
    python3 src/multi_robot_localization/scripts/map_server.py &
}

trap kill_processes SIGINT

for TRIAL in {1..1}; do
    echo "==========================================================="
    echo " Running trial ${TRIAL}... "
    echo "==========================================================="

    TRIAL=${TRIAL} run_trial

    sleep 420
    kill_processes
    sleep 4
done
