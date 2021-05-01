### add rooms model to gazebo models path:

```bash
export GAZEBO_MODEL_PATH="$(rospack find multi_robot_localization)/models:${GAZEBO_MODEL_PATH}"
```

it is recommended to add this line in the end of you `~/.bashrc` so the `GAZEBO_MODEL_PATH` can be aways set.

### building:

```bash
$ catkin_make --only-pkg-with-deps multi_robo
```

### running the gazebo environment (it also serves as roscore)

```bash
roslaunch -v multi_robot_localization main.launch n_robots:=3
```

### running the node (it subscribes and publishes stuff)

```bash
$ rosrun multi_robot_localization robot_node 1
$ rosrun multi_robot_localization robot_node 2
$ rosrun multi_robot_localization robot_node 3
...
```

### running the map server

```bash
python3 scripts/map_server.py
```

### running the manual teleoperator

```bash
rosrun multi_robot_localization scripts/turtlebot3_teleop_key /ugv1/cmd_vel 1
rosrun multi_robot_localization scripts/turtlebot3_teleop_key /ugv2/cmd_vel 2
rosrun multi_robot_localization scripts/turtlebot3_teleop_key /ugv3/cmd_vel 3
...
```
