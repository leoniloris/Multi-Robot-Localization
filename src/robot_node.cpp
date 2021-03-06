#include <ros/console.h>
#include <stdint.h>

#include <string>

#include "occupancy_grid.h"
#include "robot.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

static uint8_t parse_robot_index_from_arguments(char** argv) {
    if (argv[1] == NULL) {
        ROS_ERROR_STREAM("First argument (Robot index) was not privided or is invalid.");
        exit(1);
    }

    return std::stoi(argv[1]);
}

int main(int argc, char** argv) {
    uint8_t robot_index = parse_robot_index_from_arguments(argv);
    Robot robot(robot_index, argc, argv);

    ros::Rate loop_hz(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_hz.sleep();
    }

    return 0;
}
