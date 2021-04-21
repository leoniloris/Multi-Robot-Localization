#include <ros/console.h>
#include <stdint.h>
#include <string>

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "robot.h"


static std::string parse_robot_index_from_arguments(char** argv) {
    if(argv[1] == NULL){
        ROS_ERROR_STREAM("First argument (Robot index) was not privided or is invalid.");
        exit(1);
    }

    return std::string(argv[1]);
}

int main(int argc, char** argv) {
    std::string robot_index = parse_robot_index_from_arguments(argv);
    Robot robot(robot_index, argc, argv);

    ros::Rate loop_hz(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_hz.sleep();
    }

    return 0;
}


