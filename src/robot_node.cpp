#include <ros/console.h>
#include <stdint.h>
#include <string>

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "robot.h"
#include "map.h"


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
    Map map("/home/leoni/catkin_ws/src/multi_robot/occupancy_grid/base_occupancy_grid.csv");

    ros::Rate loop_hz(10);
    while (ros::ok()) {
        map.draw();
        ros::spinOnce(); // << ele simplesmente chama a função dos subscribers quando chega aqui
        //  ROS_INFO_STREAM(info12); //LOGA NO TERMINAL
        loop_hz.sleep();
    }

    return 0;
}


