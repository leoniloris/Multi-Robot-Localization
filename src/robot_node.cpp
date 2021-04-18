#include <ros/console.h>
#include <stdint.h>
#include <string>

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "robot.h"



void laser_clbk(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> laser_measurements = scan->ranges;

    ROS_INFO_STREAM("90 graus:" << laser_measurements[90]);
    ROS_INFO_STREAM("180 graus:" << laser_measurements[180]);
    ROS_INFO_STREAM("0 graus:" << laser_measurements[0]);

    //Aqui vai o código do filto de partículas
}

static std::string parse_robot_index_from_arguments(char** argv) {
    if(argv[1] == NULL){
        ROS_ERROR_STREAM("First argument (Robot index) was not privided or is invalid.");
        exit(1);
    }

    return std::string(argv[1]);
}

int main(int argc, char** argv) {
    std::string robot_index = parse_robot_index_from_arguments(argv);
    // ros::init(argc, argv, "robot_node" + robot_index);
    // ros::NodeHandle node_hanle;
    // ros::Subscriber laser_scan = node_hanle.subscribe<sensor_msgs::LaserScan>("/ugv" + robot_index + "/scan", MESSAGE_QUEUE_SIZE, laser_clbk);
    // ros::Publisher broadcaster = node_hanle.advertise<std_msgs::String>("broadcast", MESSAGE_QUEUE_SIZE);
    Robot robot(robot_index, argc, argv);

    ros::spin();

    std_msgs::String info12;
    info12.data = "Oi robô 2, eu sou o robô 1";

    // ros::Rate loop_hz(10);
    // while (ros::ok()) {
    //     broadcaster.publish(info12);  // PUBLICA NO TÓPICO broadcast
    //                                   // ROS_INFO_STREAM(info12); //LOGA NO TERMINAL
    //     loop_hz.sleep();
    //     ROS_ERROR_STREAM("baby");

    // }

    return 0;
}
