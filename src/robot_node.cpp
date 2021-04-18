#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

void laser_clbk(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> laser_measurements = scan->ranges;

    //ROS_INFO_STREAM("90 graus:" << laser_measurements[90]);
    //ROS_INFO_STREAM("180 graus:" << laser_measurements[180]);
    //ROS_INFO_STREAM("0 graus:" << laser_measurements[0]);

    //Aqui vai o código do filto de partículas
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_filter_1_n");
    ros::NodeHandle node_hanle;
    ROS_INFO_STREAM("argsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargs:" << argv);
    ROS_INFO_STREAM("argsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargs:" << argv);
    ROS_INFO_STREAM("argsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargs:" << argv);
    ROS_INFO_STREAM("argsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargs:" << argv);
    ROS_INFO_STREAM("argsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargsargs:" << argv);
    //Subscriber que lê medidas dos sensores
    ros::Subscriber laser_robot1 = node_hanle.subscribe<sensor_msgs::LaserScan>("/robot1/scan", 10, laser_clbk);
    ros::spin();

    //Publisher que envia informações para outro robô
    ros::Publisher my_pub = node_hanle.advertise<std_msgs::String>("info_exchange12_t", 10);
    std_msgs::String info12;
    info12.data = "Oi robô 2, eu sou o robô 1";
    ros::Rate loop_hz(10);

    while (ros::ok()) {
        my_pub.publish(info12);  //PUBLICA NO TÓPICO info_exchange12_t
                                 //  ROS_INFO_STREAM(info12); //LOGA NO TERMINAL
        loop_hz.sleep();
    }

    return 0;
}
