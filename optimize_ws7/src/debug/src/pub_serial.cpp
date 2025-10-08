#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <cstring>  // 或者 #include <string.h>
#include <iostream>
#include <thread>
#include "rc_msgs/serial.h"




int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_serial_node");
    ros::NodeHandle nh;
    ros::Publisher status_pub = nh.advertise<rc_msgs::serial>("/serial/status", 10);
    ros::Rate rate(100);
    while(ros::ok())
    {
        rc_msgs::serial serial_pub;
        serial_pub.stamp = ros::Time::now();
        serial_pub.map.push_back(2);
        status_pub.publish(serial_pub);
        std::cout<<"ok"<<std::endl;
        rate.sleep();
    }
    return 0;
}