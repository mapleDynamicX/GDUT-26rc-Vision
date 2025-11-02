#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <cstdbool>
#include <cstdlib>
#include <std_msgs/Bool.h>
#include <thread>
#include <mutex>
#include <rc_msgs/serial.h>

std::mutex mutex_serial;

double serial_health = 0;
ros::Time serial_begin;
// double camera_health;

// // ros::Time camera_end = ros::Time::now();
// // ros::Time serial_emd = ros::Time::now();
void boolCallback(const rc_msgs::serial::ConstPtr msg) {
    std::cout<<"boolCallback"<<std::endl;
    serial_begin = ros::Time::now();
}
// //获得相机参数
// void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
// {
//     static ros::Time camera_begin = ros::Time::now();
//     ROS_INFO("Received camera: ");
//     ros::Rate sl(0.1);
//     mutex_camera.lock();
//     camera_health = (ros::Time::now() - camera_begin).toSec();

//     if(camera_health >= 3)
//     {
//         int ret = system("/home/robocon/RC/project6_ws/src/bridgepkg/bash/camera.sh");
//         sl.sleep();
//     }
//     camera_begin = ros::Time::now();
// }
//保护节点
int main(int argc, char** argv)
{
    ros::init(argc, argv, "protected_node");
    ros::NodeHandle nh;
    ros::Subscriber serial_sub = nh.subscribe("/serial/status", 2, boolCallback);
    // ros::Subscriber camera_sub = nh.subscribe("camera/camera_info", 1, cameraInfoCallback);
    // //ros::spin();
    // ros::AsyncSpinner spinner(3);
    std::string bash_path;
    nh.param<std::string>("bash_path", bash_path, 
            "/home/rc/RC_2026/optimize_ws8/src/debug/bash/serial.sh");

    ros::Rate sl(30);
    ros::Rate stop(0.1);

    // spinner.start();
    // //int ret = system("/home/maple/study/project6_ws/src/bridgepkg/bash/xxx.sh");
    serial_begin = ros::Time::now();
    while(ros::ok())
    {
        //std::cout<<"I working now!"<<std::endl;
        serial_health = (ros::Time::now() - serial_begin).toSec();
        std::cout<<"serial_health"<<serial_health<<std::endl;
        if(serial_health >= 1)
        {
            int ret = system(bash_path.c_str());
            std::cout<<"restart serial"<<std::endl;
            stop.sleep();
            serial_begin = ros::Time::now();
        }
        sl.sleep();
        ros::spinOnce();
    }
    // ros::waitForShutdown();
    return 0;
}




