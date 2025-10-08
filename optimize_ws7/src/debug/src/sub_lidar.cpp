//44 28
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovariance.h> 
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <cstring>  // 或者 #include <string.h>
#include <iostream>

#include <chrono> 

void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    for (const auto& transform : msg->transforms)
    {
        if(transform.child_frame_id != "body")
        {
            continue;
        }
        // 获取四元数
        geometry_msgs::Quaternion quat = transform.transform.rotation;

        // 使用 tf2 库将四元数转换为欧拉角
        tf2::Quaternion tf_quat;
        tf_quat.setX(quat.x);
        tf_quat.setY(quat.y);
        tf_quat.setZ(quat.z);
        tf_quat.setW(quat.w);

        // 转换为旋转矩阵，然后提取欧拉角
        tf2::Matrix3x3 mat(tf_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);  // 得到的是弧度值
        
        double yaw_deg = yaw * 180.0 / M_PI;
        float _x  = transform.transform.translation.x - 0.28*cos(yaw) + 0.28;
        float _y = transform.transform.translation.y - 0.28*sin(yaw);

        std::cout << "base x = "<< _x << std::endl;
        std::cout << "base y = "<< _y << std::endl;
        std::cout << "base yaw = "<< yaw << std::endl;

    }
}







int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "sub_lidar_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    ros::Subscriber tf_sub = nh.subscribe("/tf", 10, tfCallback);

    // 设置循环频率
    ros::Rate loop_rate(30); // 10Hz
 
    ros::spin();

    return 0;
}






















