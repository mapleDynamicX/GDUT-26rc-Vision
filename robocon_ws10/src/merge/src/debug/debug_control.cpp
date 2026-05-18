#include "./../rcekf/rcekf.h"
#include "./../serial.h"
#include "./../openvino.h"
#include "./../threadpool.h"
#include "./..//livox_ros_driver2/src/livox_ros_driver.h"
#include "./..//point_lio/src/laserMapping2.h"
#include "./../lidar.h"
#include "./../camera.h"
// #include "test.cpp"
// #include "test2.cpp"
#include "./../relocation.h"
#include "./../coordinate.h"
#include "./../recognition/camera_calibration.h"
#include "./../recognition/world_to_camera.h"
#include "./../velocity.h"
#include "./../calibration.h"
#include "./../log/logger.h"
#include <cctype>     // isdigit() 字符判断（必须包含，否则部分编译器报错）
#include <fstream>    // 文件读写
#include "./../filter.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


// 里程计消息回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    Ten::_TF_GET2_.write_data(*msg);
}



void Loopcallback()
{
    ros::NodeHandle nh("/");
    ros::Subscriber odom_sub = nh.subscribe("/fast_lio2/odom", 10, odomCallback);
    ros::Rate loop_rate(20);  
    while (Ten::_TREADPOOL_FLAG_.read_flag())  
    {
        ros::spinOnce();  
        loop_rate.sleep();  
    }
}


void calibration2()
{
    std::string log_path = std::string(ROOT_DIR) + std::string("map/map.pcd");
    //std::string log_path = std::string("/home/maple/study2/maple/map/map.pcd");
    Ten::Ten_relocation<pcl::PointXYZI> rel(log_path);
    Ten::XYZRPY xyzrpy = rel.get_transformation();

    std::cout << "---------------------------" << std::endl; 
    std::cout << "x: " << xyzrpy._xyz._x << std::endl;
    std::cout << "y: " << xyzrpy._xyz._y << std::endl;
    std::cout << "z: " << xyzrpy._xyz._z << std::endl;
    std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
    std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
    std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0.025;
    xyzrpy_error._xyz._y = -0.045;
    xyzrpy_error._xyz._z = 0.10;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;

    Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::worldtocurrent(xyzrpy._xyz, xyzrpy._rpy) * Ten::worldtocurrent(xyzrpy_error._xyz, xyzrpy_error._rpy) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());

    Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(world_origin);
}

void test_input()
{
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        int flag = 0;
        std::cin >> flag;
        if(flag == 1)
        {
            std::cout<< "flag == 1" << std::endl;
            calibration2();
        }
        else if(flag == 0)
        {
            break;
        }
    }
}

void test_lidar_point_lio()
{
    urcu_memb_register_thread();

    ros::Rate sl(1);
    Ten::XYZRPYFilter coordinate_ft;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();

        std::cout << "--------------point_lio_xyzrpy----------" << std::endl;

        std::cout << "x: " << result._xyz._x << std::endl;
        std::cout << "y: " << result._xyz._y << std::endl;
        std::cout << "z: " << result._xyz._z << std::endl;

        std::cout << "roll: " << result._rpy._roll << std::endl;
        std::cout << "pitch: " << result._rpy._pitch << std::endl;
        std::cout << "yaw: " << result._rpy._yaw << std::endl;

        std::cout << "--------------point_lio_xyzrpy----------" << std::endl;

        sl.sleep();
    }

    urcu_memb_unregister_thread();
}

/**
 * @brief 发布 nav_msgs::Odometry 消息（静态发布者，调用一次发一次）
 * @param pv 输入：自定义位姿+速度结构体 Ten::PV
 * @param stamp 输入：自定义ROS时间戳（odom_msg.header.stamp）
 */
void publishOdometryFromPVraw(const Ten::PV& pv, const ros::Time& stamp)
{
    // ====================== 静态发布者（仅初始化一次）======================
    static ros::Publisher odom_pub;
    if (!odom_pub)
    {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("/point_lio/raw", 10);
    }

    // ====================== 填充 Odometry 消息 ======================
    nav_msgs::Odometry odom_msg;

    // 1. 消息头部：使用外部传入的时间戳 ✅
    odom_msg.header.stamp = stamp; 
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 2. 位姿部分
    odom_msg.pose.pose.position.x = pv.pose._xyz._x;
    odom_msg.pose.pose.position.y = pv.pose._xyz._y;
    odom_msg.pose.pose.position.z = pv.pose._xyz._z;

    tf::Quaternion quat;
    quat.setRPY(pv.pose._rpy._roll, pv.pose._rpy._pitch, pv.pose._rpy._yaw);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(quat, q_msg);
    odom_msg.pose.pose.orientation = q_msg;

    // 3. 速度部分
    odom_msg.twist.twist.linear.x = pv.velocity._xyz._x;
    odom_msg.twist.twist.linear.y = pv.velocity._xyz._y;
    odom_msg.twist.twist.linear.z = pv.velocity._xyz._z;
    odom_msg.twist.twist.angular.x = pv.velocity._rpy._roll;
    odom_msg.twist.twist.angular.y = pv.velocity._rpy._pitch;
    odom_msg.twist.twist.angular.z = pv.velocity._rpy._yaw;

    // 4. 协方差矩阵
    odom_msg.pose.covariance.assign(0.0);
    odom_msg.twist.covariance.assign(0.0);

    // ====================== 发布消息 ======================
    odom_pub.publish(odom_msg);
}

/**
 * @brief 发布 nav_msgs::Odometry 消息（静态发布者，调用一次发一次）
 * @param pv 输入：自定义位姿+速度结构体 Ten::PV
 * @param stamp 输入：自定义ROS时间戳（odom_msg.header.stamp）
 */
void publishOdometryFromPVekf(const Ten::PV& pv, const ros::Time& stamp)
{
    // ====================== 静态发布者（仅初始化一次）======================
    static ros::Publisher odom_pub;
    if (!odom_pub)
    {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("/point_lio/ekf", 10);
    }

    // ====================== 填充 Odometry 消息 ======================
    nav_msgs::Odometry odom_msg;

    // 1. 消息头部：使用外部传入的时间戳 ✅
    odom_msg.header.stamp = stamp; 
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 2. 位姿部分
    odom_msg.pose.pose.position.x = pv.pose._xyz._x;
    odom_msg.pose.pose.position.y = pv.pose._xyz._y;
    odom_msg.pose.pose.position.z = pv.pose._xyz._z;

    tf::Quaternion quat;
    quat.setRPY(pv.pose._rpy._roll, pv.pose._rpy._pitch, pv.pose._rpy._yaw);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(quat, q_msg);
    odom_msg.pose.pose.orientation = q_msg;

    // 3. 速度部分
    odom_msg.twist.twist.linear.x = pv.velocity._xyz._x;
    odom_msg.twist.twist.linear.y = pv.velocity._xyz._y;
    odom_msg.twist.twist.linear.z = pv.velocity._xyz._z;
    odom_msg.twist.twist.angular.x = pv.velocity._rpy._roll;
    odom_msg.twist.twist.angular.y = pv.velocity._rpy._pitch;
    odom_msg.twist.twist.angular.z = pv.velocity._rpy._yaw;

    // 4. 协方差矩阵
    odom_msg.pose.covariance.assign(0.0);
    odom_msg.twist.covariance.assign(0.0);

    // ====================== 发布消息 ======================
    odom_pub.publish(odom_msg);
}



void test_lidar_ekf_of_point_lio()
{
    urcu_memb_register_thread();
    ros::Rate sl(200);
    Ten::PoseVelocityKalmanFilter ekf_fliter;
    Ten::XYZRPYFilter coordinate_ft;
    Ten::PV pose_and_velocity_now;
    double last_time = 0.0;
    double curtime = 0.0;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
        //数据是否无效
        if(result.XYZRPYisnan())
        {
            sl.sleep();
            continue;
        }
        pose_and_velocity_now.pose = result;
        //速度变化
        Ten::XYZRPY lidar_LA;
        lidar_LA._xyz._x = odo.twist.twist.linear.x;
        lidar_LA._xyz._y = odo.twist.twist.linear.y;
        lidar_LA._xyz._z = odo.twist.twist.linear.z;
        lidar_LA._rpy._roll = odo.twist.twist.angular.x;
        lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
        lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
        pose_and_velocity_now.velocity = lidar_LA;
        //ekf
        curtime = odo.header.stamp.toSec();
        double dt = curtime - last_time;
        //时间是否为负
        if(dt <= 0.0)
        {
            sl.sleep();
            continue;
        }
        Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
        last_time = curtime;
        //发布调试数据
        publishOdometryFromPVraw(pose_and_velocity_now, odo.header.stamp);
        publishOdometryFromPVekf(pose_and_velocity_ekf, odo.header.stamp);
        sl.sleep();
    }

    urcu_memb_unregister_thread();
}


void test_lidar_fast_lio()
{
    urcu_memb_register_thread();
    ros::Rate sl(1);
    Ten::XYZRPYFilter coordinate_ft;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET2_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
        std::cout << "--------------fast_lio_xyzrpy----------" << std::endl;

        std::cout << "x: " << result._xyz._x << std::endl;
        std::cout << "y: " << result._xyz._y << std::endl;
        std::cout << "z: " << result._xyz._z << std::endl;

        std::cout << "roll: " << result._rpy._roll << std::endl;
        std::cout << "pitch: " << result._rpy._pitch << std::endl;
        std::cout << "yaw: " << result._rpy._yaw << std::endl;

        std::cout << "--------------fast_lio_xyzrpy----------" << std::endl;

        sl.sleep();
    }

    urcu_memb_unregister_thread();
}



