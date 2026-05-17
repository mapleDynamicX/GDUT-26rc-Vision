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
    float arr[9] = {0};

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0;
    xyzrpy_error._xyz._y = 0;
    xyzrpy_error._xyz._z = 0;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;
    //Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    Ten::XYZRPY xyzrpy_car;
    xyzrpy_car._xyz._x = -0.40944;            //-0.40944
    xyzrpy_car._xyz._y = 0.40944 + 0.088 / 2; // 0.40944
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = -M_PI / 2.0;
    //Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car);

    //Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
    // nav_msgs::Odometry odo_n;
    ros::Rate sl(1);
    Ten::XYZRPYFilter coordinate_ft;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
        //Ten::XYZRPY result = coordinate_ft(result_last);
        // 速度变化
        // Ten::XYZRPY lidar_LA;
        // lidar_LA._xyz._x = odo.twist.twist.linear.x;
        // lidar_LA._xyz._y = odo.twist.twist.linear.y;
        // lidar_LA._xyz._z = odo.twist.twist.linear.z;
        // lidar_LA._rpy._roll = odo.twist.twist.angular.x;
        // lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
        // lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
        // Ten::_VELOCITY_TRANSFORMATION_.set_lidar(lidar_LA);
        // Ten::XYZRPY car_LA = Ten::_VELOCITY_TRANSFORMATION_.getvelocity();

        // float roll = result._rpy._roll;
        // float pitch = result._rpy._pitch;
        // float yaw = result._rpy._yaw;

        // arr[0] = result._xyz._x;
        // arr[1] = result._xyz._y;
        // arr[2] = result._xyz._z;

        std::cout << "--------------point_lio_xyzrpy----------" << std::endl;

        std::cout << "x: " << result._xyz._x << std::endl;
        std::cout << "y: " << result._xyz._y << std::endl;
        std::cout << "z: " << result._xyz._z << std::endl;

        std::cout << "roll: " << result._rpy._roll << std::endl;
        std::cout << "pitch: " << result._rpy._pitch << std::endl;
        std::cout << "yaw: " << result._rpy._yaw << std::endl;

        std::cout << "--------------point_lio_xyzrpy----------" << std::endl;
        //std::cout << result._xyz._x << " " << result._xyz._y << " " << result._xyz._z << std::endl;

        // arr[3] = roll * 180.0 / M_PI;
        // arr[4] = pitch * 180.0 / M_PI;
        // arr[5] = yaw * 180.0 / M_PI;

        // arr[6] = car_LA._xyz._x;
        // arr[7] = car_LA._xyz._y;
        // arr[8] = car_LA._xyz._z;



        // std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;

        // odo_n.twist.twist.linear.x = result._xyz._x;
        // odo_n.twist.twist.linear.y = result._xyz._y;
        // odo_n.twist.twist.linear.z = result._xyz._z;

        // odo_n.twist.twist.linear.x = car_LA._xyz._x;
        // odo_n.twist.twist.linear.y = car_LA._xyz._y;
        // odo_n.twist.twist.linear.z = car_LA._xyz._z;

        // Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);

        sl.sleep();
    }

    urcu_memb_unregister_thread();
}

void test_lidar_fast_lio()
{
    urcu_memb_register_thread();
    float arr[9] = {0};

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0;
    xyzrpy_error._xyz._y = 0;
    xyzrpy_error._xyz._z = 0;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;
    //Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    Ten::XYZRPY xyzrpy_car;
    xyzrpy_car._xyz._x = -0.40944;            //-0.40944
    xyzrpy_car._xyz._y = 0.40944 + 0.088 / 2; // 0.40944
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = -M_PI / 2.0;
    //Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car);

    //Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
    // nav_msgs::Odometry odo_n;
    ros::Rate sl(1);
    Ten::XYZRPYFilter coordinate_ft;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET2_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
        //Ten::XYZRPY result = coordinate_ft(result_last);
        // 速度变化
        // Ten::XYZRPY lidar_LA;
        // lidar_LA._xyz._x = odo.twist.twist.linear.x;
        // lidar_LA._xyz._y = odo.twist.twist.linear.y;
        // lidar_LA._xyz._z = odo.twist.twist.linear.z;
        // lidar_LA._rpy._roll = odo.twist.twist.angular.x;
        // lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
        // lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
        // Ten::_VELOCITY_TRANSFORMATION_.set_lidar(lidar_LA);
        // Ten::XYZRPY car_LA = Ten::_VELOCITY_TRANSFORMATION_.getvelocity();

        // float roll = result._rpy._roll;
        // float pitch = result._rpy._pitch;
        // float yaw = result._rpy._yaw;

        // arr[0] = result._xyz._x;
        // arr[1] = result._xyz._y;
        // arr[2] = result._xyz._z;

        std::cout << "--------------fast_lio_xyzrpy----------" << std::endl;

        std::cout << "x: " << result._xyz._x << std::endl;
        std::cout << "y: " << result._xyz._y << std::endl;
        std::cout << "z: " << result._xyz._z << std::endl;

        std::cout << "roll: " << result._rpy._roll << std::endl;
        std::cout << "pitch: " << result._rpy._pitch << std::endl;
        std::cout << "yaw: " << result._rpy._yaw << std::endl;

        std::cout << "--------------fast_lio_xyzrpy----------" << std::endl;
        //std::cout << result._xyz._x << " " << result._xyz._y << " " << result._xyz._z << std::endl;

        // arr[3] = roll * 180.0 / M_PI;
        // arr[4] = pitch * 180.0 / M_PI;
        // arr[5] = yaw * 180.0 / M_PI;

        // arr[6] = car_LA._xyz._x;
        // arr[7] = car_LA._xyz._y;
        // arr[8] = car_LA._xyz._z;



        // std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;

        // odo_n.twist.twist.linear.x = result._xyz._x;
        // odo_n.twist.twist.linear.y = result._xyz._y;
        // odo_n.twist.twist.linear.z = result._xyz._z;

        // odo_n.twist.twist.linear.x = car_LA._xyz._x;
        // odo_n.twist.twist.linear.y = car_LA._xyz._y;
        // odo_n.twist.twist.linear.z = car_LA._xyz._z;

        // Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);

        sl.sleep();
    }

    urcu_memb_unregister_thread();
}



