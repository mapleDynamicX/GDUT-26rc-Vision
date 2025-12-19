
#ifndef __TESTT_H_
#define __TESTT_H_


#include "test.cpp"
#include "methon_math.cpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <image_transport/image_transport.h>
#include <cmath>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <cstring>  
#include <iostream>
#include <thread>
#include <stdexcept>
#include <bitset>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>

#define _L_ 1.2
#define _H_ 0.2
#define _ly1_ 0.425
#define _ly2_ 0.775
#define _lx1_ 0.425
#define _lh_ 0.35 
#define _X_   2.58     //2.9            //2.17
#define _Y_  -0.205             //0.2  //-0.24





struct G
{
    G()
    {
        // 2. 相机内参矩阵 K
        _K = (cv::Mat_<double>(3,3) <<
            1380.4350, 0, 974.0183,
            0,  1385.0788, 541.4301,
            0, 0, 1);
        // _K = (cv::Mat_<double>(3,3) <<
        //     387.2624, 0, 318.6802,
        //     0,  387.2624, 247.4131,
        //     0, 0, 1);
        // 3. 畸变系数（假设零畸变）
        _distCoeffs = cv::Mat::zeros(5, 1, CV_64F);


        
        _rvec = (cv::Mat_<double>(3,1) << 1.2126, -1.2788, 1.2281); // 旋转（弧度）
        _tvec = (cv::Mat_<double>(3,1) << 0.0214, 0.3877, 0.4997);   // 平移（米）

        // 1. 定义3D点（世界坐标）
        //std::vector<cv::Point3f> objectPoints;
        //objectPoints.push_back(cv::Point3f(0, 0, 0));   // 原点
        for(int j = 0; j < 4; j++)
        {
            for(int i = 0; i < 3; i++)
            {
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, -_Y_ - i*_L_ - _ly1_, _arr[j*3+i]+_lh_));
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, -_Y_ - i*_L_ - _ly2_, _arr[j*3+i]+_lh_));
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, -_Y_ - i*_L_ - _ly2_, _arr[j*3+i]));
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, -_Y_ - i*_L_ - _ly1_, _arr[j*3+i]));

                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly1_, _arr[j*3+i]+_lh_));
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly2_, _arr[j*3+i]+_lh_));
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly2_, _arr[j*3+i]));
                // _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly1_, _arr[j*3+i]));

                _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly1_, _arr[j*3+i]+_lh_));
                _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly2_, _arr[j*3+i]+_lh_));
                _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly2_, _arr[j*3+i]));
                _objectPoints.push_back(cv::Point3f(_X_ + j*_L_ + _lx1_, _Y_ + i*_L_ + _ly1_, _arr[j*3+i]));
            }
        }

        for(int i = 0; i < 48; i++)
        {
            //_c_objectPoints.push_back(cv::Point3f(-_objectPoints[i].y, 1.3 - _objectPoints[i].z, _objectPoints[i].x));
            _c_objectPoints.push_back(cv::Point3f(_objectPoints[i].x, _objectPoints[i].y, _objectPoints[i].z - 0.717));
        }
        for(int i = 0; i < 48; i++)
        {
            //_c_objectPoints.push_back(cv::Point3f(-_objectPoints[i].y, 1.3 - _objectPoints[i].z, _objectPoints[i].x));
            _sc_objectPoints.push_back(cv::Point3f(_objectPoints[i].x, _objectPoints[i].y, _objectPoints[i].z - 0.717));
        }

    }
    std::vector<cv::Point3f> _objectPoints;
    std::vector<cv::Point3f> _c_objectPoints;
    std::vector<cv::Point3f> _sc_objectPoints;
    cv::Mat _K;
    cv::Mat _distCoeffs;
    cv::Mat _rvec;
    cv::Mat _tvec;
    cv::Mat _image;
    std::mutex _mtx_image;
    std::mutex _mtx_tf;
    float _arr[12] = {0.4,0.2,0.4,0.6,0.4,0.2,0.4,0.6,0.4,0.2,0.4,0.2};
    int _mask[12] = {0};


    //loss
    

}global;



Eigen::Matrix3d createRotationMatrix(float rx, float ry, float rz) {
    // 转换为弧度
    // rx = rx * M_PI / 180.0f; // Roll (绕X轴)
    // ry = ry * M_PI / 180.0f; // Pitch (绕Y轴)
    // rz = rz * M_PI / 180.0f; // Yaw (绕Z轴)
    // 创建绕各轴的旋转矩阵
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
           0, cos(rx), -sin(rx),
           0, sin(rx), cos(rx);
    Eigen::Matrix3d R_y;
    R_y << cos(ry), 0, sin(ry),
           0, 1, 0,
           -sin(ry), 0, cos(ry);
    Eigen::Matrix3d R_z;
    R_z << cos(rz), -sin(rz), 0,
           sin(rz), cos(rz), 0,
           0, 0, 1;
    // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
    return R_z * R_y * R_x;
}

Eigen::Vector3d createTranslationVector(float tx, float ty, float tz) {
    Eigen::Vector3d translation(tx, ty, tz);
    return translation;
}

// 分离和组合现有旋转矩阵与平移向量
Eigen::Matrix4d combineRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}

cv::Point3f transformPoint(const cv::Point3f& p, const cv::Mat& R, const cv::Point3f& t) {
    // 检查旋转矩阵是否为3x3的float类型
    CV_Assert(R.rows == 3 && R.cols == 3 && R.type() == CV_32F);
    
    // 计算旋转后的坐标
    float x_rot = R.at<float>(0, 0) * p.x + R.at<float>(0, 1) * p.y + R.at<float>(0, 2) * p.z;
    float y_rot = R.at<float>(1, 0) * p.x + R.at<float>(1, 1) * p.y + R.at<float>(1, 2) * p.z;
    float z_rot = R.at<float>(2, 0) * p.x + R.at<float>(2, 1) * p.y + R.at<float>(2, 2) * p.z;
    
    // 叠加平移向量
    return cv::Point3f(x_rot + t.x, y_rot + t.y, z_rot + t.z);
}



void imageCallback(const nav_msgs::Odometry msg, cv::Mat& image_)
{

    tf2::Quaternion tf_quat(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);  
    double x =  msg.pose.pose.position.x + 0.0037;
    double y =  msg.pose.pose.position.y + 0.0012;
    double z =  msg.pose.pose.position.z + 0.0026;

    Ten::XYZ xyz;
    xyz._x = x;
    xyz._y = y;
    //xyz._z = z - 0.049303;
    xyz._z = z;
    Ten::RPY rpy;
    rpy._roll = roll + 0.007;
    rpy._pitch = pitch - 0.01;
    rpy._yaw = yaw - 0.0003;
    std::cout<< "-----------------------------------------" << std::endl;
    std::cout<<"x: "<<x<<" y: "<<y<<" z: "<< z <<std::endl;
    std::cout<<"roll: "<<rpy._roll<<" pitch: "<<rpy._pitch<<" yaw: "<< rpy._yaw <<std::endl;
    std::cout<<"-----------------------------------------" << std::endl;

    Eigen::Matrix3d rot = createRotationMatrix(-rpy._roll, -rpy._pitch, -rpy._yaw);
    Eigen::Vector3d tra = -(rot*createTranslationVector(x, y, z));
    cv::Mat R = (cv::Mat_<float>(3, 3) << 
        rot(0,0), rot(0,1), rot(0,2),
        rot(1,0),  rot(1,1), rot(1,2),
        rot(2,0),   rot(2,1),  rot(2,2)
    );
    cv::Point3f t(tra[0], tra[1], tra[2]);
    for(int i = 0; i < global._c_objectPoints.size(); i++)
    {
        cv::Point3f p_transformed = transformPoint(global._c_objectPoints[i], R, t);
        global._sc_objectPoints[i] = p_transformed;
        //std::cout<<i << " " <<p_transformed.x << " " << p_transformed.y << " " <<p_transformed.z << std::endl;
    }

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(global._sc_objectPoints, global._rvec, global._tvec, global._K, global._distCoeffs, imagePoints);
    std::cout<<"imagePoints.size()"<<imagePoints.size()<<std::endl;
    
    for (int i = 0; i < imagePoints.size(); i += 4) 
    {
        if(global._mask[i / 4] == 1)
        {
            std::cout<<"global._mask[i]:" <<i<<std::endl;
            continue;
        }
        cv::line(image_, cv::Point(cvRound(imagePoints[i].x), cvRound(imagePoints[i].y)),cv::Point(cvRound(imagePoints[i+1].x), cvRound(imagePoints[i+1].y)),cv::Scalar(0, 0, 255), 2);
        cv::line(image_, cv::Point(cvRound(imagePoints[i+1].x), cvRound(imagePoints[i+1].y)),cv::Point(cvRound(imagePoints[i+2].x), cvRound(imagePoints[i+2].y)),cv::Scalar(0, 0, 255), 2);
        cv::line(image_, cv::Point(cvRound(imagePoints[i+2].x), cvRound(imagePoints[i+2].y)),cv::Point(cvRound(imagePoints[i+3].x), cvRound(imagePoints[i+3].y)),cv::Scalar(0, 0, 255), 2);
        cv::line(image_, cv::Point(cvRound(imagePoints[i+3].x), cvRound(imagePoints[i+3].y)),cv::Point(cvRound(imagePoints[i].x), cvRound(imagePoints[i].y)),cv::Scalar(0, 0, 255), 2);
    } 
}




// int test_speed()
// {

//     ros::NodeHandle nh;
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise("pub_image_topic", 2);
//     ros::Rate rate(10);
//     while(ros::ok())
//     {
//         sensor_msgs::ImagePtr msg;
//         {
//             std::lock_guard<std::mutex> lock(global._mtx_image);
//             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", global._image).toImageMsg();
//         }
//         pub.publish(msg);
//         std::cout << "publish success" << std::endl;
//         rate.sleep();
//     }
//     return 0;
// }



int test_speed()
{
    urcu_memb_register_thread();
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("pub_image_topic", 1);
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    ros::Rate rate(10);
    std::queue<cv::Mat> q;

    cv::namedWindow("img", cv::WINDOW_NORMAL);
    while(ros::ok())
    {
        // Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(odo);
        // std::cout<< "xyz: "<<tf._xyz._x << " " << tf._xyz._y << " " << tf._xyz._z << std::endl;
        // std::cout<< "rpy: "<<tf._rpy._roll << " " << tf._rpy._pitch << " " << tf._rpy._yaw << std::endl;

        cv::Mat img = camera.camera_read();
        rate.sleep();
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();


        sensor_msgs::ImagePtr msg;
        imageCallback(odo, img);

        cv::imshow("img", img);
        cv::waitKey(23);
        // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", last).toImageMsg();
        // pub.publish(msg);
        //std::cout << "publish success" << std::endl;
        
        
    }

    camera.~Ten_camera();

    urcu_memb_unregister_thread();
    return 0;
}








#endif




