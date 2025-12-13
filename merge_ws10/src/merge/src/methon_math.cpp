#ifndef __METHOD_MATH_H_
#define __METHOD_MATH_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// 引入tf库，用于四元数转欧拉角
#include <tf/transform_datatypes.h>
// 引入数学库，用于弧度转角度（可选）
#include <cmath>
#include <eigen3/Eigen/Core>       // 核心矩阵/向量
#include <eigen3/Eigen/Geometry>   // 几何变换（旋转、平移）
#include <opencv2/opencv.hpp>

namespace Ten
{

struct XYZ
{
double _x = 0.0;
double _y = 0.0;
double _z = 0.0;
};

struct RPY
{
double _roll = 0.0;
double _pitch = 0.0;
double _yaw = 0.0;
};

struct XYZRPY
{
XYZ _xyz;
RPY _rpy;
};

/**
    @brief 里程计消息转xyzrpy
    @param msg: nav_msgs::Odometry
    @return xyzrpy
*/
XYZRPY Nav_Odometrytoxyzrpy(nav_msgs::Odometry msg)
{
    XYZRPY change;

    // 1. 获取位姿基础数据（位置 + 四元数）
    change._xyz._x = msg.pose.pose.position.x;
    change._xyz._y = msg.pose.pose.position.y;
    change._xyz._z = msg.pose.pose.position.z;

    double ori_x = msg.pose.pose.orientation.x;
    double ori_y = msg.pose.pose.orientation.y;
    double ori_z = msg.pose.pose.orientation.z;
    double ori_w = msg.pose.pose.orientation.w;

    // 2. 四元数转欧拉角（Roll/Pitch/Yaw，单位：弧度）
    // 步骤1：构造tf四元数对象
    tf::Quaternion quat(ori_x, ori_y, ori_z, ori_w);
    // 步骤2：定义存储欧拉角的变量（roll:绕X轴, pitch:绕Y轴, yaw:绕Z轴）
    double roll, pitch, yaw;
    // 步骤3：转换为欧拉角 弧度
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    change._rpy._roll = roll;
    change._rpy._pitch = pitch;
    change._rpy._yaw = yaw;
    return change;
}

/**
    @brief 创建旋转矩阵
    @param rx: roll (弧度)
    @param ry: pitch (弧度)
    @param rz: yaw (弧度)
    @return Eigen::Matrix3f: 3x3的旋转矩阵
*/
Eigen::Matrix3f createRotationMatrix(float rx, float ry, float rz) {
    // 弧度
    // 创建绕各轴的旋转矩阵
    Eigen::Matrix3f R_x;
    R_x << 1, 0, 0,
           0, cos(rx), -sin(rx),
           0, sin(rx), cos(rx);
    Eigen::Matrix3f R_y;
    R_y << cos(ry), 0, sin(ry),
           0, 1, 0,
           -sin(ry), 0, cos(ry);
    Eigen::Matrix3f R_z;
    R_z << cos(rz), -sin(rz), 0,
           sin(rz), cos(rz), 0,
           0, 0, 1;
    // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
    return R_z * R_y * R_x;
}
/**
    @brief 创建旋转矩阵
    @param tx: x (米)
    @param ty: y (米)
    @param tz: z (米)
    @return Eigen::Vector3f: 1x3平移矩阵
*/
Eigen::Vector3f createTranslationVector(float tx, float ty, float tz) {
    Eigen::Vector3f translation(tx, ty, tz);
    return translation;
}
/**
    @brief 创建旋转矩阵
    @param rotation: 旋转矩阵
    @param translation: 平移矩阵
    @return Eigen::Matrix4f: 4x4的RT矩阵
*/
//组合现有旋转矩阵与平移向量
Eigen::Matrix4f combineRotationAndTranslation(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}

/**
    @brief 世界到当前
    @param xyz：当前点在世界坐标系的xyz坐标
    @param rpy: 当前点相对于世界坐标系的旋转和平移
*/
Eigen::Matrix4f worldtocurrent(XYZ xyz, RPY rpy)
{
    Eigen::Matrix3f rot = createRotationMatrix(-rpy._roll, -rpy._pitch, -rpy._yaw);
    Eigen::Vector3f tra = -(rot*createTranslationVector(xyz._x, xyz._y, xyz._z));
    Eigen::Matrix4f T = combineRotationAndTranslation(rot, tra);
    return T;
}

/**
    @brief 当前到世界
    @param xyz：当前点在世界坐标系的xyz坐标
    @param rpy: 当前点相对于世界坐标系的旋转和平移
*/
Eigen::Matrix4f currenttoworld(XYZ xyz, RPY rpy)
{
    Eigen::Matrix4f T = worldtocurrent(xyz, rpy);
    return T.inverse();
}

/**
    @brief 把Eigen::Matrix3f转cv::Mat rvec
    @param R: 旋转矩阵
    @return cv::Mat：rvec
*/
cv::Mat RotationMatrixtorvec(Eigen::Matrix3f R)
{
    cv::Mat rot_matrix = (cv::Mat_<double>(3, 3) <<
    R(0,0), R(0,1), R(0,2),
    R(1,0), R(1,1), R(1,2),
    R(2,0), R(2,1), R(2,2));
    cv::Mat rvec;
    cv::Rodrigues(rot_matrix, rvec);
    return rvec;
}

/**
    @brief 把cv::Mat rvec转 Eigen::Matrix3f
    @param rvec: 旋转向量 cv::Mat
    @return Eigen::Matrix3f：R
*/
Eigen::Matrix3f RotationMatrixtorvec(cv::Mat rvec)
{
    if(rvec.rows != 3 && rvec.cols != 1)
    {
        std::cout<< "error! Eigen::Matrix3f RotationMatrixtorvec(cv::Mat rvec)" << std::endl;
        return Eigen::Matrix3f();
    }
    cv::Mat rot_matrix;
    cv::Rodrigues(rvec, rot_matrix);
    Eigen::Matrix3f R;
    for(int i = 0; i < 3; i ++)
    {
        for(int j = 0; j < 3; j++)
        {
            R(i,j) = rot_matrix.at<double>(i, j);
        }
    }
    return R;
}



}







#endif

