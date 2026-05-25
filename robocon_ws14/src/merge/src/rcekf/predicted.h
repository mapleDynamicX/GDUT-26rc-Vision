#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <cmath>
#include <vector>
#include "./../method_math.h"

//  ==================== 说明 ====================
//  XYZ / RPY / XYZRPY / isDoubleEqual 来自 method_math.h，直接复用
//  核心功能：缓存最新N帧IMU → 传入初始速度 → 角速度+线加速度死区滤波 → 独立积分窗口数据 → 输出【小段相对位姿】
// ==============================================

namespace Ten
{

class ImuOdometry
{
public:
    /**
     * @brief 构造函数
     * @param max_queue 最大缓存IMU帧数（默认20）
     * @param gravity 世界坐标系重力向量
     * @param ang_deadzone 角速度死区阈值 (rad/s)
     * @param acc_deadzone 线加速度死区阈值 (m/s²) 【新增】
     */
    ImuOdometry(size_t max_queue = 20, 
                const XYZ& gravity = XYZ{0.0, 0.0, -0.982047},
                double ang_deadzone = 0.02,
                double acc_deadzone = 0.01)  // 新增线加速度死区默认值
        : max_queue_size_(max_queue),
          gravity_world_(gravity),
          angular_vel_deadzone_(ang_deadzone),
          linear_acc_deadzone_(acc_deadzone), // 初始化加速度死区
          is_initialized_(false)
    {
        initial_rot_.assign(3, std::vector<double>(3, 0.0));
    }

    // ==================== 核心接口 ====================
    /**
     * @brief 输入IMU数据 + 初始速度 → 缓存最新N帧 → 计算【小段相对位姿】
     * @param imu_msg ROS标准IMU消息
     * @param initial_vel 小段积分的初始速度
     * @return XYZRPY 最近窗口内的相对位置+相对姿态
     */
    XYZRPY processImu(const sensor_msgs::Imu& imu_msg, const XYZ& initial_vel)
    {
        // 1. 缓存数据：仅保留最新N帧
        cacheImuData(imu_msg);

        // 2. 第一帧：初始化坐标系矫正（Z轴向上，仅执行一次）
        if (!is_initialized_)
        {
            initialAlignment(imu_msg);
            return XYZRPY{};
        }

        // 3. 独立积分当前缓存的所有帧
        return integrateWindow(initial_vel);
    }

    // 重置队列
    void resetQueue()
    {
        std::queue<sensor_msgs::Imu> empty_q;
        std::swap(imu_queue_, empty_q);
    }

    // 设置最大缓存帧数
    void setMaxQueueSize(size_t size) { max_queue_size_ = size; }

    // 设置世界坐标系重力
    void setWorldGravity(const XYZ& gravity) { gravity_world_ = gravity; }

    // 设置角速度死区阈值
    void setAngularVelDeadzone(double threshold)
    {
        angular_vel_deadzone_ = std::fabs(threshold);
    }

    // ==================== 新增：设置线加速度死区阈值 ====================
    void setLinearAccDeadzone(double threshold)
    {
        linear_acc_deadzone_ = std::fabs(threshold);
    }

private:
    // 缓存IMU数据
    void cacheImuData(const sensor_msgs::Imu& imu_msg)
    {
        imu_queue_.push(imu_msg);
        while (imu_queue_.size() > max_queue_size_)
        {
            imu_queue_.pop();
        }
    }

    // 初始对准：矫正坐标系Z轴向上
    void initialAlignment(const sensor_msgs::Imu& imu_msg)
    {
        const double ax = imu_msg.linear_acceleration.x;
        const double ay = imu_msg.linear_acceleration.y;
        const double az = imu_msg.linear_acceleration.z;
        const double norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1.0) return;

        const double init_roll  = std::atan2(ay, az);
        const double init_pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        const double init_yaw   = 0.0;

        computeRotationMatrix(init_roll, init_pitch, init_yaw, initial_rot_);
        is_initialized_ = true;
    }

    /**
     * @brief 核心积分（角速度死区 + 线加速度死区 + 初始速度）
     */
    XYZRPY integrateWindow(const XYZ& initial_vel)
    {
        XYZ rel_pos;
        RPY rel_rpy;
        XYZ rel_vel = initial_vel;
        double last_ts = 0.0;

        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        while (!temp_q.empty())
        {
            const auto imu = temp_q.front();
            temp_q.pop();

            const double curr_ts = imu.header.stamp.toSec();
            if (last_ts == 0.0) {
                last_ts = curr_ts;
                continue;
            }

            const double dt = curr_ts - last_ts;

            // Step1：IMU数据旋转到世界坐标系
            XYZ angular_vel, linear_acc;
            transformImuToWorld(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, angular_vel);
            transformImuToWorld(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, linear_acc);

            // 角速度死区处理
            if (std::fabs(angular_vel._x) < angular_vel_deadzone_) angular_vel._x = 0.0;
            if (std::fabs(angular_vel._y) < angular_vel_deadzone_) angular_vel._y = 0.0;
            if (std::fabs(angular_vel._z) < angular_vel_deadzone_) angular_vel._z = 0.0;

            

            // Step2：相对姿态积分
            rel_rpy._roll  += angular_vel._x * dt;
            rel_rpy._pitch += angular_vel._y * dt;
            rel_rpy._yaw   += angular_vel._z * dt;

            // Step3：去除重力
            linear_acc._x -= gravity_world_._x;
            linear_acc._y -= gravity_world_._y;
            linear_acc._z -= gravity_world_._z;

            // ==================== 新增：线加速度死区处理 ====================
            if (std::fabs(linear_acc._x) < linear_acc_deadzone_) linear_acc._x = 0.0;
            if (std::fabs(linear_acc._y) < linear_acc_deadzone_) linear_acc._y = 0.0;
            if (std::fabs(linear_acc._z) < linear_acc_deadzone_) linear_acc._z = 0.0;

            // Step4：速度积分
            rel_vel._x += linear_acc._x * dt;
            rel_vel._y += linear_acc._y * dt;
            rel_vel._z += linear_acc._z * dt;

            // Step5：位置积分
            rel_pos._x += rel_vel._x * dt;
            rel_pos._y += rel_vel._y * dt;
            rel_pos._z += rel_vel._z * dt;

            last_ts = curr_ts;
        }

        XYZRPY relative_pose;
        relative_pose._xyz = rel_pos;
        relative_pose._rpy = rel_rpy;
        return relative_pose;
    }

    // 旋转矩阵计算
    void computeRotationMatrix(double roll, double pitch, double yaw, std::vector<std::vector<double>>& rot)
    {
        const double cr = cos(roll), sr = sin(roll);
        const double cp = cos(pitch), sp = sin(pitch);
        const double cy = cos(yaw), sy = sin(yaw);

        rot[0][0] = cp*cy;   rot[0][1] = sr*sp*cy - cr*sy; rot[0][2] = cr*sp*cy + sr*sy;
        rot[1][0] = cp*sy;   rot[1][1] = sr*sp*sy + cr*cy; rot[1][2] = cr*sp*sy - sr*cy;
        rot[2][0] = -sp;     rot[2][1] = sr*cp;            rot[2][2] = cr*cp;
    }

    // 坐标变换
    void transformImuToWorld(double x, double y, double z, XYZ& out)
    {
        out._x = initial_rot_[0][0]*x + initial_rot_[0][1]*y + initial_rot_[0][2]*z;
        out._y = initial_rot_[1][0]*x + initial_rot_[1][1]*y + initial_rot_[1][2]*z;
        out._z = initial_rot_[2][0]*x + initial_rot_[2][1]*y + initial_rot_[2][2]*z;
    }

private:
    // 队列配置
    std::queue<sensor_msgs::Imu> imu_queue_;
    size_t max_queue_size_;

    // 坐标系参数
    XYZ gravity_world_;
    bool is_initialized_;
    std::vector<std::vector<double>> initial_rot_;

    // 死区参数
    double angular_vel_deadzone_;  // 角速度死区
    double linear_acc_deadzone_;   // 【新增】线加速度死区
};

}
