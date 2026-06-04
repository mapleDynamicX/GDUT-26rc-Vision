#ifndef __PREDICTED_H_
#define __PREDICTED_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <cmath>
#include <vector>
#include <algorithm>
#include "./../method_math.h"

namespace Ten {
// 全局常量：单位转换系数（g → m/s²）
constexpr double G_TO_MPS2 = 9.81;
// 自适应阈值：加速度变化率超过此值判定为剧烈运动
constexpr double DYNAMIC_THRESHOLD = 5.0; // m/s³
// 最大平滑系数：慢运动时的最大平滑程度
constexpr double MAX_SMOOTH_ALPHA = 0.3;
// 最小阻尼系数：剧烈运动时的最小阻尼
constexpr double MIN_DAMPING = 0.99;
// 速度翻转检测：最小速度差值（确认是真实翻转）
constexpr double FLIP_VEL_DIFF_THRESHOLD = 0.03; // m/s（约3cm/s）

struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
};

// 四元数乘法（标准）
inline Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
    Quaternion res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

// 四元数归一化（标准）
inline Quaternion normalize(const Quaternion& q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if(norm < 1e-8) return Quaternion(1,0,0,0);
    return Quaternion(q.w/norm, q.x/norm, q.y/norm, q.z/norm);
}

// 四元数导数（标准欧拉积分）
inline Quaternion quatDerivative(const Quaternion& q, const XYZ& body_omega) {
    double w = q.w, x = q.x, y = q.y, z = q.z;
    double wx = body_omega._x, wy = body_omega._y, wz = body_omega._z;
    Quaternion dq;
    dq.w = 0.5 * (-x*wx - y*wy - z*wz);
    dq.x = 0.5 * (w*wx + y*wz - z*wy);
    dq.y = 0.5 * (w*wy - x*wz + z*wx);
    dq.z = 0.5 * (w*wz + x*wy - y*wx);
    return dq;
}

// 四元数转旋转矩阵（标准）
inline void quatToRot(const Quaternion& q, std::vector<std::vector<double>>& rot) {
    double w = q.w, x = q.x, y = q.y, z = q.z;
    rot[0][0] = 1-2*y*y-2*z*z; rot[0][1] = 2*x*y-2*z*w;   rot[0][2] = 2*x*z+2*y*w;
    rot[1][0] = 2*x*y+2*z*w;   rot[1][1] = 1-2*x*x-2*z*z; rot[1][2] = 2*y*z-2*x*w;
    rot[2][0] = 2*x*z-2*y*w;   rot[2][1] = 2*y*z+2*x*w;   rot[2][2] = 1-2*x*x-2*y*y;
}

// RPY转四元数（标准Z-Y-X顺序）
inline Quaternion rpyToQuat(const RPY& rpy) {
    double cr = cos(rpy._roll*0.5), sr = sin(rpy._roll*0.5);
    double cp = cos(rpy._pitch*0.5), sp = sin(rpy._pitch*0.5);
    double cy = cos(rpy._yaw*0.5), sy = sin(rpy._yaw*0.5);
    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

// 四元数转RPY（标准Z-Y-X顺序）
inline RPY quatToRpy(const Quaternion& q) {
    RPY rpy;
    double w = q.w, x = q.x, y = q.y, z = q.z;
    rpy._roll  = atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
    double sin_pitch = -2*(x*z - w*y);
    sin_pitch = std::max(-1.0, std::min(sin_pitch, 1.0));
    rpy._pitch = asin(sin_pitch);
    rpy._yaw   = atan2(2*(w*z + x*y), 1-2*(y*y + z*z));
    return rpy;
}

class ImuOdometry
{
public:
    // ✅ 所有参数完全不变，默认值一个没改
    ImuOdometry(size_t max_queue = 20,
                const XYZ& gravity = XYZ{0.0, 0.0, -1.0}, // 单位：g
                double ang_deadzone = 0.01, // 单位：rad/s
                double acc_deadzone = 0.05, // 单位：g
                double vel_deadzone = 0.02) // 单位：m/s
        : max_queue_size_(max_queue),
          gravity_world_(gravity),
          angular_vel_deadzone_(ang_deadzone),
          linear_acc_deadzone_(acc_deadzone),
          velocity_deadzone_(vel_deadzone),
          vel_damping_(0.999),
          acc_scale_factor_(1.0),
          enable_centripetal_compensation_(false),
          enable_flip_detection_(true), // 默认开启速度翻转检测
          flip_vel_diff_threshold_(FLIP_VEL_DIFF_THRESHOLD)
    {
        current_rot_.assign(3, std::vector<double>(3, 0.0));
        base_quat_ = Quaternion();
        rel_quat_ = Quaternion();
        imu_offset_ = XYZ{0.011, 0.02329, -0.04412}; // 完全保留你的偏移值
    }

    // 原有接口全部保留，签名完全不变
    void addImuData(const sensor_msgs::Imu& imu_msg) {
        cacheImuData(imu_msg);
    }

    // processImu完全不变，仅在开头添加翻转预测逻辑
    XYZRPY processImu(const XYZ& initial_vel, const RPY& base_rpy = RPY()) {
        if (imu_queue_.empty()) {
            XYZRPY res;
            res._xyz._x = 0; res._xyz._y = 0; res._xyz._z = 0;
            res._rpy._roll = 0; res._rpy._pitch = 0; res._rpy._yaw = 0;
            return res;
        }

        // ✅ 核心：积分前先预测速度翻转，提前截断反向数据
        if (enable_flip_detection_ && imu_queue_.size() >= 2) {
            predictAndHandleVelocityFlip(initial_vel);
        }

        // 前3帧平均重力对齐，完全不变
        XYZ avg_acc;
        avg_acc._x = 0; avg_acc._y = 0; avg_acc._z = 0;
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        int count = 0;
        while (!temp_q.empty() && count < 3) {
            const auto& imu = temp_q.front();
            avg_acc._x += imu.linear_acceleration.x;
            avg_acc._y += imu.linear_acceleration.y;
            avg_acc._z += imu.linear_acceleration.z;
            count++;
            temp_q.pop();
        }
        avg_acc._x /= count;
        avg_acc._y /= count;
        avg_acc._z /= count;
        
        base_quat_ = alignGravity(avg_acc);
        rel_quat_ = Quaternion();

        return integrateWindow(initial_vel);
    }

    void resetQueue() {
        std::queue<sensor_msgs::Imu> empty_q;
        std::swap(imu_queue_, empty_q);
    }

    void setMaxQueueSize(size_t size) { max_queue_size_ = size; }
    void setWorldGravity(const XYZ& gravity) { gravity_world_ = gravity; }
    void setAngularVelDeadzone(double threshold) { angular_vel_deadzone_ = std::fabs(threshold); }
    void setLinearAccDeadzone(double threshold) { linear_acc_deadzone_ = std::fabs(threshold); }
    void setVelocityDeadzone(double threshold) { velocity_deadzone_ = std::fabs(threshold); }
    void setVelocityDamping(double val) {
        vel_damping_ = std::max(0.9, std::min(val, 1.0));
    }
    void setImuOffset(const XYZ& offset) { imu_offset_ = offset; }
    void setAccScaleFactor(double factor) { acc_scale_factor_ = factor; }
    void enableCentripetalCompensation(bool enable) { enable_centripetal_compensation_ = enable; }
    // 新增：速度翻转检测开关与阈值设置
    void enableFlipDetection(bool enable) { enable_flip_detection_ = enable; }
    void setFlipVelDiffThreshold(double threshold) { flip_vel_diff_threshold_ = std::fabs(threshold); }

private:
    void cacheImuData(const sensor_msgs::Imu& imu_msg) {
        imu_queue_.push(imu_msg);
        while (imu_queue_.size() > max_queue_size_) imu_queue_.pop();
    }

    // ✅ 完全按照你的要求实现：初速度 + 全队列加速度积分预测翻转
    void predictAndHandleVelocityFlip(const XYZ& initial_vel) {
        // 复制队列用于预计算
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        std::vector<sensor_msgs::Imu> imu_list;
        
        // 将队列转换为列表，方便随机访问
        while (!temp_q.empty()) {
            imu_list.push_back(temp_q.front());
            temp_q.pop();
        }

        // 预计算姿态和加速度，得到每个时刻的速度
        XYZ pred_vel = initial_vel;
        Quaternion pred_quat = base_quat_;
        std::vector<std::vector<double>> pred_rot(3, std::vector<double>(3, 0.0));
        double last_ts = 0.0;
        bool flip_detected = false;
        size_t flip_index = imu_list.size();

        for (size_t i = 0; i < imu_list.size(); i++) {
            const auto& imu = imu_list[i];
            double curr_ts = imu.header.stamp.toSec();
            if (last_ts == 0.0) { last_ts = curr_ts; continue; }
            double dt = curr_ts - last_ts;
            dt = std::max(1e-6, std::min(dt, 0.01));

            // 预计算姿态（与积分时完全一致）
            XYZ body_omega;
            body_omega._x = imu.angular_velocity.x;
            body_omega._y = imu.angular_velocity.y;
            body_omega._z = imu.angular_velocity.z;
            applyDeadzone(body_omega, angular_vel_deadzone_);

            Quaternion dq = quatDerivative(pred_quat, body_omega);
            pred_quat.w += dq.w * dt;
            pred_quat.x += dq.x * dt;
            pred_quat.y += dq.y * dt;
            pred_quat.z += dq.z * dt;
            pred_quat = normalize(pred_quat);
            quatToRot(pred_quat, pred_rot);

            // 预计算加速度（与积分时完全一致）
            XYZ linear_acc;
            linear_acc._x = pred_rot[0][0]*imu.linear_acceleration.x + pred_rot[0][1]*imu.linear_acceleration.y + pred_rot[0][2]*imu.linear_acceleration.z;
            linear_acc._y = pred_rot[1][0]*imu.linear_acceleration.x + pred_rot[1][1]*imu.linear_acceleration.y + pred_rot[1][2]*imu.linear_acceleration.z;
            linear_acc._z = pred_rot[2][0]*imu.linear_acceleration.x + pred_rot[2][1]*imu.linear_acceleration.y + pred_rot[2][2]*imu.linear_acceleration.z;

            // 重力补偿 + 单位转换
            linear_acc._x += gravity_world_._x;
            linear_acc._y += gravity_world_._y;
            linear_acc._z += gravity_world_._z;
            linear_acc._x *= G_TO_MPS2 * acc_scale_factor_;
            linear_acc._y *= G_TO_MPS2 * acc_scale_factor_;
            linear_acc._z *= G_TO_MPS2 * acc_scale_factor_;

            // 向心加速度补偿（如果开启）
            if (enable_centripetal_compensation_) {
                XYZ omega_world;
                omega_world._x = pred_rot[0][0]*body_omega._x + pred_rot[0][1]*body_omega._y + pred_rot[0][2]*body_omega._z;
                omega_world._y = pred_rot[1][0]*body_omega._x + pred_rot[1][1]*body_omega._y + pred_rot[1][2]*body_omega._z;
                omega_world._z = pred_rot[2][0]*body_omega._x + pred_rot[2][1]*body_omega._y + pred_rot[2][2]*body_omega._z;
                XYZ omega_cross_r = crossProduct(omega_world, imu_offset_);
                XYZ centripetal_acc = crossProduct(omega_world, omega_cross_r);
                linear_acc._x -= centripetal_acc._x;
                linear_acc._y -= centripetal_acc._y;
                linear_acc._z -= centripetal_acc._z;
            }

            // 应用死区
            applyDeadzone(linear_acc, linear_acc_deadzone_ * G_TO_MPS2);

            // 预计算速度（欧拉积分，足够准确用于预测）
            XYZ new_vel;
            new_vel._x = pred_vel._x + linear_acc._x * dt;
            new_vel._y = pred_vel._y + linear_acc._y * dt;
            new_vel._z = pred_vel._z + linear_acc._z * dt;

            // ✅ 核心判断：速度符号反转且差值超过阈值
            if ((pred_vel._x * new_vel._x < 0 && fabs(new_vel._x - pred_vel._x) > flip_vel_diff_threshold_) ||
                (pred_vel._y * new_vel._y < 0 && fabs(new_vel._y - pred_vel._y) > flip_vel_diff_threshold_) ||
                (pred_vel._z * new_vel._z < 0 && fabs(new_vel._z - pred_vel._z) > flip_vel_diff_threshold_)) {
                flip_detected = true;
                flip_index = i; // 记录翻转发生的帧索引
                break;
            }

            pred_vel = new_vel;
            last_ts = curr_ts;
        }

        // ✅ 检测到翻转：截断队列，只保留翻转点之前的数据
        if (flip_detected && flip_index > 0) {
            std::queue<sensor_msgs::Imu> new_queue;
            for (size_t i = 0; i < flip_index; i++) {
                new_queue.push(imu_list[i]);
            }
            std::swap(imu_queue_, new_queue);
            
            ROS_DEBUG("速度翻转预测到，队列已截断到第%zu帧", flip_index);
        }
    }

    XYZ crossProduct(const XYZ& v1, const XYZ& v2) const {
        XYZ res;
        res._x = v1._y * v2._z - v1._z * v2._y;
        res._y = v1._z * v2._x - v1._x * v2._z;
        res._z = v1._x * v2._y - v1._y * v2._x;
        return res;
    }

    void applyDeadzone(XYZ& vec, double threshold) const {
        auto soft = [th = threshold](double v) -> double {
            double absv = fabs(v);
            if(absv < th) return 0.0;
            return v * (absv - th) / absv;
        };
        vec._x = soft(vec._x);
        vec._y = soft(vec._y);
        vec._z = soft(vec._z);
    }

    void transformImuToWorld(double x, double y, double z, XYZ& out) const {
        out._x = current_rot_[0][0]*x + current_rot_[0][1]*y + current_rot_[0][2]*z;
        out._y = current_rot_[1][0]*x + current_rot_[1][1]*y + current_rot_[1][2]*z;
        out._z = current_rot_[2][0]*x + current_rot_[2][1]*y + current_rot_[2][2]*z;
    }

    Quaternion alignGravity(const XYZ& imu_acc) const {
        XYZ a = imu_acc;
        double acc_norm = sqrt(a._x*a._x + a._y*a._y + a._z*a._z);
        
        if (acc_norm < 1e-6) return Quaternion();

        a._x /= acc_norm;
        a._y /= acc_norm;
        a._z /= acc_norm;

        XYZ world_up;
        world_up._x = 0.0;
        world_up._y = 0.0;
        world_up._z = 1.0;

        XYZ axis = crossProduct(a, world_up);
        double axis_norm = sqrt(axis._x*axis._x + axis._y*axis._y + axis._z*axis._z);

        if (axis_norm < 1e-6) return Quaternion();

        double dot_product = a._x*world_up._x + a._y*world_up._y + a._z*world_up._z;
        dot_product = std::max(-1.0, std::min(dot_product, 1.0));
        double angle = acos(dot_product);

        double half_angle = angle * 0.5;
        double sin_half = sin(half_angle);
        axis._x /= axis_norm;
        axis._y /= axis_norm;
        axis._z /= axis_norm;

        return Quaternion(
            cos(half_angle),
            axis._x * sin_half,
            axis._y * sin_half,
            axis._z * sin_half
        );
    }

    // 自适应动态调节版：慢运动平滑降噪，剧烈运动全速响应
    XYZRPY integrateWindow(const XYZ& initial_vel)
    {
        XYZ rel_pos; // 单位：m
        XYZ filtered_vel = initial_vel; // 单位：m/s
        applyDeadzone(filtered_vel, velocity_deadzone_);
        XYZ rel_vel = filtered_vel; // 单位：m/s

        double last_ts = 0.0;
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;

        XYZ prev_acc;
        prev_acc._x = 0; prev_acc._y = 0; prev_acc._z = 0;
        bool first_frame = true;

        while (!temp_q.empty())
        {
            const auto imu = temp_q.front();
            temp_q.pop();
            const double curr_ts = imu.header.stamp.toSec();
            if (last_ts == 0.0) { last_ts = curr_ts; continue; }
            double dt = curr_ts - last_ts;
            dt = std::max(1e-6, std::min(dt, 0.01));

            // 1. 姿态更新（完全不变）
            XYZ body_omega;
            body_omega._x = imu.angular_velocity.x;
            body_omega._y = imu.angular_velocity.y;
            body_omega._z = imu.angular_velocity.z;
            applyDeadzone(body_omega, angular_vel_deadzone_);

            Quaternion dq = quatDerivative(rel_quat_, body_omega);
            rel_quat_.w += dq.w * dt;
            rel_quat_.x += dq.x * dt;
            rel_quat_.y += dq.y * dt;
            rel_quat_.z += dq.z * dt;
            rel_quat_ = normalize(rel_quat_);

            Quaternion total_quat = base_quat_ * rel_quat_;
            quatToRot(total_quat, current_rot_);

            // 2. 转换比力到世界坐标系（完全不变）
            XYZ linear_acc;
            transformImuToWorld(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, linear_acc);

            // 3. 重力补偿（完全不变）
            linear_acc._x += gravity_world_._x;
            linear_acc._y += gravity_world_._y;
            linear_acc._z += gravity_world_._z;

            // 4. 单位转换 + 刻度因子修正
            linear_acc._x *= G_TO_MPS2 * acc_scale_factor_;
            linear_acc._y *= G_TO_MPS2 * acc_scale_factor_;
            linear_acc._z *= G_TO_MPS2 * acc_scale_factor_;

            // 5. 可选向心加速度补偿（默认关闭，平动场景不需要）
            if (enable_centripetal_compensation_) {
                XYZ omega_world;
                transformImuToWorld(body_omega._x, body_omega._y, body_omega._z, omega_world);
                XYZ omega_cross_r = crossProduct(omega_world, imu_offset_);
                XYZ centripetal_acc = crossProduct(omega_world, omega_cross_r);
                linear_acc._x -= centripetal_acc._x;
                linear_acc._y -= centripetal_acc._y;
                linear_acc._z -= centripetal_acc._z;
            }

            // 6. 应用加速度死区（完全不变）
            applyDeadzone(linear_acc, linear_acc_deadzone_ * G_TO_MPS2);

            // 自适应平滑系数
            double smooth_alpha = 0.0;
            if (!first_frame) {
                double acc_dot_x = fabs(linear_acc._x - prev_acc._x) / dt;
                double acc_dot_y = fabs(linear_acc._y - prev_acc._y) / dt;
                double acc_dot_z = fabs(linear_acc._z - prev_acc._z) / dt;
                double max_acc_dot = std::max({acc_dot_x, acc_dot_y, acc_dot_z});
                
                smooth_alpha = MAX_SMOOTH_ALPHA * std::max(0.0, 1.0 - max_acc_dot / DYNAMIC_THRESHOLD);
            }

            // 应用自适应平滑
            XYZ smooth_acc;
            smooth_acc._x = (1 - smooth_alpha) * linear_acc._x + smooth_alpha * prev_acc._x;
            smooth_acc._y = (1 - smooth_alpha) * linear_acc._y + smooth_alpha * prev_acc._y;
            smooth_acc._z = (1 - smooth_alpha) * linear_acc._z + smooth_alpha * prev_acc._z;

            // 自适应阻尼系数
            double dynamic_damping = vel_damping_;
            if (!first_frame) {
                double vel_dot_x = fabs(rel_vel._x - filtered_vel._x) / dt;
                double vel_dot_y = fabs(rel_vel._y - filtered_vel._y) / dt;
                double vel_dot_z = fabs(rel_vel._z - filtered_vel._z) / dt;
                double max_vel_dot = std::max({vel_dot_x, vel_dot_y, vel_dot_z});
                
                double damping_reduction = 0.009 * std::min(1.0, max_vel_dot / (DYNAMIC_THRESHOLD * 0.5));
                dynamic_damping = std::max(MIN_DAMPING, vel_damping_ - damping_reduction);
            }

            // 自适应阻尼时机
            if (smooth_alpha > MAX_SMOOTH_ALPHA * 0.5) {
                rel_vel._x *= dynamic_damping;
                rel_vel._y *= dynamic_damping;
                rel_vel._z *= dynamic_damping;
            }

            // 标准中点积分
            if (first_frame) {
                rel_vel._x += smooth_acc._x * dt;
                rel_vel._y += smooth_acc._y * dt;
                rel_vel._z += smooth_acc._z * dt;
                prev_acc = linear_acc;
                first_frame = false;
            } else {
                XYZ mid_acc;
                mid_acc._x = (prev_acc._x + smooth_acc._x) * 0.5;
                mid_acc._y = (prev_acc._y + smooth_acc._y) * 0.5;
                mid_acc._z = (prev_acc._z + smooth_acc._z) * 0.5;
                
                rel_vel._x += mid_acc._x * dt;
                rel_vel._y += mid_acc._y * dt;
                rel_vel._z += mid_acc._z * dt;
                prev_acc = linear_acc;
            }

            // 剧烈运动：后置阻尼
            if (smooth_alpha <= MAX_SMOOTH_ALPHA * 0.5) {
                rel_vel._x *= dynamic_damping;
                rel_vel._y *= dynamic_damping;
                rel_vel._z *= dynamic_damping;
            }

            // 位置中点积分
            if (!first_frame) {
                XYZ mid_vel;
                mid_vel._x = (filtered_vel._x + rel_vel._x) * 0.5;
                mid_vel._y = (filtered_vel._y + rel_vel._y) * 0.5;
                mid_vel._z = (filtered_vel._z + rel_vel._z) * 0.5;
                
                rel_pos._x += mid_vel._x * dt;
                rel_pos._y += mid_vel._y * dt;
                rel_pos._z += mid_vel._z * dt;
            } else {
                rel_pos._x += rel_vel._x * dt;
                rel_pos._y += rel_vel._y * dt;
                rel_pos._z += rel_vel._z * dt;
            }

            filtered_vel = rel_vel;
            last_ts = curr_ts;
        }

        XYZRPY relative_pose;
        relative_pose._xyz = rel_pos;
        relative_pose._rpy = quatToRpy(rel_quat_);
        return relative_pose;
    }

private:
    std::queue<sensor_msgs::Imu> imu_queue_;
    size_t max_queue_size_;
    XYZ gravity_world_; // 单位：g
    double angular_vel_deadzone_; // 单位：rad/s
    double linear_acc_deadzone_; // 单位：g
    double velocity_deadzone_; // 单位：m/s
    double vel_damping_;
    double acc_scale_factor_; // 加速度计刻度因子
    bool enable_centripetal_compensation_; // 向心加速度补偿开关
    XYZ imu_offset_; // IMU相对于旋转中心的安装偏移(m)
    std::vector<std::vector<double>> current_rot_;
    Quaternion base_quat_;
    Quaternion rel_quat_;
    
    // 速度翻转检测相关成员
    bool enable_flip_detection_;
    double flip_vel_diff_threshold_; // 最小速度差值(m/s)
};

}

#endif
