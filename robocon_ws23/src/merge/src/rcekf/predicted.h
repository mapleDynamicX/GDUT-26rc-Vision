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
constexpr double DYNAMIC_THRESHOLD = 1.0; // m/s³
// 最大平滑系数：慢运动时的最大平滑程度
constexpr double MAX_SMOOTH_ALPHA = 0.3;
// 最小阻尼系数：剧烈运动时的最小阻尼
constexpr double MIN_DAMPING = 0.98;
// 重力对齐窗口大小：仅在静止时使用前N帧对齐重力
constexpr size_t GRAVITY_ALIGN_WINDOW = 10;

struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
};

// XYZ类型运算符重载
inline XYZ operator*(const XYZ& v, double scalar) {
    return XYZ{v._x * scalar, v._y * scalar, v._z * scalar};
}

inline XYZ operator*(double scalar, const XYZ& v) {
    return v * scalar;
}

inline XYZ operator+(const XYZ& v1, const XYZ& v2) {
    return XYZ{v1._x + v2._x, v1._y + v2._y, v1._z + v2._z};
}

inline XYZ operator-(const XYZ& v1, const XYZ& v2) {
    return XYZ{v1._x - v2._x, v1._y - v2._y, v1._z - v2._z};
}

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

// 四元数指数映射（精确姿态更新）
inline Quaternion quatExponentialMap(const XYZ& omega, double dt) {
    double theta = sqrt(omega._x*omega._x + omega._y*omega._y + omega._z*omega._z) * dt;
    if (theta < 1e-6) {
        return Quaternion(1, 0.5*omega._x*dt, 0.5*omega._y*dt, 0.5*omega._z*dt);
    }
    double half_theta = theta * 0.5;
    double sin_half = sin(half_theta) / theta;
    return Quaternion(
        cos(half_theta),
        omega._x * sin_half * dt,
        omega._y * sin_half * dt,
        omega._z * sin_half * dt
    );
}

// 用四元数旋转向量（优化实现）
inline XYZ rotateVectorByQuat(const XYZ& v, const Quaternion& q) {
    double qv_x = q.y*v._z - q.z*v._y;
    double qv_y = q.z*v._x - q.x*v._z;
    double qv_z = q.x*v._y - q.y*v._x;
    
    XYZ res;
    res._x = v._x + 2 * (q.w*qv_x + q.y*qv_z - q.z*qv_y);
    res._y = v._y + 2 * (q.w*qv_y + q.z*qv_x - q.x*qv_z);
    res._z = v._z + 2 * (q.w*qv_z + q.x*qv_y - q.y*qv_x);
    return res;
}

// 四元数转旋转矩阵（标准，保留用于兼容）
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
          enable_coriolis_compensation_(true),
          imu_offset_(XYZ{0.011, 0.02329, -0.04412}),
          is_initialized_(false),
          last_processed_ts_(0.0)
    {
        current_rot_.assign(3, std::vector<double>(3, 0.0));
        base_quat_ = Quaternion();
        rel_quat_ = Quaternion();
        
        // 初始化增量式预积分状态
        resetIntegration();
    }

    // 原有接口全部保留，签名完全不变
    void addImuData(const sensor_msgs::Imu& imu_msg) {
        cacheImuData(imu_msg);
    }

    // ✅ processImu接口完全不变，内部已改为增量式预积分
    XYZRPY processImu(const XYZ& initial_vel, const RPY& base_rpy = RPY()) {
        if (imu_queue_.empty()) {
            XYZRPY res;
            res._xyz._x = 0; res._xyz._y = 0; res._xyz._z = 0;
            res._rpy._roll = 0; res._rpy._pitch = 0; res._rpy._yaw = 0;
            return res;
        }

        // ✅ 仅在第一次调用时对齐重力，之后不再重新计算
        if (!is_initialized_) {
            initializeGravityAlignment();
            is_initialized_ = true;
        }

        // ✅ 增量式积分：只处理上次调用后新增的IMU数据
        integrateNewData(initial_vel);

        // 计算当前窗口的相对位姿
        XYZRPY relative_pose;
        relative_pose._xyz = total_delta_p_;
        relative_pose._rpy = quatToRpy(total_delta_q_);
        return relative_pose;
    }

    void resetQueue() {
        std::queue<sensor_msgs::Imu> empty_q;
        std::swap(imu_queue_, empty_q);
        resetIntegration();
        is_initialized_ = false;
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
    void enableCoriolisCompensation(bool enable) { enable_coriolis_compensation_ = enable; }

private:
    void cacheImuData(const sensor_msgs::Imu& imu_msg) {
        imu_queue_.push(imu_msg);
        while (imu_queue_.size() > max_queue_size_) {
            // ✅ 窗口滑动时，移除最旧的数据并从积分结果中减去其贡献
            if (is_initialized_ && !integration_history_.empty()) {
                const auto& oldest = integration_history_.front();
                total_delta_p_ = total_delta_p_ - oldest.delta_p;
                total_delta_v_ = total_delta_v_ - oldest.delta_v;
                // 姿态无法简单减法，这里我们保持总姿态不变
                // 对于0.1s的短窗口，姿态漂移可以忽略
                integration_history_.pop_front();
            }
            imu_queue_.pop();
        }
    }

    XYZ crossProduct(const XYZ& v1, const XYZ& v2) const {
        XYZ res;
        res._x = v1._y * v2._z - v1._z * v2._y;
        res._y = v1._z * v2._x - v1._x * v2._z;
        res._z = v1._x * v2._y - v1._y * v2._x;
        return res;
    }

    // ✅ 改进：平滑死区函数，消除阈值附近的跳变
    void applyDeadzone(XYZ& vec, double threshold) const {
        auto smooth_soft = [th = threshold](double v) -> double {
            double absv = fabs(v);
            if(absv < th * 0.5) return 0.0;
            if(absv > th * 1.5) return v * (absv - th) / absv;
            // 0.5th ~ 1.5th 之间平滑过渡
            double t = (absv - th * 0.5) / th;
            double factor = t * t * (3 - 2 * t); // 三次平滑函数
            return v * factor * (absv - th * 0.5) / absv;
        };
        vec._x = smooth_soft(vec._x);
        vec._y = smooth_soft(vec._y);
        vec._z = smooth_soft(vec._z);
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

    // ✅ 初始化重力对齐：使用前N帧的平均加速度，确保是静止状态
    void initializeGravityAlignment() {
        XYZ avg_acc;
        avg_acc._x = 0; avg_acc._y = 0; avg_acc._z = 0;
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        int count = 0;
        
        // 使用前GRAVITY_ALIGN_WINDOW帧进行重力对齐
        while (!temp_q.empty() && count < GRAVITY_ALIGN_WINDOW) {
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
    }

    // ✅ 重置增量式预积分状态
    void resetIntegration() {
        total_delta_p_ = {0, 0, 0};
        total_delta_v_ = {0, 0, 0};
        total_delta_q_ = Quaternion(1, 0, 0, 0);
        last_processed_ts_ = 0.0;
        integration_history_.clear();
    }

    // ✅ 核心：增量式预积分，只处理新数据
    void integrateNewData(const XYZ& initial_vel) {
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        // 跳过已经处理过的数据
        while (!temp_q.empty() && temp_q.front().header.stamp.toSec() <= last_processed_ts_) {
            temp_q.pop();
        }

        if (temp_q.empty()) return;

        XYZ prev_acc_body = {0, 0, 0};
        XYZ prev_omega_body = {0, 0, 0};
        double last_ts = 0.0;
        bool first_frame = true;

        while (!temp_q.empty()) {
            const auto& imu = temp_q.front();
            temp_q.pop();
            const double curr_ts = imu.header.stamp.toSec();
            
            if (last_ts == 0.0) { 
                last_ts = curr_ts; 
                prev_omega_body._x = imu.angular_velocity.x;
                prev_omega_body._y = imu.angular_velocity.y;
                prev_omega_body._z = imu.angular_velocity.z;
                applyDeadzone(prev_omega_body, angular_vel_deadzone_);
                
                prev_acc_body._x = imu.linear_acceleration.x;
                prev_acc_body._y = imu.linear_acceleration.y;
                prev_acc_body._z = imu.linear_acceleration.z;
                applyDeadzone(prev_acc_body, linear_acc_deadzone_);
                continue; 
            }
            
            double dt = curr_ts - last_ts;
            dt = std::max(1e-6, std::min(dt, 0.01));

            // 1. 角速度处理
            XYZ omega_body = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
            applyDeadzone(omega_body, angular_vel_deadzone_);

            // 2. 姿态预积分
            XYZ avg_omega;
            avg_omega._x = (prev_omega_body._x + omega_body._x) * 0.5;
            avg_omega._y = (prev_omega_body._y + omega_body._y) * 0.5;
            avg_omega._z = (prev_omega_body._z + omega_body._z) * 0.5;
            
            Quaternion dq = quatExponentialMap(avg_omega, dt);
            Quaternion new_delta_q = normalize(total_delta_q_ * dq);

            // 3. 加速度处理
            XYZ acc_body = {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
            applyDeadzone(acc_body, linear_acc_deadzone_);
            acc_body._x *= G_TO_MPS2 * acc_scale_factor_;
            acc_body._y *= G_TO_MPS2 * acc_scale_factor_;
            acc_body._z *= G_TO_MPS2 * acc_scale_factor_;

            // 4. 运动学模型补偿
            if (enable_centripetal_compensation_ || enable_coriolis_compensation_) {
                XYZ omega_cross_r = crossProduct(avg_omega, imu_offset_);
                
                if (enable_centripetal_compensation_) {
                    XYZ centripetal_acc = crossProduct(avg_omega, omega_cross_r);
                    acc_body._x -= centripetal_acc._x;
                    acc_body._y -= centripetal_acc._y;
                    acc_body._z -= centripetal_acc._z;
                }
                
                if (enable_coriolis_compensation_) {
                    XYZ coriolis_acc = crossProduct(2.0 * avg_omega, total_delta_v_);
                    acc_body._x -= coriolis_acc._x;
                    acc_body._y -= coriolis_acc._y;
                    acc_body._z -= coriolis_acc._z;
                }
            }

            // 5. RK4积分加速度
            XYZ acc_rot_prev = rotateVectorByQuat(prev_acc_body, total_delta_q_);
            XYZ acc_rot_curr = rotateVectorByQuat(acc_body, new_delta_q);
            
            XYZ k1 = acc_rot_prev;
            XYZ k2;
            k2._x = (acc_rot_prev._x + acc_rot_curr._x) * 0.5;
            k2._y = (acc_rot_prev._y + acc_rot_curr._y) * 0.5;
            k2._z = (acc_rot_prev._z + acc_rot_curr._z) * 0.5;
            XYZ k3 = k2;
            XYZ k4 = acc_rot_curr;
            
            XYZ delta_v_step;
            delta_v_step._x = (k1._x + 2*k2._x + 2*k3._x + k4._x) * dt / 6.0;
            delta_v_step._y = (k1._y + 2*k2._y + 2*k3._y + k4._y) * dt / 6.0;
            delta_v_step._z = (k1._z + 2*k2._z + 2*k3._z + k4._z) * dt / 6.0;
            
            // ✅ 每一步都进行重力补偿（在IMU坐标系下）
            XYZ gravity_body = rotateVectorByQuat(gravity_world_ * G_TO_MPS2, total_delta_q_.inverse());
            delta_v_step = delta_v_step + gravity_body * dt;
            
            XYZ new_delta_v = total_delta_v_ + delta_v_step;

            // 6. RK4积分位置
            XYZ k1_p = total_delta_v_;
            XYZ k2_p;
            k2_p._x = total_delta_v_._x + delta_v_step._x * 0.5;
            k2_p._y = total_delta_v_._y + delta_v_step._y * 0.5;
            k2_p._z = total_delta_v_._z + delta_v_step._z * 0.5;
            XYZ k3_p = k2_p;
            XYZ k4_p;
            k4_p._x = total_delta_v_._x + delta_v_step._x;
            k4_p._y = total_delta_v_._y + delta_v_step._y;
            k4_p._z = total_delta_v_._z + delta_v_step._z;
            
            XYZ delta_p_step;
            delta_p_step._x = (k1_p._x + 2*k2_p._x + 2*k3_p._x + k4_p._x) * dt / 6.0;
            delta_p_step._y = (k1_p._y + 2*k2_p._y + 2*k3_p._y + k4_p._y) * dt / 6.0;
            delta_p_step._z = (k1_p._z + 2*k2_p._z + 2*k3_p._z + k4_p._z) * dt / 6.0;
            
            XYZ new_delta_p = total_delta_p_ + delta_p_step;

            // 7. 保存积分历史，用于窗口滑动时的增量移除
            IntegrationStep step;
            step.delta_p = delta_p_step;
            step.delta_v = delta_v_step;
            step.delta_q = dq;
            step.timestamp = curr_ts;
            integration_history_.push_back(step);

            // 8. 更新总积分状态
            total_delta_p_ = new_delta_p;
            total_delta_v_ = new_delta_v;
            total_delta_q_ = new_delta_q;

            // 更新上一帧数据
            prev_acc_body = acc_body;
            prev_omega_body = omega_body;
            last_ts = curr_ts;
            last_processed_ts_ = curr_ts;
            first_frame = false;
        }

        // 9. 自适应后处理平滑
        if (!first_frame && integration_history_.size() >= 2) {
            double max_acc_dot = 0.0;
            for (size_t i = 1; i < integration_history_.size(); i++) {
                double dt_i = integration_history_[i].timestamp - integration_history_[i-1].timestamp;
                double acc_dot_x = fabs(integration_history_[i].delta_v._x - integration_history_[i-1].delta_v._x) / dt_i;
                double acc_dot_y = fabs(integration_history_[i].delta_v._y - integration_history_[i-1].delta_v._y) / dt_i;
                double acc_dot_z = fabs(integration_history_[i].delta_v._z - integration_history_[i-1].delta_v._z) / dt_i;
                max_acc_dot = std::max({max_acc_dot, acc_dot_x, acc_dot_y, acc_dot_z});
            }
            
            double smooth_alpha = MAX_SMOOTH_ALPHA * std::max(0.0, 1.0 - max_acc_dot / DYNAMIC_THRESHOLD);
            
            double dynamic_damping = vel_damping_;
            if (integration_history_.size() >= 2) {
                double max_vel_dot = 0.0;
                for (size_t i = 1; i < integration_history_.size(); i++) {
                    double dt_i = integration_history_[i].timestamp - integration_history_[i-1].timestamp;
                    double vel_dot_x = fabs(integration_history_[i].delta_p._x - integration_history_[i-1].delta_p._x) / dt_i;
                    double vel_dot_y = fabs(integration_history_[i].delta_p._y - integration_history_[i-1].delta_p._y) / dt_i;
                    double vel_dot_z = fabs(integration_history_[i].delta_p._z - integration_history_[i-1].delta_p._z) / dt_i;
                    max_vel_dot = std::max({max_vel_dot, vel_dot_x, vel_dot_y, vel_dot_z});
                }
                double damping_reduction = 0.009 * std::min(1.0, max_vel_dot / (DYNAMIC_THRESHOLD * 0.5));
                dynamic_damping = std::max(MIN_DAMPING, vel_damping_ - damping_reduction);
            }
            
            total_delta_v_ = total_delta_v_ * dynamic_damping;
        }

        // 应用速度死区
        applyDeadzone(total_delta_v_, velocity_deadzone_);

        // 更新全局姿态（保持与原代码兼容）
        rel_quat_ = total_delta_q_;
        quatToRot(base_quat_ * rel_quat_, current_rot_);
    }

    // 积分步骤结构体，用于保存历史数据
    struct IntegrationStep {
        XYZ delta_p;
        XYZ delta_v;
        Quaternion delta_q;
        double timestamp;
    };

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
    bool enable_coriolis_compensation_; // 科里奥利加速度补偿开关
    XYZ imu_offset_; // IMU相对于旋转中心的安装偏移(m)
    std::vector<std::vector<double>> current_rot_;
    Quaternion base_quat_;
    Quaternion rel_quat_;
    
    // ✅ 增量式预积分状态
    bool is_initialized_;
    double last_processed_ts_;
    XYZ total_delta_p_; // 总位置增量（IMU坐标系下）
    XYZ total_delta_v_; // 总速度增量（IMU坐标系下）
    Quaternion total_delta_q_; // 总姿态增量
    std::deque<IntegrationStep> integration_history_; // 积分历史
};

}

#endif
