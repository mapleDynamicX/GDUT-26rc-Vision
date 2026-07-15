#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

def mock_sender():
    # 初始化 ROS 节点
    rospy.init_node('mock_upper_data_sender', anonymous=True)

    # ==========================================================
    # 注意：请将以下话题名称修改为与你的 monitor_config.json 中完全一致的 topic
    # ==========================================================
    odom_topic = "/R2/odom"
    imu_topic = "/livox/imu"

    # 创建发布者
    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
    imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=10)

    # 设置发送频率 (例如 30Hz，与上位机接收频率匹配)
    rate = rospy.Rate(30)
    rospy.loginfo(f"开始模拟发送上位机数据...\nOdom话题: {odom_topic}\nImu话题: {imu_topic}")

    start_time = time.time()

    while not rospy.is_shutdown():
        t = time.time() - start_time

        # ---------------- 1. 模拟构建 Odometry 数据 ----------------
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"

        # 模拟产生一个圆周运动的位置 (XYZ)
        odom_msg.pose.pose.position.x = 2.0 * math.cos(0.5 * t)
        odom_msg.pose.pose.position.y = 2.0 * math.sin(0.5 * t)
        odom_msg.pose.pose.position.z = 0.5 * math.sin(t)

        # 模拟产生变化的 RPY 姿态角 (弧度)
        roll = 0.1 * math.sin(t)
        pitch = 0.05 * math.cos(t)
        yaw = 0.5 * t  # 持续打转

        # 将 RPY 转换为四元数
        q_odom = quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = q_odom[0]
        odom_msg.pose.pose.orientation.y = q_odom[1]
        odom_msg.pose.pose.orientation.z = q_odom[2]
        odom_msg.pose.pose.orientation.w = q_odom[3]

        # ---------------- 2. 模拟构建 Imu 数据 ----------------
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # 模拟 Imu 姿态 (这里直接复用上面的四元数)
        imu_msg.orientation.x = q_odom[0]
        imu_msg.orientation.y = q_odom[1]
        imu_msg.orientation.z = q_odom[2]
        imu_msg.orientation.w = q_odom[3]

        # 模拟角速度 (rad/s)
        imu_msg.angular_velocity.x = 0.1 * math.cos(t)
        imu_msg.angular_velocity.y = 0.05 * math.sin(t)
        imu_msg.angular_velocity.z = 0.5  # 绕 Z 轴恒定旋转

        # 模拟线加速度 (m/s^2)，包含重力加速度分量
        imu_msg.linear_acceleration.x = 0.2 * math.sin(2 * t)
        imu_msg.linear_acceleration.y = 0.2 * math.cos(2 * t)
        imu_msg.linear_acceleration.z = 9.81 + 0.5 * math.sin(t)

        # ---------------- 3. 发布数据 ----------------
        odom_pub.publish(odom_msg)
        imu_pub.publish(imu_msg)

        # 按照设定频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        mock_sender()
    except rospy.ROSInterruptException:
        pass