#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
import math
from std_msgs.msg import Float32MultiArray


def talker():
    # 初始化 ROS 节点
    rospy.init_node('mock_lower_data_publisher', anonymous=True)

    # 创建发布者，发布到下位机对应的话题
    pub = rospy.Publisher("/scripts/liao", Float32MultiArray, queue_size=10)

    # 设置发布频率（例如 10Hz）
    rate = rospy.Rate(10)

    rospy.loginfo("虚拟下位机数据发布器已启动，正在发送数据...")

    # 为了让数据看起来有连续性，设置几个初始值
    x, y, yaw = 0.0, 0.0, 0.0

    while not rospy.is_shutdown():
        # 模拟机器人的运动产生连续或随机的数据
        x += random.uniform(-0.05, 0.1)
        y += random.uniform(-0.05, 0.05)
        yaw = (yaw + random.uniform(-5.0, 5.0)) % 360.0  # 模拟角度 0~360 循环

        # 模拟激光雷达测距数据（随机 0.1 ~ 8.0 米）
        laser_1 = random.uniform(0.1, 8.0)
        laser_2 = random.uniform(0.1, 8.0)

        # 组装成指定的五位数组格式: ["X", "Y", "Yaw", "激光1", "激光2"]
        data_list = [x, y, yaw, laser_1, laser_2]

        # 创建并填充 Float64MultiArray 消息
        msg = Float32MultiArray()
        msg.data = data_list

        # 发布消息
        pub.publish(msg)

        # 打印日志以便在终端观察
        rospy.loginfo(f"Published: X={x:.3f}, Y={y:.3f}, Yaw={yaw:.1f}°, L1={laser_1:.2f}m, L2={laser_2:.2f}m")

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass