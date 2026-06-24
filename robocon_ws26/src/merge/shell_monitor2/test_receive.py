#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题转发控制节点
功能：
- 订阅一个int32类型的输入话题
- 收到消息值为1时：不执行任何操作
- 收到消息值为2时：立即在输出话题发送2
- 收到其他任何值时：在输出话题发送1
"""

import rospy
from std_msgs.msg import Int32

class TopicForwarder:
    def __init__(self):
        # 初始化节点
        rospy.init_node('topic_forwarder_node')

        # 可配置的话题名称（建议通过launch文件参数化）
        self.input_topic = rospy.get_param('~input_topic', '/scripts/send_path')
        self.output_topic = rospy.get_param('~output_topic', '/scripts/response')

        # 创建发布者（队列大小设为10，足够大多数场景使用）
        self.pub = rospy.Publisher(self.output_topic, Int32, queue_size=10)

        # 创建订阅者
        self.sub = rospy.Subscriber(self.input_topic, Int32, self.callback)

        rospy.loginfo("话题转发控制节点已启动")
        rospy.loginfo(f"订阅话题: {self.input_topic}")
        rospy.loginfo(f"发布话题: {self.output_topic}")
        rospy.loginfo("等待消息...")

    def callback(self, msg):
        """消息回调函数，处理收到的int32消息"""
        value = msg.data
        # rospy.logdebug(f"收到消息: {value}")

        # 核心逻辑处理
        if value == 1:
            # 收到1时什么都不做
            self.pub.publish(1)
            rospy.loginfo(f"收到值{value}，已发送1")
            return
        elif value == 2:
            # 收到2时立即发送2
            self.pub.publish(2)
            rospy.loginfo("收到值2，已发送2")
        # else:
        #     # 其他任何值都发送1
        #     self.pub.publish(1)
        #     rospy.loginfo(f"收到值{value}，已发送1")

    def run(self):
        """运行节点，保持阻塞直到ROS关闭"""
        rospy.spin()


if __name__ == '__main__':
    try:
        forwarder = TopicForwarder()
        forwarder.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已被中断")
    except Exception as e:
        rospy.logerr(f"节点运行出错: {str(e)}")