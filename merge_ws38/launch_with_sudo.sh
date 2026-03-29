#!/bin/bash
# 加载 ROS 环境
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
#sleep
sleep 1
# 运行 launch 文件
#roslaunch merge merge.launch
rosrun merge merge_node
