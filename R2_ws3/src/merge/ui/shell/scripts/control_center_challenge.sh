#!/bin/bash
source /opt/ros/noetic/setup.bash

# 使用环境变量获取当前用户
current_user=$USER
root1="/home/"
root2="/rc2026/"
root_path="${root1}${current_user}${root2}"
workspace_path="robocon_ws4"
target_path=$root1$current_user$root2$workspace_path

# 切换目录并加载环境、启动 launch
cd "$target_path" || exit 1
path2="/src/merge/shell_monitor3/"
path=$target_path$path2
cd $path
python3 control_center_challenge.py
