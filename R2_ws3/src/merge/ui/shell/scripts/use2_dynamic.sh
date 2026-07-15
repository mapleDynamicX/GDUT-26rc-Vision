#!/bin/bash
source /opt/ros/noetic/setup.bash

# 使用环境变量获取当前用户
current_user=$USER
root1="/home/"
root2="/RC2026/"
root_path="${root1}${current_user}${root2}"
target_path=""

# 判断目录是否存在：存在返回0（真），不存在返回1（假）
judge_dir_exist()
{
    if [ -d "$1" ]; then
        return 0  # 存在 → 条件成立
    else
        return 1  # 不存在 → 条件不成立
    fi
}

# 遍历查找第一个存在的 robocon_ws 目录
for i in $(seq 1 100); do
    root_name="robocon_ws"
    if [ $i -eq 1 ]; then
    	path="${root_path}${root_name}"
    else
    	path="${root_path}${root_name}${i}"
    fi
    if judge_dir_exist "$path"; then
        target_path="$path"
    fi
done

# 容错：没找到目录就报错退出
if [ -z "$target_path" ]; then
    echo "错误：未找到任何 robocon_ws 工作目录"
    exit 1
fi

# 切换目录并加载环境、启动 launch
cd "$target_path" || exit 1

echo "运行路径: "$target_path

source ./devel/setup.bash
roslaunch merge merge2.launch
