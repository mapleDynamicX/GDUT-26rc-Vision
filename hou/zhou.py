#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import math
from matplotlib.widgets import Button, Slider
import warnings
# 新增tkinter导入（GUI输入窗口）
import tkinter as tk
from tkinter import messagebox, Listbox, END
warnings.filterwarnings("ignore")

# ===================== 配置参数 =====================
plt.rcParams["font.family"] = "WenQuanYi Micro Hei"
plt.rcParams["axes.unicode_minus"] = False
margin_ratio = 0.1    # 边距比例
ZOOM_FACTOR = 0.1     # 滚轮缩放步长
PLAY_SPEED = 0.05     # 自动播放步长（秒）
FRAME_INTERVAL = 50   # 动画帧间隔（毫秒）
SLIDER_VALSTEP = 0.01 # 时间轴滑块精度（秒）
# ====================================================

def quaternion_to_euler(x, y, z, w):
    """
    四元数转欧拉角（roll, pitch, yaw）
    输入：四元数x,y,z,w（ROS标准）
    输出：roll (rad), pitch (rad), yaw (rad)
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))  # 防止数值溢出
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def calculate_accurate_frequency(bag, topic, max_valid_dt=1.0):
    """
    鲁棒计算话题的真实频率（修复高频虚高问题）
    """
    timestamps = []
    try:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            try:
                ts = msg.header.stamp.to_sec()
                timestamps.append(ts)
            except AttributeError:
                return None
    except:
        return None
    
    if len(timestamps) < 2:
        return None
    
    total_duration = timestamps[-1] - timestamps[0]
    if total_duration < 1e-6:
        return None
    fallback_freq = len(timestamps) / total_duration
    dt_array = np.diff(np.array(timestamps))
    valid_dt = dt_array[(dt_array > 0.0001) & (dt_array < max_valid_dt)]
    
    if len(valid_dt) == 0:
        return fallback_freq
    
    median_dt = np.median(valid_dt)
    median_freq = 1 / median_dt
    is_imu = "imu" in topic.lower()
    is_odom = "odom" in topic.lower() or "ekf" in topic.lower() or "raw" in topic.lower() or "pose" in topic.lower()
    
    if is_odom and median_freq > 100:
        print(f"⚠️ {topic} 中位数频率{median_freq:.2f}Hz（ODOM超阈值），改用真实发布频率{fallback_freq:.2f}Hz")
        return fallback_freq
    elif is_imu and (median_freq < 100 or median_freq > 500):
        print(f"⚠️ {topic} 中位数频率{median_freq:.2f}Hz（IMU超合理范围），改用真实发布频率{fallback_freq:.2f}Hz")
        return fallback_freq
    elif median_freq > 500:
        print(f"⚠️ {topic} 中位数频率{median_freq:.2f}Hz（异常高频），改用真实发布频率{fallback_freq:.2f}Hz")
        return fallback_freq
    
    return median_freq

def get_all_topics_and_select(bag_path):
    bag = rosbag.Bag(bag_path)
    all_topics = list(bag.get_type_and_topic_info()[1].keys())
    
    print("="*50)
    print(f"检测到bag中所有话题（共{len(all_topics)}个）：")
    topic_freq_map = {}
    # 提前标记话题类型（ODOM/IMU/未知）
    topic_type_map = {}
    for idx, topic in enumerate(all_topics, 1):
        freq = calculate_accurate_frequency(bag, topic)
        topic_freq_map[topic] = freq
        # 标记话题类型
        if "odom" in topic.lower() or "ekf" in topic.lower() or "raw" in topic.lower() or "pose" in topic.lower():
            topic_type_map[topic] = "odom"
        elif "imu" in topic.lower():
            topic_type_map[topic] = "imu"
        else:
            topic_type_map[topic] = "unknown"
        freq_str = f"{freq:.2f} Hz" if freq is not None else "N/A"
        type_str = topic_type_map[topic].upper()
        print(f"{idx}. {topic} | 类型：{type_str} | 频率：{freq_str}")
    print("="*50)
    
    while True:
        user_input = input("请输入以0结尾数字串选话题(如13450选1、3、4、5号)：").strip()
        if not user_input.endswith('0') or not user_input[:-1].isdigit():
            print("格式错误！必须以0结尾，例：30、120")
            continue
        num_str = user_input[:-1]
        if not num_str:
            print("0前面必须填数字！")
            continue
        try:
            indices = [int(c)-1 for c in num_str]
        except:
            print("只能输入数字！")
            continue
        indices = sorted(list(set(indices)))
        invalid = [i+1 for i in indices if i<0 or i>=len(all_topics)]
        if invalid:
            print(f"无效序号：{invalid}，范围1~{len(all_topics)}")
            continue
        if not indices:
            print("至少选一个话题！")
            continue
        break
    selected_topics = [all_topics[i] for i in indices]
    print(f"已选序号：{[i+1 for i in indices]}")
    for t in selected_topics:
        freq_str = f"{topic_freq_map[t]:.2f} Hz" if topic_freq_map[t] is not None else "N/A"
        type_str = topic_type_map[t].upper()
        print(f"{t} | 类型：{type_str} | 频率：{freq_str}")
    print("="*50)
    bag.close()
    return selected_topics, topic_type_map

def read_sensor_data(bag_path, topics, topic_type_map, data_subtype=None):
    """
    分类型读取数据：
    - ODOM类话题：位姿（position/orientation）、线速度（linear velocity）、角速度（angular velocity）
    - IMU类话题：线加速度、角速度、角加速度
    - 未知类型：跳过
    """
    data = {t: {"raw_t": [], "x": [], "y": [], "z": [], 
                "roll": [], "pitch": [], "yaw": [], "type": None} for t in topics}
    bag = rosbag.Bag(bag_path)
    
    for topic, msg, _ in bag.read_messages(topics=topics):
        if topic not in data:
            continue
        
        # 获取话题类型（ODOM/IMU/未知）
        topic_type = topic_type_map.get(topic, "unknown")
        try:
            ts = msg.header.stamp.to_sec()
        except AttributeError:
            print(f"⚠️ {topic} 无header.stamp字段，跳过该消息")
            continue
        
        # 1. 处理ODOM类话题
        if topic_type == "odom":
            if data_subtype is None:
                print(f"⚠️ {topic} (ODOM) 未指定数据子类型，跳过")
                continue
            
            try:
                data[topic]["raw_t"].append(ts)
                # ODOM-位姿数据
                if data_subtype == "position":
                    # 位置坐标
                    data[topic]["x"].append(msg.pose.pose.position.x)
                    data[topic]["y"].append(msg.pose.pose.position.y)
                    data[topic]["z"].append(msg.pose.pose.position.z)
                    # 四元数转欧拉角（姿态角）
                    qx = msg.pose.pose.orientation.x
                    qy = msg.pose.pose.orientation.y
                    qz = msg.pose.pose.orientation.z
                    qw = msg.pose.pose.orientation.w
                    roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
                    data[topic]["roll"].append(roll)
                    data[topic]["pitch"].append(pitch)
                    data[topic]["yaw"].append(yaw)
                    data[topic]["type"] = "position"
                
                # ODOM-线速度（twist.twist.linear）
                elif data_subtype == "odom_linear_vel":
                    data[topic]["x"].append(msg.twist.twist.linear.x)
                    data[topic]["y"].append(msg.twist.twist.linear.y)
                    data[topic]["z"].append(msg.twist.twist.linear.z)
                    data[topic]["type"] = "odom_linear_velocity"
                
                # ODOM-角速度（twist.twist.angular）
                elif data_subtype == "odom_angular_vel":
                    data[topic]["x"].append(msg.twist.twist.angular.x)
                    data[topic]["y"].append(msg.twist.twist.angular.y)
                    data[topic]["z"].append(msg.twist.twist.angular.z)
                    data[topic]["type"] = "odom_angular_velocity"
            
            except AttributeError as e:
                print(f"⚠️ {topic} (ODOM-{data_subtype}) 缺失字段：{str(e)}，跳过该消息")
                continue
        
        # 2. 处理IMU类话题
        elif topic_type == "imu":
            if data_subtype is None:
                print(f"⚠️ {topic} (IMU) 未指定数据子类型，跳过")
                continue
            
            try:
                data[topic]["raw_t"].append(ts)
                # IMU-线加速度
                if data_subtype == "imu_linear_acc":
                    data[topic]["x"].append(msg.linear_acceleration.x)
                    data[topic]["y"].append(msg.linear_acceleration.y)
                    data[topic]["z"].append(msg.linear_acceleration.z)
                    data[topic]["type"] = "imu_linear_acceleration"
                
                # IMU-角速度
                elif data_subtype == "imu_angular_vel":
                    data[topic]["x"].append(msg.angular_velocity.x)
                    data[topic]["y"].append(msg.angular_velocity.y)
                    data[topic]["z"].append(msg.angular_velocity.z)
                    data[topic]["type"] = "imu_angular_velocity"
                
                # IMU-角加速度
                elif data_subtype == "imu_angular_acc":
                    data[topic]["x"].append(msg.angular_acceleration.x)
                    data[topic]["y"].append(msg.angular_acceleration.y)
                    data[topic]["z"].append(msg.angular_acceleration.z)
                    data[topic]["type"] = "imu_angular_acceleration"
            
            except AttributeError as e:
                print(f"⚠️ {topic} (IMU-{data_subtype}) 缺失字段：{str(e)}，跳过该消息")
                continue
        
        # 3. 未知类型话题
        else:
            print(f"⚠️ {topic} 为未知类型，跳过")
            continue
    
    bag.close()
    # 核心修复：统一所有数据长度，包括姿态角
    valid_data = {}
    for t in data:
        t_list = data[t]["raw_t"]
        base_lists = [t_list, data[t]["x"], data[t]["y"], data[t]["z"]]
        # 位姿数据需要包含姿态角
        if data[t]["type"] == "position":
            base_lists.extend([data[t]["roll"], data[t]["pitch"], data[t]["yaw"]])
        
        min_len = min(len(lst) for lst in base_lists)
        if min_len < 5:
            print(f"跳过 {t}：有效数据过少（仅{min_len}条）")
            continue
        
        # 所有数据统一截断到最短长度
        data[t]["raw_t"] = t_list[:min_len]
        data[t]["x"] = data[t]["x"][:min_len]
        data[t]["y"] = data[t]["y"][:min_len]
        data[t]["z"] = data[t]["z"][:min_len]
        if data[t]["type"] == "position":
            data[t]["roll"] = data[t]["roll"][:min_len]
            data[t]["pitch"] = data[t]["pitch"][:min_len]
            data[t]["yaw"] = data[t]["yaw"][:min_len]
        
        valid_data[t] = data[t]
    
    if not valid_data:
        raise ValueError("无有效ODOM/IMU数据！")
    
    print("=== 有效话题信息 ===")
    for t,d in valid_data.items():
        topic_type = topic_type_map.get(t, "unknown").upper()
        print(f"{t} | 类型：{topic_type}/{d['type']} | 点数：{len(d['raw_t'])}")
    return valid_data

def align_by_raw_timestamp(data, axis):
    """
    修复：提前校验数据长度，避免维度不匹配错误
    """
    topics = list(data.keys())
    if not topics:
        raise ValueError("无有效话题数据")
    
    topic_raw_t = {t: np.array(data[t]["raw_t"]) for t in topics}
    t_start = max([arr[0] for arr in topic_raw_t.values()])
    t_end = min([arr[-1] for arr in topic_raw_t.values()])
    topic_times = {}
    topic_values = {}
    for t in topics:
        # 1. 先校验目标轴数据是否有效
        if axis not in data[t] or len(data[t][axis]) == 0:
            print(f"⚠️ {t} 无{axis}轴数据，跳过")
            continue
        if len(data[t][axis]) != len(data[t]["raw_t"]):
            print(f"⚠️ {t} {axis}轴数据长度与时间戳不匹配，跳过")
            continue
        # 2. 筛选时间交集内的原始数据
        mask = (topic_raw_t[t] >= t_start) & (topic_raw_t[t] <= t_end)
        if not np.any(mask):
            print(f"⚠️ {t} 无时间交集内的原始数据，跳过")
            continue
        
        raw_t_filtered = topic_raw_t[t][mask]
        rel_t = raw_t_filtered - t_start  # 转为相对时间（从0开始）
        topic_times[t] = rel_t
        
        # 3. 安全索引（长度已经提前校验，不会再报错）
        val_raw = np.array(data[t][axis])[mask]
        topic_values[t] = val_raw
    valid_topics = [t for t in topics if t in topic_times and len(topic_times[t]) > 0]
    if not valid_topics:
        raise ValueError("所有话题在时间交集内无有效原始数据")
    return topic_times, topic_values, t_start, t_end  # 返回绝对时间的起止

def on_scroll(event):
    ax = event.inaxes
    if not ax: return
    x1,x2 = ax.get_xlim()
    y1,y2 = ax.get_ylim()
    cx, cy = event.xdata, event.ydata
    scale = 0.9 if event.button=="up" else 1.1
    ax.set_xlim(cx-(cx-x1)*scale, cx+(x2-cx)*scale)
    ax.set_ylim(cy-(cy-y1)*scale, cy+(y2-cy)*scale)
    plt.draw()

# ===================== 新增：GUI版本选择对比话题函数 =====================
def select_compare_topics_gui(valid_topics):
    """
    GUI版本选择对比话题，固定第一个选的话题 - 第二个选的话题
    """
    # 创建tkinter顶层窗口（独立于matplotlib）
    root = tk.Tk()
    root.title("选择对比话题")
    root.geometry("450x350")
    root.attributes('-topmost', True)  # 窗口置顶，优先显示
    
    # 存储选中的两个话题
    selected_t1 = tk.StringVar(value="")
    selected_t2 = tk.StringVar(value="")
    
    # 标题标签
    tk.Label(root, text="选择差值对比的两个话题（第一个 - 第二个）", font=("Arial", 12, "bold")).pack(pady=10)
    
    # 第一个话题选择区域
    tk.Label(root, text="1. 选择被减数话题：", font=("Arial", 10)).pack(anchor="w", padx=20)
    listbox1 = Listbox(root, height=6, font=("Arial", 9))
    listbox1.pack(pady=5, fill=tk.X, padx=20)
    for idx, topic in enumerate(valid_topics):
        listbox1.insert(END, f"{idx+1}. {topic}")
    
    # 第二个话题选择区域
    tk.Label(root, text="2. 选择减数话题：", font=("Arial", 10)).pack(anchor="w", padx=20, pady=5)
    listbox2 = Listbox(root, height=6, font=("Arial", 9))
    listbox2.pack(pady=5, fill=tk.X, padx=20)
    for idx, topic in enumerate(valid_topics):
        listbox2.insert(END, f"{idx+1}. {topic}")
    
    # 列表框选择事件处理
    def on_list1_select(event):
        if listbox1.curselection():
            idx = listbox1.curselection()[0]
            selected_t1.set(valid_topics[idx])
    
    def on_list2_select(event):
        if listbox2.curselection():
            idx = listbox2.curselection()[0]
            selected_t2.set(valid_topics[idx])
    
    listbox1.bind('<<ListboxSelect>>', on_list1_select)
    listbox2.bind('<<ListboxSelect>>', on_list2_select)
    
    # 确认按钮回调
    def confirm_selection():
        t1 = selected_t1.get()
        t2 = selected_t2.get()
        
        # 校验选择有效性
        if not t1 or not t2:
            messagebox.showerror("错误", "请完整选择两个话题！")
            return
        if t1 == t2:
            messagebox.showerror("错误", "不能选择相同的话题！")
            return
        
        # 关闭窗口并返回结果
        root.quit()
        root.destroy()
    
    # 确认按钮
    tk.Button(
        root, text="确认选择", command=confirm_selection,
        bg="#4CAF50", fg="white", font=("Arial", 10, "bold"),
        padx=20, pady=5
    ).pack(pady=15)
    
    # 运行tkinter主循环
    root.mainloop()
    
    # 获取最终选择的话题
    topic1 = selected_t1.get()
    topic2 = selected_t2.get()
    
    if not topic1 or not topic2:
        raise ValueError("未选择有效的对比话题")
    
    subtract_order = f"{topic1}-{topic2}"
    print(f"\n✅ 已选择差值对比：{subtract_order}（固定第一个减第二个）")
    return [topic1, topic2], subtract_order

def plot_sensor_comparison(bag_path):
    # 1. 选择要分析的话题
    topics, topic_type_map = get_all_topics_and_select(bag_path)
    
    # 2. 自动过滤话题并选择数据类型
    selected_odom = [t for t in topics if topic_type_map.get(t) == "odom"]
    selected_imu = [t for t in topics if topic_type_map.get(t) == "imu"]
    
    if selected_odom and selected_imu:
        print("\n⚠️ 同时选中了ODOM和IMU类话题，建议分开分析！")
        print(f"ODOM话题：{selected_odom}")
        print(f"IMU话题：{selected_imu}")
        choice = input("请选择分析类型（1=仅ODOM，2=仅IMU）：").strip()
        if choice == "1":
            topics = selected_odom
            print(f"已过滤为仅分析ODOM话题：{topics}")
        else:
            topics = selected_imu
            print(f"已过滤为仅分析IMU话题：{topics}")
    
    # 选择数据子类型（区分ODOM/IMU）
    print("\n请选择数据类型：")
    data_subtype = None
    if selected_odom and topics == selected_odom:
        # ODOM话题可选：位姿、线速度、角速度
        print("1. ODOM/EKF 位姿数据（position/姿态角）")
        print("2. ODOM/EKF 线速度（linear velocity, m/s）")
        print("3. ODOM/EKF 角速度（angular velocity, rad/s）")
        while True:
            data_type_input = input("输入数字1/2/3选择数据类型：").strip()
            if data_type_input == "1":
                data_subtype = "position"
                break
            elif data_type_input == "2":
                data_subtype = "odom_linear_vel"
                break
            elif data_type_input == "3":
                data_subtype = "odom_angular_vel"
                break
            else:
                print("输入错误！只能选1/2/3")
    
    elif selected_imu and topics == selected_imu:
        # IMU话题可选：线加速度、角速度、角加速度
        print("1. IMU 线加速度（linear acceleration, m/s²）")
        print("2. IMU 角速度（angular velocity, rad/s）")
        print("3. IMU 角加速度（angular acceleration, rad/s²）")
        while True:
            data_type_input = input("输入数字1/2/3选择数据类型：").strip()
            if data_type_input == "1":
                data_subtype = "imu_linear_acc"
                break
            elif data_type_input == "2":
                data_subtype = "imu_angular_vel"
                break
            elif data_type_input == "3":
                data_subtype = "imu_angular_acc"
                break
            else:
                print("输入错误！只能选1/2/3")
    else:
        raise ValueError("无有效ODOM/IMU话题可分析")
    
    # 3. 读取传感器数据
    data = read_sensor_data(bag_path, topics, topic_type_map, data_subtype)
    
    # 4. 选择要绘制的轴（根据数据类型限定可选范围）
    print("\n选择绘制轴：")
    data_type = data[list(data.keys())[0]]["type"]
    axis_map = {"r": "roll", "p": "pitch", "h": "yaw"}
    valid_axis = []
    
    if data_type == "position":
        # ODOM位姿：x/y/z/roll/pitch/yaw
        print("x / y / z / r(roll横滚) / p(pitch俯仰) / h(yaw航向)")
        valid_axis = ["x", "y", "z", "roll", "pitch", "yaw"]
    else:
        # ODOM线速度/角速度、IMU所有类型：仅x/y/z
        print("x / y / z")
        valid_axis = ["x", "y", "z"]
    
    axis = input("请输入：").strip().lower()
    # 映射姿态角简写
    if axis in axis_map:
        axis = axis_map[axis]
    # 校验轴合法性
    while axis not in valid_axis:
        axis = input(f"输入错误！只能输入 {valid_axis} 对应选项：").lower()
        if axis in axis_map:
            axis = axis_map[axis]
    
    # 5. 时间对齐（原始时间戳，不插值）
    topic_times, topic_values, abs_t_start, abs_t_end = align_by_raw_timestamp(data, axis)
    rel_t_start = 0.0  # 相对时间起始为0
    rel_t_end = abs_t_end - abs_t_start  # 相对时间结束
    
    # 6. 初始化绘图
    fig, ax = plt.subplots(figsize=(14, 9))
    plt.subplots_adjust(bottom=0.25)  # 底部预留控件空间
    # 设置标题和轴标签
    ylab = ""
    title = ""
    if data_type == "position":
        if axis in ("roll", "pitch", "yaw"):
            ylab = f"{axis} (rad)"
            title = f"多话题{axis}姿态角对比（拖动滑块查看任意时刻）"
        else:
            ylab = f"{axis} (m)"
            title = f"多话题{axis}位置对比（拖动滑块查看任意时刻）"
    elif data_type == "odom_linear_velocity":
        ylab = f"{axis} (m/s)"
        title = f"多话题{axis}ODOM线速度对比（拖动滑块查看任意时刻）"
    elif data_type == "odom_angular_velocity":
        ylab = f"{axis} (rad/s)"
        title = f"多话题{axis}ODOM角速度对比（拖动滑块查看任意时刻）"
    elif data_type == "imu_linear_acceleration":
        ylab = f"{axis} (m/s²)"
        title = f"多话题{axis}IMU线加速度对比（拖动滑块查看任意时刻）"
    elif data_type == "imu_angular_velocity":
        ylab = f"{axis} (rad/s)"
        title = f"多话题{axis}IMU角速度对比（拖动滑块查看任意时刻）"
    elif data_type == "imu_angular_acceleration":
        ylab = f"{axis} (rad/s²)"
        title = f"多话题{axis}IMU角加速度对比（拖动滑块查看任意时刻）"
    
    ax.set_title(title, fontsize=16, pad=15)
    ax.set_xlabel("相对时间 (s)", fontsize=12)
    ax.set_ylabel(ylab, fontsize=12)
    ax.grid(alpha=0.3)
    # 计算全局范围
    all_val = []
    for v in topic_values.values():
        all_val.extend(v)
    all_val = np.array(all_val)
    val_range = all_val.ptp() or 1
    ax.set_ylim(all_val.min() - val_range * margin_ratio, all_val.max() + val_range * margin_ratio)
    
    all_t = []
    for t in topic_times.values():
        all_t.extend(t)
    all_t = np.array(all_t)
    ax.set_xlim(all_t.min() - rel_t_end * margin_ratio, all_t.max() + rel_t_end * margin_ratio)
    # 差异化线条样式
    styles = [('#1f77b4', '-'), ('#ff7f0e', '--'), ('#2ca02c', ':'), ('#d62728', '-.')]
    lines = {}
    markers = {}  # 每个话题的当前点标记
    valid_topics = list(topic_times.keys())
    for i, t in enumerate(valid_topics):
        color, linestyle = styles[i % len(styles)]
        line, = ax.plot([], [], lw=2, label=f"{t}", color=color, linestyle=linestyle)
        marker, = ax.plot([], [], 'o', markersize=5, alpha=0.9, color=color)
        lines[t] = line
        markers[t] = marker
    ax.legend(loc='upper right', fontsize=10)
    
    # -------------------------- 原有D键跟随竖线功能 --------------------------
    # 初始化跟随竖线和文本标签
    vline = ax.axvline(x=0, color='red', linestyle='--', alpha=0.8, linewidth=2, visible=False)
    vline_text = ax.text(
        0, 0, '', color='red', fontsize=12,
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="red", alpha=0.8)
    )
    vline_text.set_visible(False)
    follow_mode = False  # 跟随模式开关

    # ===================== 新增：F键差值对比功能相关变量 =====================
    compare_topics = []  # 存储选中的两个对比话题
    subtract_order = ""  # 减法顺序（固定为"topic1-topic2"）
    diff_follow_mode = False  # F键差值跟随模式
    # 初始化差值竖线和文本
    diff_vline = ax.axvline(x=0, color='purple', linestyle='-.', alpha=0.9, linewidth=2, visible=False)
    diff_vline_text = ax.text(
        0, 0, '', color='purple', fontsize=12,
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="purple", alpha=0.8)
    )
    diff_vline_text.set_visible(False)

    # -------------------------- 按键事件（整合D/F键） --------------------------
    def on_key_press(event):
        nonlocal follow_mode, diff_follow_mode
        # D键：原有跟随模式
        if event.key == 'd':
            follow_mode = not follow_mode
            vline.set_visible(follow_mode)
            vline_text.set_visible(follow_mode)
            fig.canvas.draw_idle()
        # F键：差值对比模式
        elif event.key == 'f':
            if not compare_topics:
                messagebox.showwarning("提示", "请先点击【选择对比话题】按钮选择两个对比话题！")
                return
            diff_follow_mode = not diff_follow_mode
            diff_vline.set_visible(diff_follow_mode)
            diff_vline_text.set_visible(diff_follow_mode)
            fig.canvas.draw_idle()

    # -------------------------- 鼠标移动事件（整合D/F键） --------------------------
    def on_mouse_move(event):
        nonlocal follow_mode, diff_follow_mode
        # 原有D键跟随逻辑
        if follow_mode and event.inaxes == ax:
            x_pos = event.xdata
            if x_pos is None:
                return
            # 更新竖线位置
            vline.set_xdata(x_pos)
            # 更新文本内容和位置
            vline_text.set_text(f"相对时间 = {x_pos:.3f} s")
            vline_text.set_position((x_pos, ax.get_ylim()[1] * 0.95))
            fig.canvas.draw_idle()
        
        # 新增F键差值对比逻辑（固定第一个减第二个）
        if diff_follow_mode and event.inaxes == ax and compare_topics:
            x_pos = event.xdata
            if x_pos is None:
                return
            
            # 获取两个对比话题的时间和值数组
            t1, t2 = compare_topics
            t1_times = topic_times[t1]
            t1_vals = topic_values[t1]
            t2_times = topic_times[t2]
            t2_vals = topic_values[t2]
            
            # 找最近时间戳对应的数值（线性插值）
            def get_nearest_val(times, vals, target_t):
                if target_t < times[0]:
                    return vals[0]
                elif target_t > times[-1]:
                    return vals[-1]
                # 线性插值
                idx = np.searchsorted(times, target_t)
                if idx == 0:
                    return vals[0]
                t_prev, t_next = times[idx-1], times[idx]
                v_prev, v_next = vals[idx-1], vals[idx]
                ratio = (target_t - t_prev) / (t_next - t_prev)
                return v_prev + ratio * (v_next - v_prev)
            
            # 计算两个话题的当前值（固定t1 - t2）
            val1 = get_nearest_val(t1_times, t1_vals, x_pos)
            val2 = get_nearest_val(t2_times, t2_vals, x_pos)
            diff = val1 - val2  # 固定第一个减第二个
            
            # 更新差值竖线位置
            diff_vline.set_xdata(x_pos)
            # 更新差值文本
            text_content = (
                f"相对时间 = {x_pos:.3f} s\n"
                f"{subtract_order} = {diff:.6f}\n"
                f"{t1} = {val1:.6f}\n{t2} = {val2:.6f}"
            )
            diff_vline_text.set_text(text_content)
            # 文本位置（避免超出画布）
            text_y = ax.get_ylim()[1] * 0.85
            diff_vline_text.set_position((x_pos, text_y))
            fig.canvas.draw_idle()

    # -------------------------- 新增：选择对比话题按钮回调（GUI版本） --------------------------
    def on_select_compare_click(event):
        nonlocal compare_topics, subtract_order
        try:
            # 调用GUI版本的话题选择函数
            compare_topics, subtract_order = select_compare_topics_gui(valid_topics)
            messagebox.showinfo("成功", "差值对比话题已选择完成！按F键开启差值跟随模式")
        except Exception as e:
            err_msg = f"选择对比话题失败：{str(e)}"
            print(f"❌ {err_msg}")
            messagebox.showerror("错误", err_msg)

    # -------------------------- 绑定事件 --------------------------
    # 按键和鼠标事件
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
    # 滚轮缩放
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    # -------------------------- 界面控件 --------------------------
    # 状态信息文本
    info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=10,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # 时间轴滑块
    ax_slider = plt.axes([0.15, 0.1, 0.7, 0.03], facecolor='lightgoldenrodyellow')
    time_slider = Slider(
        ax=ax_slider,
        label='时间轴 (s)',
        valmin=rel_t_start,
        valmax=rel_t_end,
        valinit=rel_t_start,
        valstep=SLIDER_VALSTEP,
        valfmt='%.2f s',
        color='#1f77b4'
    )
    time_slider.ax.xaxis.set_minor_locator(plt.MultipleLocator(rel_t_end/10))
    time_slider.ax.grid(alpha=0.3, axis='x')
    
    # 全局状态变量
    is_paused = [True]
    current_t = [rel_t_start]

    # -------------------------- 核心更新函数 --------------------------
    def update_plot(current_time):
        """根据当前时间更新所有话题的绘制"""
        # 遍历所有有效话题更新数据
        for t in valid_topics:
            t_arr = topic_times[t]
            v_arr = topic_values[t]
            
            # 筛选当前时间之前的所有数据
            mask = t_arr <= current_time
            x_data = t_arr[mask]
            y_data = v_arr[mask]
            
            # 更新线条
            lines[t].set_data(x_data, y_data)
            # 更新当前点标记
            if len(x_data) > 0:
                markers[t].set_data(x_data[-1], y_data[-1])
            else:
                markers[t].set_data([], [])
        
        # 更新状态文本
        total_time = rel_t_end - rel_t_start
        drawn_points = {t: len(topic_times[t][topic_times[t] <= current_time]) for t in valid_topics}
        point_info = ", ".join([f"{t}: {drawn_points[t]}" for t in valid_topics])
        info_text.set_text(
            f"当前时间: {current_time:.2f} / {rel_t_end:.2f} s\n"
            f"已绘制点数 - {point_info}"
        )
        fig.canvas.draw_idle()

    # -------------------------- 滑块回调 --------------------------
    def on_slider_change(val, is_user=True):
        current_t[0] = val
        if is_user:
            is_paused[0] = True
            pause_btn.label.set_text("播放")
        update_plot(val)

    def slider_callback_wrapper(val):
        on_slider_change(val, is_user=True)

    time_slider.on_changed(slider_callback_wrapper)

    # -------------------------- 播放/暂停按钮 --------------------------
    def on_pause_click(event):
        is_paused[0] = not is_paused[0]
        pause_btn.label.set_text("暂停" if not is_paused[0] else "播放")

    ax_pause = plt.axes([0.35, 0.02, 0.1, 0.06])
    pause_btn = Button(ax_pause, '播放', color='lightgoldenrodyellow', hovercolor='0.975')
    pause_btn.on_clicked(on_pause_click)

    # -------------------------- 重置按钮 --------------------------
    def on_reset_click(event):
        current_t[0] = rel_t_start
        is_paused[0] = True
        pause_btn.label.set_text("播放")
        time_slider.set_val(rel_t_start)
        update_plot(rel_t_start)

    ax_reset = plt.axes([0.55, 0.02, 0.1, 0.06])
    reset_btn = Button(ax_reset, '重置', color='lightgoldenrodyellow', hovercolor='0.975')
    reset_btn.on_clicked(on_reset_click)

    # ===================== 新增：选择对比话题按钮（GUI版本） =====================
    ax_compare = plt.axes([0.75, 0.02, 0.15, 0.06])
    compare_btn = Button(ax_compare, '选择对比话题', color='lightblue', hovercolor='0.975')
    compare_btn.on_clicked(on_select_compare_click)

    # -------------------------- 自动播放动画 --------------------------
    def animate(frame):
        if not is_paused[0]:
            new_t = current_t[0] + PLAY_SPEED
            if new_t >= rel_t_end:
                new_t = rel_t_end
                is_paused[0] = True
                pause_btn.label.set_text("播放")
            current_t[0] = new_t
            time_slider.set_val(new_t)
            update_plot(new_t)
        return list(lines.values()) + list(markers.values()) + [info_text]

    # 启动动画
    ani = FuncAnimation(
        fig, animate, interval=FRAME_INTERVAL, blit=False, repeat=False
    )

    # 初始化显示
    update_plot(rel_t_start)
    plt.show()

if __name__ == "__main__":
    BAG_PATH = "/home/hou/RC2026/rosbag/bag11/6.bag"
    plot_sensor_comparison(BAG_PATH)