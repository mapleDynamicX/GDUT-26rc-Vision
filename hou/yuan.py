#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rosbag
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import warnings
import sys
import psutil
from matplotlib.widgets import Button, Slider
from math import sqrt
warnings.filterwarnings("ignore")

# ===================== 配置参数 =====================
plt.rcParams["font.family"] = "WenQuanYi Micro Hei"
plt.rcParams["axes.unicode_minus"] = False
margin_ratio = 0.2    # 数据边距比例
ZOOM_FACTOR = 0.1     # 滚轮缩放步长
PLAY_SPEED = 0.05     # 自动播放步长（秒）
FRAME_INTERVAL = 50   # 动画帧间隔（毫秒）
SLIDER_VALSTEP = 0.01 # 时间轴滑块精度（秒）
CIRCLE_POINTS = 200   # 拟合圆绘制的点数
MIN_AXIS_RANGE = 1.0   # 坐标轴最小数据范围（米）
# ====================================================

# -------------------------- 最小二乘法拟合圆 --------------------------
class Circle2D:
    def __init__(self, center=(0,0), radius=0):
        self.center = np.array(center, dtype=np.float64)
        self.radius = float(radius)

def distance_2d(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def fit_circle_least_squares(x, y):
    """
    最小二乘法拟合圆
    圆的一般方程：x² + y² + ax + by + c = 0
    圆心：(-a/2, -b/2)，半径：√((a²+b²)/4 - c)
    """
    # 去除无效值
    valid_mask = np.isfinite(x) & np.isfinite(y)
    x_valid = x[valid_mask]
    y_valid = y[valid_mask]
    n = len(x_valid)
    
    if n < 3:
        raise ValueError("拟合圆至少需要3个有效点")
    
    print(f"  有效点数：{n}")
    
    # 构建线性方程组 Ax = B
    A = np.vstack([x_valid, y_valid, np.ones(n)]).T
    B = -(x_valid**2 + y_valid**2)
    
    # 最小二乘求解
    try:
        a, b, c = np.linalg.lstsq(A, B, rcond=None)[0]
    except np.linalg.LinAlgError as e:
        print(f"  最小二乘求解失败：{e}，使用简化方法")
        # 简化方法：取均值作为圆心，平均距离作为半径
        cx = np.mean(x_valid)
        cy = np.mean(y_valid)
        r = np.mean(np.sqrt((x_valid - cx)**2 + (y_valid - cy)**2))
        return Circle2D((cx, cy), r), np.nan
    
    # 计算圆心和半径
    cx = -a / 2.0
    cy = -b / 2.0
    radius_sq = (a**2 + b**2) / 4.0 - c
    
    # 处理半径为负的情况（拟合失败）
    if radius_sq < 0:
        print(f"  拟合半径平方为负({radius_sq:.6f})，使用简化方法")
        cx = np.mean(x_valid)
        cy = np.mean(y_valid)
        r = np.mean(np.sqrt((x_valid - cx)**2 + (y_valid - cy)**2))
        circle = Circle2D((cx, cy), r)
    else:
        r = np.sqrt(radius_sq)
        circle = Circle2D((cx, cy), r)
    
    # 计算拟合误差（均方误差MSE）
    distances = np.sqrt((x_valid - cx)**2 + (y_valid - cy)**2)
    mse = np.mean((distances - r)**2)
    rmse = np.sqrt(mse)
    
    print(f"  拟合结果：圆心({cx:.6f}, {cy:.6f})，半径{r:.6f}m")
    print(f"  拟合误差：均方误差(MSE)={mse:.6f}，均方根误差(RMSE)={rmse:.6f}m")
    
    return circle, rmse

def generate_circle_2d(center, radius, n_points=CIRCLE_POINTS):
    """生成圆的采样点用于绘图"""
    theta = np.linspace(0, 2*np.pi, n_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    return x, y

# -------------------------- 话题选择 & 数据读取 --------------------------
def select_point_fast_topics(bag_path):
    bag = rosbag.Bag(bag_path)
    all_topics = list(bag.get_type_and_topic_info()[1].keys())
    bag.close()
    print("="*50)
    print(f"检测到bag中所有话题（共{len(all_topics)}个）：")
    for idx, topic in enumerate(all_topics, 1):
        print(f"{idx}. {topic}")
    print("="*50)
    print("\n--- 选择第一个路径话题 ---")
    while True:
        idx = input(f"输入话题序号（1~{len(all_topics)}）：").strip()
        if idx.isdigit() and 1<=int(idx)<=len(all_topics):
            point_topic = all_topics[int(idx)-1]
            break
        print("输入无效！")
    print("\n--- 选择第二个路径话题 ---")
    while True:
        idx = input(f"输入话题序号（1~{len(all_topics)}）：").strip()
        if idx.isdigit() and 1<=int(idx)<=len(all_topics):
            fast_topic = all_topics[int(idx)-1]
            break
        print("输入无效！")
    print(f"\n已选话题：\n路径1：{point_topic}\n路径2：{fast_topic}")
    print("="*50)
    return point_topic, fast_topic

def read_and_align_xyz_data(bag_path, point_topic, fast_topic):
    # 新增z轴数据存储
    raw_data = {"point": {"x": [], "y": [], "z": [], "t": []}, 
                "fast": {"x": [], "y": [], "z": [], "t": []}}
    bag = rosbag.Bag(bag_path)
    for topic, msg, _ in bag.read_messages(topics=[point_topic]):
        try:
            ts = msg.header.stamp.to_sec()
            raw_data["point"]["t"].append(ts)
            raw_data["point"]["x"].append(msg.pose.pose.position.x)
            raw_data["point"]["y"].append(msg.pose.pose.position.y)
            raw_data["point"]["z"].append(msg.pose.pose.position.z)  # 读取z值
        except AttributeError: continue
    for topic, msg, _ in bag.read_messages(topics=[fast_topic]):
        try:
            ts = msg.header.stamp.to_sec()
            raw_data["fast"]["t"].append(ts)
            raw_data["fast"]["x"].append(msg.pose.pose.position.x)
            raw_data["fast"]["y"].append(msg.pose.pose.position.y)
            raw_data["fast"]["z"].append(msg.pose.pose.position.z)  # 读取z值
        except AttributeError: continue
    bag.close()

    for key in ["point", "fast"]:
        raw_data[key]["t"] = np.array(raw_data[key]["t"])
        raw_data[key]["x"] = np.array(raw_data[key]["x"])
        raw_data[key]["y"] = np.array(raw_data[key]["y"])
        raw_data[key]["z"] = np.array(raw_data[key]["z"])  # 转为numpy数组
        sort_idx = np.argsort(raw_data[key]["t"])
        raw_data[key]["t"] = raw_data[key]["t"][sort_idx].copy()
        raw_data[key]["x"] = raw_data[key]["x"][sort_idx].copy()
        raw_data[key]["y"] = raw_data[key]["y"][sort_idx].copy()
        raw_data[key]["z"] = raw_data[key]["z"][sort_idx].copy()  # 按时间排序z值

    if len(raw_data["point"]["t"]) == 0: raise ValueError("路径1话题未读取到有效数据！")
    if len(raw_data["fast"]["t"]) == 0: raise ValueError("路径2话题未读取到有效数据！")
    t_start = max(raw_data["point"]["t"][0], raw_data["fast"]["t"][0])
    t_end = min(raw_data["point"]["t"][-1], raw_data["fast"]["t"][-1])
    print(f"\n原始时间范围：\n路径1: {raw_data['point']['t'][0]:.3f} ~ {raw_data['point']['t'][-1]:.3f} s\n路径2: {raw_data['fast']['t'][0]:.3f} ~ {raw_data['fast']['t'][-1]:.3f} s")
    print(f"重叠时间范围：{t_start:.3f} ~ {t_end:.3f} s (总时长 {t_end-t_start:.3f} s)")
    if t_end - t_start < 0.1: raise ValueError("两个话题时间重叠不足0.1秒，无法对比！")

    aligned_data = {}
    for key in ["point", "fast"]:
        mask = (raw_data[key]["t"] >= t_start) & (raw_data[key]["t"] <= t_end)
        aligned_data[key] = {
            "t": raw_data[key]["t"][mask].copy(),
            "x": raw_data[key]["x"][mask].copy(),
            "y": raw_data[key]["y"][mask].copy(),
            "z": raw_data[key]["z"][mask].copy()  # 对齐后的z值
        }
    print(f"\n数据对齐完成：\n有效点数 - 路径1: {len(aligned_data['point']['t'])}, 路径2: {len(aligned_data['fast']['t'])}")
    
    # 计算并输出X/Y/Z相关统计（原有X/Y + 新增Z）
    # 路径1统计
    n1 = min(10, len(aligned_data['point']['x']))
    avg_x1 = np.mean(aligned_data['point']['x'][:n1])
    avg_y1 = np.mean(aligned_data['point']['y'][:n1])
    total_avg_z1 = np.mean(aligned_data['point']['z'])  # 总平均Z值
    top10_avg_z1 = np.mean(aligned_data['point']['z'][:n1])  # 前10个点平均Z值
    
    # 路径2统计
    n2 = min(10, len(aligned_data['fast']['x']))
    avg_x2 = np.mean(aligned_data['fast']['x'][:n2])
    avg_y2 = np.mean(aligned_data['fast']['y'][:n2])
    total_avg_z2 = np.mean(aligned_data['fast']['z'])  # 总平均Z值
    top10_avg_z2 = np.mean(aligned_data['fast']['z'][:n2])  # 前10个点平均Z值
    
    # 打印统计信息
    print(f"路径1前{n1}个点的平均坐标：x={avg_x1:.6f}, y={avg_y1:.6f}")
    print(f"路径1 - 总平均Z值：{total_avg_z1:.6f} | 前{n1}个点平均Z值：{top10_avg_z1:.6f}")
    print(f"路径2前{n2}个点的平均坐标：x={avg_x2:.6f}, y={avg_y2:.6f}")
    print(f"路径2 - 总平均Z值：{total_avg_z2:.6f} | 前{n2}个点平均Z值：{top10_avg_z2:.6f}")
    
    return aligned_data, t_start, t_end

# -------------------------- 2D绘图 & 交互 --------------------------
def plot_2d_with_time_slider(aligned_data, t_start, t_end, topic1, topic2):
    # 原始全量数据（用于拟合圆）
    x1_full, y1_full = aligned_data["point"]["x"], aligned_data["point"]["y"]
    circle1, rmse1 = fit_circle_least_squares(x1_full, y1_full)
    
    x2_full, y2_full = aligned_data["fast"]["x"], aligned_data["fast"]["y"]
    circle2, rmse2 = fit_circle_least_squares(x2_full, y2_full)

    # 生成拟合圆的绘制点
    circle_x1, circle_y1 = generate_circle_2d(circle1.center, circle1.radius)
    circle_x2, circle_y2 = generate_circle_2d(circle2.center, circle2.radius)

    # 创建画布
    fig = plt.figure(figsize=(18, 8))
    # 自动激活平移（pan）工具
    fig.canvas.toolbar.pan()
    plt.subplots_adjust(left=0.06, right=0.96, bottom=0.25)
    ax = fig.add_subplot(111)

    # 初始化绘制对象（路径初始为空，随时间轴更新）
    line1, = ax.plot([], [], 'b-', label=f"路径1: {topic1}", lw=2, alpha=0.9)
    line2, = ax.plot([], [], 'r--', label=f"路径2: {topic2}", lw=2, alpha=0.9)
    marker1, = ax.plot([], [], 'bo', markersize=8, alpha=1.0, label="路径1当前点")
    marker2, = ax.plot([], [], 'ro', markersize=8, alpha=1.0, label="路径2当前点")
    # 拟合圆始终显示全量
    circle_line1, = ax.plot(circle_x1, circle_y1, 'c-', label=f"路径1-最小二乘圆", lw=2, alpha=0.7)
    circle_line2, = ax.plot(circle_x2, circle_y2, 'm-', label=f"路径2-最小二乘圆", lw=2, alpha=0.7)
    center1_marker, = ax.plot([circle1.center[0]], [circle1.center[1]], 'b*', markersize=14, label="路径1-圆心", alpha=1.0)
    center2_marker, = ax.plot([circle2.center[0]], [circle2.center[1]], 'r*', markersize=14, label="路径2-圆心", alpha=1.0)

    # 设置图表属性
    ax.set_title("二维双路径对比（XY维度最小二乘圆拟合）", fontsize=16, pad=20)
    ax.set_xlabel("X (m)", fontsize=12, labelpad=10)
    ax.set_ylabel("Y (m)", fontsize=12, labelpad=10)
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')  # 保持XY等比例

    # 计算坐标轴范围（基于全量数据）
    all_x = np.concatenate([x1_full, y1_full, circle_x1, circle_x2])
    all_y = np.concatenate([y1_full, y2_full, circle_y1, circle_y2])
    all_x = all_x[np.isfinite(all_x)]
    all_y = all_y[np.isfinite(all_y)]
    x_range = max(all_x.max() - all_x.min(), MIN_AXIS_RANGE)
    y_range = max(all_y.max() - all_y.min(), MIN_AXIS_RANGE)
    x_center = (all_x.max() + all_x.min()) / 2
    y_center = (all_y.max() + all_y.min()) / 2
    max_range = max(x_range, y_range) * (1 + margin_ratio)
    ax.set_xlim(x_center - max_range/2, x_center + max_range/2)
    ax.set_ylim(y_center - max_range/2, y_center + max_range/2)

    # 信息文本框（和yuan.py同款）
    info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=10,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # 时间滑块（和yuan.py同款位置/样式）
    ax_slider = plt.axes([0.15, 0.1, 0.7, 0.03], facecolor='lightgoldenrodyellow')
    time_slider = Slider(
        ax=ax_slider,
        label='时间轴 (s)',
        valmin=t_start,
        valmax=t_end,
        valinit=t_start,
        valstep=SLIDER_VALSTEP,
        valfmt='%.2f s',
        color='#1f77b4'
    )
    time_slider.ax.xaxis.set_minor_locator(plt.MultipleLocator((t_end-t_start)/10))
    time_slider.ax.grid(alpha=0.3, axis='x')

    is_paused = [True]
    current_t = [t_start]

    # 核心修改：和yuan.py同款的update_path函数
    def update_path(current_time):
        """根据当前时间更新整条路径绘制（而非仅更新当前点）"""
        # 筛选当前时间之前的所有数据
        mask1 = aligned_data["point"]["t"] <= current_time
        x1 = aligned_data["point"]["x"][mask1]
        y1 = aligned_data["point"]["y"][mask1]
        
        mask2 = aligned_data["fast"]["t"] <= current_time
        x2 = aligned_data["fast"]["x"][mask2]
        y2 = aligned_data["fast"]["y"][mask2]

        # 更新整条路径
        line1.set_data(x1, y1)
        line2.set_data(x2, y2)
        
        # 更新当前点
        if len(x1) > 0:
            marker1.set_data(x1[-1], y1[-1])
        if len(x2) > 0:
            marker2.set_data(x2[-1], y2[-1])
        
        # 更新信息文本（和yuan.py同款）
        info_text.set_text(
            f"当前时间: {current_time - t_start:.2f} / {t_end - t_start:.2f} s\n"
            f"已绘制点数 - 路径1: {len(x1)}, 路径2: {len(x2)}"
        )
        fig.canvas.draw_idle()

    # 滑块回调（和yuan.py同款）
    def on_slider_change(val):
        """拖动滑块时更新整条路径"""
        current_t[0] = val
        is_paused[0] = True
        pause_btn.label.set_text("播放")
        update_path(val)
    time_slider.on_changed(on_slider_change)

    # 播放/暂停按钮（和yuan.py同款位置）
    def on_pause_click(event):
        """播放/暂停按钮回调"""
        is_paused[0] = not is_paused[0]
        pause_btn.label.set_text("暂停" if not is_paused[0] else "播放")
    
    ax_pause = plt.axes([0.35, 0.02, 0.1, 0.06])
    pause_btn = Button(ax_pause, '播放', color='lightgoldenrodyellow', hovercolor='0.975')
    pause_btn.on_clicked(on_pause_click)

    # 重置按钮（和yuan.py同款位置）
    def on_reset_click(event):
        """重置按钮回调"""
        current_t[0] = t_start
        is_paused[0] = True
        pause_btn.label.set_text("播放")
        time_slider.set_val(t_start)
        update_path(t_start)
    
    ax_reset = plt.axes([0.55, 0.02, 0.1, 0.06])
    reset_btn = Button(ax_reset, '重置', color='lightgoldenrodyellow', hovercolor='0.975')
    reset_btn.on_clicked(on_reset_click)

    # ========== 新增：拟合圆显示/隐藏切换按钮 ==========
    circle_visible = [True]  # 拟合圆可见状态（用列表实现可变对象）
    def on_circle_toggle_click(event):
        """切换拟合圆和圆心标记的显示/隐藏"""
        # 切换可见状态
        circle_visible[0] = not circle_visible[0]
        # 更新拟合圆绘制对象的可见性
        circle_line1.set_visible(circle_visible[0])
        circle_line2.set_visible(circle_visible[0])
        # 更新圆心标记的可见性
        center1_marker.set_visible(circle_visible[0])
        center2_marker.set_visible(circle_visible[0])
        # 更新按钮文本
        circle_toggle_btn.label.set_text("显示拟合圆" if not circle_visible[0] else "隐藏拟合圆")
        # 刷新画布
        fig.canvas.draw_idle()
    
    # 创建切换按钮（位置在重置按钮右侧）
    ax_circle_toggle = plt.axes([0.75, 0.02, 0.1, 0.06])
    circle_toggle_btn = Button(ax_circle_toggle, '隐藏拟合圆', 
                               color='lightgoldenrodyellow', hovercolor='0.975')
    circle_toggle_btn.on_clicked(on_circle_toggle_click)
    # ==================================================

    # 滚轮缩放（保留原有逻辑）
    def on_scroll(event):
        """滚轮缩放回调"""
        if not event.inaxes or event.inaxes != ax:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        scale = 1 - ZOOM_FACTOR if event.button == "up" else 1 + ZOOM_FACTOR
        x_lim = ax.get_xlim()
        y_lim = ax.get_ylim()
        new_xlim = [x - (x - x_lim[0]) * scale, x + (x_lim[1] - x) * scale]
        new_ylim = [y - (y - y_lim[0]) * scale, y + (y_lim[1] - y) * scale]
        ax.set_xlim(new_xlim)
        ax.set_ylim(new_ylim)
        fig.canvas.draw_idle()
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    # 动画函数（和yuan.py同款逻辑）
    def animate(frame):
        """动画更新函数"""
        if not is_paused[0]:
            current_t[0] += PLAY_SPEED
            if current_t[0] >= t_end:
                current_t[0] = t_end
                is_paused[0] = True
                pause_btn.label.set_text("播放")
            time_slider.set_val(current_t[0])
            update_path(current_t[0])
        return line1, line2, marker1, marker2, info_text, center1_marker, center2_marker, circle_line1, circle_line2

    # 启动动画
    ani = animation.FuncAnimation(fig, animate, interval=FRAME_INTERVAL, blit=False, repeat=False)
    update_path(t_start)  # 初始化显示
    plt.show()

def main(bag_path):
    try:
        point_topic, fast_topic = select_point_fast_topics(bag_path)
        aligned_data, t_start, t_end = read_and_align_xyz_data(bag_path, point_topic, fast_topic)
        plot_2d_with_time_slider(aligned_data, t_start, t_end, point_topic, fast_topic)
    except Exception as e:
        print(f"\n程序异常终止：{e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    # 修改为你的ROS bag文件路径
    BAG_PATH = "/home/hou/RC2026/rosbag/bag12/1.bag"
    main(BAG_PATH)