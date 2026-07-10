#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
# 在 import rospy 之前强行指定 ROS 日志配置文件路径
if 'ROS_ETC_DIR' not in os.environ:
    os.environ['ROS_ETC_DIR'] = '/opt/ros/noetic/etc/ros'
import rospy
import tkinter as tk
import json
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Float32MultiArray

class MonitorStandalone:
    def __init__(self, master):
        self.master = master
        self.master.title("数据监视区")
        # 窗口尺寸逻辑与原代码保持一致
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()
        win_width = int(screen_width * 0.4)
        win_height = int(screen_height * 0.4)
        x_coordinate = int((screen_width - win_width) / 2)
        y_coordinate = int((screen_height - win_height) / 2)
        self.master.geometry(f"{win_width}x{win_height}+{x_coordinate}+{y_coordinate}")
        self.master.configure(bg='#1e1e1e')

        # 初始化 ROS 节点
        rospy.init_node('monitor_only_node', anonymous=True)
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # ========== 原代码：监视区全部变量 一字未改 ==========
        self.monitor_config_path = r"./monitor_config.json"
        self.monitor_meta = []
        self.monitor_subs = []  # 存储 ROS 订阅者句柄，防止被垃圾回收
        self.monitor_vars = {}  # 存储话题名称对应的 StringVar
        self.monitor_labels = {}  # 存储 Label 引用，用于修改颜色
        self.monitor_last_update = {}  # 存储各个话题最后一次收到数据的时间戳
        self.monitor_precision = 3  # 默认显示精度（小数点后2位）
        self.precision_btns = {}  # 存储精度选择按钮句柄
        self.monitor_last_process_time = {}  # 存储每个话题上一次真正处理/更新的时间戳

        # ========== 新增：上位机自定义数据排序控制变量 ==========
        # 填写 JSON 中的 name，作为默认显示的顺序，例如 ["Odom数据", "IMU数据"]。若不填或不匹配则跳过（默认全不选）
        self.default_upper_display_order = ["r2/odom", "r1/odom"]
        self.current_upper_display = []
        self.upper_sel_blocks = {}
        self.upper_sel_seq_labels = {}
        self.valid_upper_items = {}
        self.upper_subs = []  # 专用于管理上位机动态生成的订阅者句柄，以便重排时安全注销

        # ========== 新增：下位机相关变量与配置 ==========
        self.lower_topic = "/scripts/liao"  # 下位机数组话题名，可按需修改
        self.lower_names = ["X", "Y", "Yaw", "激光1", "激光2"]  # 数据名字映射list，可按需增删
        self.lower_vars = {}
        self.lower_labels = {}
        self.lower_last_update = 0.0
        self.lower_precision = 2
        self.lower_timeout = 1.5
        self.lower_block_widgets = []

        # 创建顶部切换栏
        self.build_switch_buttons()

        # 创建两套界面的独立容器
        self.upper_container = tk.Frame(self.master, bg='#1e1e1e')
        self.lower_container = tk.Frame(self.master, bg='#1e1e1e')

        # 分别构建两套界面的内容
        self._load_monitor_json()
        self.build_monitor_block_content(self.upper_container)  # 传入上位机容器
        self.build_lower_block_content(self.lower_container)  # 传入下位机容器

        # 默认展示上位机界面
        self.upper_container.pack(fill=tk.BOTH, expand=True)
        self._check_monitor_timeout()  # 启动颜色监控循环定时器

    def update_ros_status(self, status_type, text, color):
        """
        统一更新指定类型的ROS状态显示（多页面同步）
        :param status_type: path / param / lidar / common
        :param text: 显示文本
        :param color: 字体颜色：成功#50fa7b / 警告#f1fa8c / 错误#ff5555
        """
        if status_type not in self.ros_status_vars:
            return
        self.ros_status_vars[status_type].set(text)
        # 同步更新所有页面的对应标签颜色
        for lbl in self.ros_status_labels[status_type]:
            lbl.config(fg=color)
    # ==================================================================

    # ===================== 原代码：监视区所有方法 一字未改 =====================
    def _load_monitor_json(self):
        """读取监视区配置 JSON"""
        if not os.path.exists(self.monitor_config_path):
            rospy.logwarn(f"监视区配置文件不存在：{self.monitor_config_path}")
            return
        try:
            with open(self.monitor_config_path, "r", encoding="utf-8") as f:
                self.monitor_meta = json.load(f)
        except Exception as e:
            rospy.logerr(f"加载监视区json失败：{str(e)}")

    def build_monitor_block_content(self, parent):
        """监视区页面：顶部常驻ROS状态栏，下方流式布局显示块"""
        # # === 新增：精度控制行 ===
        # precision_frame = tk.Frame(parent, bg="#2d2d2d")
        # precision_frame.pack(fill=tk.X, padx=10)
        # # 靠左写文字“精度：”
        # tk.Label(precision_frame, text="精度：", bg="#2d2d2d", fg="white", font=("DejaVu Sans", 40, "bold")).pack(
        #     side=tk.LEFT)
        # # 中间放置精度选择按钮
        # precisions = ["2", "3", "4", "5"]
        # for p in precisions:
        #     btn = tk.Button(
        #         precision_frame, text=f".{p}", font=("DejaVu Sans", 45, "bold"),
        #         bg="#44475a", fg="white", width=4,
        #         command=lambda val=p: self._change_precision(val)
        #     )
        #     btn.pack(side=tk.LEFT, padx=5)
        #     self.precision_btns[p] = btn
        # # 最后放一个字符 f
        # tk.Label(precision_frame, text="f", bg="#2d2d2d", fg="white", font=("DejaVu Sans", 40, "bold")).pack(
        #     side=tk.LEFT, padx=2)
        # # 默认激活选中的高亮样式
        # self._change_precision(f"{self.monitor_precision}")
        # =======================

        # 提取JSON中支持的话题项目并建立映射字典
        self.valid_upper_items = {}
        valid_names = []
        for item in self.monitor_meta:
            if item.get("type") in ["nav_msgs/Odometry", "sensor_msgs/Imu"]:
                name = item["name"]
                self.valid_upper_items[name] = item
                valid_names.append(name)

        # --- 1. 新增：顶部数据显示控制栏 ---
        selection_frame = tk.Frame(parent, bg="#2d2d2d")
        selection_frame.pack(fill=tk.X, padx=2, pady=2)

        # 一行平分成三个
        for i in range(3):
            selection_frame.columnconfigure(i, weight=1, uniform="upper_sel_cols")

        for idx, name in enumerate(valid_names):
            block = tk.Frame(selection_frame, bg="#44475a", bd=1, relief=tk.RAISED)
            block.grid(row=idx // 3, column=idx % 3, sticky="ew", padx=1, pady=1)

            name_lbl = tk.Label(block, text=name, bg="#44475a", fg="white", font=("DejaVu Sans", 10, "bold"), height=2)
            name_lbl.pack(side=tk.LEFT, padx=1, pady=1)

            # 预留序号位置
            seq_lbl = tk.Label(block, text="", bg="#44475a", fg="#ffb86c", font=("DejaVu Sans", 11, "bold"))
            seq_lbl.pack(side=tk.RIGHT, padx=1, pady=1)

            self.upper_sel_blocks[name] = block
            self.upper_sel_seq_labels[name] = seq_lbl

            # 绑定点击事件
            def make_cb(n):
                return lambda e, n=n: self._toggle_upper_selection(n)

            block.bind("<Button-1>", make_cb(name))
            name_lbl.bind("<Button-1>", make_cb(name))
            seq_lbl.bind("<Button-1>", make_cb(name))

        # 初始化显示列表：检测是否有默认排序变量
        if hasattr(self, 'default_upper_display_order') and isinstance(self.default_upper_display_order, list):
            self.current_upper_display = [n for n in self.default_upper_display_order if n in valid_names]
        else:
            self.current_upper_display = []

        # --- 2. 原有：下方流式数据显示区 ---
        wrap_frame = tk.Frame(parent, bg="#2d2d2d")
        wrap_frame.pack(fill=tk.BOTH, expand=True, padx=2)

        scrollbar = tk.Scrollbar(wrap_frame, orient=tk.VERTICAL, width=30)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.monitor_canvas = tk.Canvas(wrap_frame, bg="#2d2d2d", highlightthickness=0, yscrollcommand=scrollbar.set)
        self.monitor_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.monitor_canvas.yview)

        self.monitor_flow_frame = tk.Frame(self.monitor_canvas, bg="#2d2d2d")
        self.monitor_flow_window = self.monitor_canvas.create_window((0, 0), window=self.monitor_flow_frame,
                                                                     anchor="nw")

        self.monitor_block_widgets = []

        self.monitor_canvas.bind("<Configure>", self._on_monitor_canvas_resize)
        self.monitor_flow_frame.bind(
            "<Configure>",
            lambda e: self.monitor_canvas.config(scrollregion=self.monitor_canvas.bbox("all"))
        )
        self.monitor_canvas.bind("<Button-4>", lambda e: self.monitor_canvas.yview_scroll(-1, "units"))
        self.monitor_canvas.bind("<Button-5>", lambda e: self.monitor_canvas.yview_scroll(1, "units"))

        # 根据初始状态生成界面
        self._update_upper_selection_ui()
        self._update_upper_data_display_ui()

    # --- 3. 新增：上位机选择交互逻辑与重绘方法 ---
    def _toggle_upper_selection(self, name):
        """处理上位机控制块的点击选中/取消选中逻辑"""
        if name in self.current_upper_display:
            self.current_upper_display.remove(name)
        else:
            self.current_upper_display.append(name)

        self._update_upper_selection_ui()
        self._update_upper_data_display_ui()

    def _update_upper_selection_ui(self):
        """更新控制栏块的高亮和显示序号"""
        for name, block in self.upper_sel_blocks.items():
            name_lbl = block.winfo_children()[0]
            seq_lbl = self.upper_sel_seq_labels[name]

            if name in self.current_upper_display:
                idx = self.current_upper_display.index(name) + 1
                block.config(bg="#6272a4")
                name_lbl.config(bg="#6272a4")
                seq_lbl.config(text=str(idx), bg="#6272a4")
            else:
                block.config(bg="#44475a")
                name_lbl.config(bg="#44475a")
                seq_lbl.config(text="", bg="#44475a")

    def _update_upper_data_display_ui(self):
        """重新生成上位机实际显示的数据块并管理 ROS 订阅"""
        # 清空现有显示的块
        for widget in self.monitor_block_widgets:
            widget.destroy()
        self.monitor_block_widgets.clear()

        # 必须注销旧的订阅，防止后台冗余订阅互相抢占与冲突
        for sub in self.upper_subs:
            sub.unregister()
        self.upper_subs.clear()

        self.monitor_vars.clear()
        self.monitor_labels.clear()
        self.monitor_last_update.clear()
        self.monitor_last_process_time.clear()

        # 按当前记录的顺序进行 UI 与 ROS订阅重建
        for name in self.current_upper_display:
            item = self.valid_upper_items[name]
            block = self._create_single_monitor_widget(self.monitor_flow_frame, item)
            self.monitor_block_widgets.append(block)

        self.master.update_idletasks()
        self._on_monitor_canvas_resize()

    def build_switch_buttons(self):
        """在顶层构建上位机/下位机切换按钮栏"""
        switch_frame = tk.Frame(self.master, bg="#2d2d2d")
        switch_frame.pack(fill=tk.X, padx=2, pady=2)

        self.btn_upper = tk.Button(
            switch_frame, text="上位机", font=("DejaVu Sans", 20, "bold"),
            bg="white", fg="black", width=10, command=lambda: self.toggle_interface("upper")
        )
        self.btn_upper.pack(side=tk.LEFT, padx=1, pady=1)

        self.btn_lower = tk.Button(
            switch_frame, text="下位机", font=("DejaVu Sans", 20, "bold"),
            bg="#44475a", fg="white", width=10, command=lambda: self.toggle_interface("lower")
        )
        self.btn_lower.pack(side=tk.LEFT, padx=1, pady=1)

    def toggle_interface(self, target):
        """切换界面容器的显示与隐藏"""
        if target == "upper":
            self.lower_container.pack_forget()
            self.upper_container.pack(fill=tk.BOTH, expand=True)
            self.btn_upper.config(bg="white", fg="black")
            self.btn_lower.config(bg="#44475a", fg="white")
        else:
            self.upper_container.pack_forget()
            self.lower_container.pack(fill=tk.BOTH, expand=True)
            self.btn_lower.config(bg="white", fg="black")
            self.btn_upper.config(bg="#44475a", fg="white")

    def build_lower_block_content(self, parent):
        """构建下位机专属流式布局界面（保持样式完全一致）"""
        wrap_frame = tk.Frame(parent, bg="#2d2d2d")
        wrap_frame.pack(fill=tk.BOTH, expand=True, padx=2)

        scrollbar = tk.Scrollbar(wrap_frame, orient=tk.VERTICAL, width=30)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.lower_canvas = tk.Canvas(wrap_frame, bg="#2d2d2d", highlightthickness=0, yscrollcommand=scrollbar.set)
        self.lower_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.lower_canvas.yview)

        self.lower_flow_frame = tk.Frame(self.lower_canvas, bg="#2d2d2d")
        self.lower_flow_window = self.lower_canvas.create_window((0, 0), window=self.lower_flow_frame, anchor="nw")

        # 根据配置的名字映射 list，动态生成与上位机完全一致的 UI 块
        for name in self.lower_names:
            block = self._create_single_lower_widget(self.lower_flow_frame, name)
            self.lower_block_widgets.append(block)

        self.lower_canvas.bind("<Configure>", self._on_lower_canvas_resize)
        self.lower_flow_frame.bind(
            "<Configure>",
            lambda e: self.lower_canvas.config(scrollregion=self.lower_canvas.bbox("all"))
        )
        self.lower_canvas.bind("<Button-4>", lambda e: self.lower_canvas.yview_scroll(-1, "units"))
        self.lower_canvas.bind("<Button-5>", lambda e: self.lower_canvas.yview_scroll(1, "units"))

        # 订阅下位机数组话题
        sub = rospy.Subscriber(self.lower_topic, Float32MultiArray, self._lower_topic_cb, queue_size=1)
        self.monitor_subs.append(sub)

    def _create_single_lower_widget(self, parent_container, name):
        """渲染单个下位机数据块"""
        block = tk.Frame(parent_container, bg="#383838", bd=2, relief=tk.RAISED, height=150)
        block.pack_propagate(False)

        tk.Label(block, text=name, bg="#383838", fg="white", font=("DejaVu Sans", 10, "bold")).pack(anchor="w", padx=2,
                                                                                                    pady=2)

        data_var = tk.StringVar(value="等待数据...")
        self.lower_vars[name] = data_var

        data_label = tk.Label(
            block, textvariable=data_var, bg="#282a36", fg="white",
            font=("DejaVu Sans", 20, "bold"), justify=tk.LEFT, anchor="nw"
        )
        data_label.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        self.lower_labels[name] = data_label

        def _scroll(e):
            self.lower_canvas.yview_scroll(-1 if e.num == 4 else 1, "units")

        block.bind("<Button-4>", _scroll)
        block.bind("<Button-5>", _scroll)
        for child in block.winfo_children():
            child.bind("<Button-4>", _scroll)
            child.bind("<Button-5>", _scroll)
        return block

    def _lower_topic_cb(self, msg):
        """下位机数组话题回调函数"""
        self.lower_last_update = time.time()
        prec = self.lower_precision
        # 遍历收到的数组，若多出位自动忽略（通过 zip 截断处理）
        for val, name in zip(msg.data, self.lower_names):
            self.lower_vars[name].set(f"{val:.{prec}f}")

    def _on_lower_canvas_resize(self, event):
        """下位机画布自适应两等分布局逻辑"""
        canvas_width = event.width
        if canvas_width <= 1: return
        cols = 2
        for i in range(cols):
            self.lower_flow_frame.columnconfigure(i, weight=1, uniform="lower_cols")
        new_block_width = max(150, (canvas_width // cols) - 20)
        for widget in self.lower_block_widgets:
            widget.grid_forget()
        for idx, widget in enumerate(self.lower_block_widgets):
            widget.config(width=new_block_width)
            widget.grid(row=idx // cols, column=idx % cols, pady=10, sticky="n")
        self.lower_canvas.itemconfig(self.lower_flow_window, width=canvas_width)

    def _create_single_monitor_widget(self, parent_container, item):
        """单个监视UI块与ROS订阅逻辑"""
        topic_name = item["topic"]
        msg_type = item["type"]
        # 1. 从 JSON 中读取设置的采样频率，若不存在则默认 30Hz
        freq = item.get("frequency", 30.0)
        block = tk.Frame(parent_container, bg="#383838", bd=2, relief=tk.RAISED, height=200)
        block.pack_propagate(False)
        tk.Label(block, text=item["name"], bg="#383838", fg="white", font=("DejaVu Sans", 10, "bold")).pack(anchor="w",
                                                                                                            padx=2,
                                                                                                            pady=2)
        data_var = tk.StringVar(value="等待数据接入...")
        self.monitor_vars[topic_name] = data_var
        data_label = tk.Label(
            block, textvariable=data_var, bg="#282a36", fg="white",
            font=("DejaVu Sans", 15, "bold"), justify=tk.LEFT, anchor="nw"
        )
        data_label.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self.monitor_labels[topic_name] = data_label
        self.monitor_last_update[topic_name] = 0.0
        self.monitor_last_process_time[topic_name] = 0.0  # 初始化单话题处理时间
        # 2. 带有频率控制与动态精度的 Odometry 回调
        if msg_type == "nav_msgs/Odometry":
            def odom_cb(msg, var=data_var, top=topic_name, f=freq):
                current_time = time.time()
                # ====== 升级版防僵尸保护：如果自己不是当前激活的UI变量，直接拦截 ======
                if top not in self.monitor_vars or self.monitor_vars[top] != var:
                    return
                # ===============================================================
                # 按照频率周期拦截采样 (周期 = 1.0 / 频率)
                if current_time - self.monitor_last_process_time[top] < (1.0 / f):
                    return
                pos = msg.pose.pose.position
                q = msg.pose.pose.orientation
                r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
                # 获取当前选中的精度
                prec = self.monitor_precision
                text = (f"XYZ: {pos.x:.{prec}f}, {pos.y:.{prec}f}, {pos.z:.{prec}f}\n"
                        f"RPY: {math.degrees(r):.{prec}f}°, {math.degrees(p):.{prec}f}°, {math.degrees(y):.{prec}f}°")
                var.set(text)
                self.monitor_last_update[top] = current_time
                self.monitor_last_process_time[top] = current_time
            sub = rospy.Subscriber(topic_name, Odometry, odom_cb, queue_size=1)
            self.upper_subs.append(sub)
        # 3. 带有频率控制与动态精度的 Imu 回调
        elif msg_type == "sensor_msgs/Imu":
            def imu_cb(msg, var=data_var, top=topic_name, f=freq):
                current_time = time.time()
                # ====== 升级版防僵尸保护：如果自己不是当前激活的UI变量，直接拦截 ======
                if top not in self.monitor_vars or self.monitor_vars[top] != var:
                    return
                # ===============================================================
                # 按照频率周期拦截采样
                if current_time - self.monitor_last_process_time[top] < (1.0 / f):
                    return
                q = msg.orientation
                r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
                w = msg.angular_velocity
                a = msg.linear_acceleration
                # 获取当前选中的精度
                prec = self.monitor_precision
                text = (f"角速度: {w.x:.{prec}f}, {w.y:.{prec}f}, {w.z:.{prec}f}\n"
                        f"线加速: {a.x:.{prec}f}, {a.y:.{prec}f}, {a.z:.{prec}f}\n"
                        f"RPY姿态角: {math.degrees(r):.{prec}f}°, {math.degrees(p):.{prec}f}°, {math.degrees(y):.{prec}f}°")
                var.set(text)
                self.monitor_last_update[top] = current_time
                self.monitor_last_process_time[top] = current_time
            sub = rospy.Subscriber(topic_name, Imu, imu_cb, queue_size=1)
            self.monitor_subs.append(sub)
        # 滚轮穿透绑定
        def _scroll(e):
            self.monitor_canvas.yview_scroll(-1 if e.num == 4 else 1, "units")
        block.bind("<Button-4>", _scroll)
        block.bind("<Button-5>", _scroll)
        for child in block.winfo_children():
            child.bind("<Button-4>", _scroll)
            child.bind("<Button-5>", _scroll)
        return block

    def _on_monitor_canvas_resize(self, event=None):
        """画布宽度变化时自动布局 (兼容程序手动调用)"""
        canvas_width = event.width if event else self.monitor_canvas.winfo_width()
        if canvas_width <= 1: return
        cols = 1
        for i in range(cols):
            self.monitor_flow_frame.columnconfigure(i, weight=1, uniform="monitor_cols")
        new_block_width = max(150, (canvas_width // cols) - 20)
        for widget in self.monitor_block_widgets:
            widget.grid_forget()
        for idx, widget in enumerate(self.monitor_block_widgets):
            widget.config(width=new_block_width)
            widget.grid(row=idx // cols, column=idx % cols, pady=10, sticky="n")
        self.monitor_canvas.itemconfig(self.monitor_flow_window, width=canvas_width)

    def _check_monitor_timeout(self):
        """主线程定时器：同步监控上位机和下位机的数据断流状态"""
        current_time = time.time()
        timeout_threshold = self.lower_timeout

        # 1. 原有上位机超时处理不变
        for topic, last_time in self.monitor_last_update.items():
            if last_time == 0.0: continue
            label = self.monitor_labels[topic]
            if (current_time - last_time) > timeout_threshold:
                if label.cget("fg") != "white": label.config(fg="white")
            else:
                if label.cget("fg") != "#50fa7b": label.config(fg="#50fa7b")

        # 2. 新增下位机超时处理
        if self.lower_last_update != 0.0:
            if (current_time - self.lower_last_update) > timeout_threshold:
                for label in self.lower_labels.values():
                    if label.cget("fg") != "white": label.config(fg="white")
            else:
                for label in self.lower_labels.values():
                    if label.cget("fg") != "#50fa7b": label.config(fg="#50fa7b")

        # 每 200 毫秒自检一次
        self.master.after(200, self._check_monitor_timeout)

    def _change_precision(self, val):
        """切换全局显示精度并更新按钮高亮"""
        self.monitor_precision = int(val)
        for p, btn in self.precision_btns.items():
            if p == val:
                btn.config(bg="white", fg="black")  # 选中高亮
            else:
                btn.config(bg="#44475a", fg="white")  # 恢复暗色
    # ==================================================================

    def on_closing(self):
        """窗口关闭处理"""
        rospy.signal_shutdown("UI Window Closed")
        self.master.destroy()


if __name__ == '__main__':
    try:
        root = tk.Tk()
        app = MonitorStandalone(root)

        # 定义一个定时检查函数
        def check_shutdown():
            if rospy.is_shutdown():
                # 收到 Ctrl+C (ROS关闭信号) 时，销毁主窗口
                root.destroy()
            else:
                # 每 100 毫秒循环自检一次，不会阻塞 UI 线程
                root.after(100, check_shutdown)

        # 启动自检循环
        root.after(100, check_shutdown)

        # 启动 UI 主循环
        root.mainloop()
    except rospy.ROSInterruptException:
        pass