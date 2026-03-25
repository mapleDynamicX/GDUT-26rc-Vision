# tf

3. 🟢 ROS 时间同步问题（rosbag 回放场景）
   使用 rosbag 回放时，如果不设置 use_sim_time，节点可能因时间戳不匹配收不到消息（虽然这不会导致窗口不弹，但会导致窗口没数据）。
   修复方案：
   严格按照以下顺序启动节点：
   启动 roscore
   bash
   运行
   roscore
   设置使用模拟时间（在新终端）
   bash
   运行
   rosparam set use_sim_time true
   运行你的 Python 节点（在新终端）
   注意：确保 _pose_topic 参数指向你 rosbag 中的实际 TF 话题名（例如 /tf）。
   bash
   运行
   
   # 示例：假设 mode 是 real_world，话题是 /tf
   
   python your_script_name.py _mode:=real_world _pose_topic:=/tf
   播放 rosbag（在新终端，务必加 --clock）
   bash
   运行
   <mark>rosbag play your_bag_file.bag --clock</mark>


