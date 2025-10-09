# Omni

```cpp
// 初始化控制器，由ROS控制器管理器调用
bool OmniChassisController::init(
    hardware_interface::EffortJointInterface *effort_joint_interface, // 硬件接口指针，用于获取关节控制句柄
    ros::NodeHandle &root_nh, // ROS根节点句柄（通常是/）
    ros::NodeHandle &controller_nh) // 控制器私有节点句柄（通常是/controller_name）
{
  // 输出日志，表示进入初始化函数
  ROS_INFO("this ok");

  // 保存控制器私有节点句柄，供类内其他方法使用
  controller_nh_ = controller_nh;

  // 从参数服务器读取底盘半径，如果读取失败则报错并返回false
  if (!controller_nh.getParam("chassis_radius", chassis_radius_)) {
    ROS_ERROR("Chassis radius not set");
    return false;
  }

  // 从参数服务器读取轮子半径，如果读取失败则报错并返回false
  if (!controller_nh.getParam("wheel_radius", wheel_radius_)) {
    ROS_ERROR("Wheel radius not set");
    return false;
  }

  // 从参数服务器读取控制命令来源的话题名，如果读取失败则报错并返回false
  if (!controller_nh.getParam("ctrl_cmd", command_source_frame_)) {
    ROS_ERROR("Command source frame not set");
    return false;
  }

  // 定义XmlRpcValue对象，用于读取并解析参数服务器中的"wheels"配置
  XmlRpc::XmlRpcValue wheels;
  controller_nh.getParam("wheels", wheels);

  // 遍历每一个轮子的配置
  for (auto &wheel : wheels) {
    // 创建该轮子对应的ROS节点句柄（路径是 wheels/轮子名）
    ros::NodeHandle nh_wheel =
        ros::NodeHandle(controller_nh, "wheels/" + wheel.first);

    // 存储当前轮子的关节名
    std::string joint_name;

    // 读取关节名参数，如果不存在则报错并返回false
    if (!nh_wheel.getParam("joint", joint_name)) {
      ROS_ERROR("Joint name not set");
      return false;
    }

    // 输出当前轮子的关节名，用于调试
    ROS_INFO_STREAM("name:"<<joint_name);

    // 在joints_字典中插入一个以关节名为键、空JointHandle为值的元素
    joints_.insert(std::make_pair(joint_name,hardware_interface::JointHandle()));

    // 从硬件接口中获取该关节的控制句柄（用于后续设置力矩）
    joints_[joint_name] = effort_joint_interface->getHandle(joint_name);

    // 定义XmlRpcValue对象，用于读取并解析该轮子的PID参数
    XmlRpc::XmlRpcValue wheel_pid;

    // 读取PID参数，如果不存在则报错并返回false
    if (!nh_wheel.getParam("pid", wheel_pid)) {
      ROS_ERROR("Joint motor pid not set");
      return false;
    }

    // 从XmlRpcValue中提取PID参数（P、I、D、积分限幅、输出限幅、死区）
    double p = static_cast<double>(wheel_pid["p"]);
    double i = static_cast<double>(wheel_pid["i"]);
    double d = static_cast<double>(wheel_pid["d"]);
    double i_max = static_cast<double>(wheel_pid["i_max"]);
    double output_max = static_cast<double>(wheel_pid["output_max"]);
    double deadzone = static_cast<double>(wheel_pid["deadzone"]);

    // 在wheel_pid_map_字典中为该关节创建一个PID控制器对象
    wheel_pid_map_[joint_name] = ctrl_common::IncrementalPID();

    // 初始化该PID控制器的参数
    wheel_pid_map_[joint_name].init(p, i, d, i_max, output_max, deadzone);
  }

  // 定义XmlRpcValue对象，用于读取偏航角调整PID参数
  XmlRpc::XmlRpcValue yaw_pid;

  // 如果参数服务器中存在"yaw_adjust"配置
  if(controller_nh.getParam("yaw_adjust", yaw_pid))
  {
    // 提取yaw PID的参数
    double yaw_p = static_cast<double>(yaw_pid["pid"]["p"]);
    double yaw_i = static_cast<double>(yaw_pid["pid"]["i"]);
    double yaw_d = static_cast<double>(yaw_pid["pid"]["d"]);
    double yaw_i_max = static_cast<double>(yaw_pid["pid"]["i_max"]);
    double yaw_output_max = static_cast<double>(yaw_pid["pid"]["output_max"]);
    double yaw_deadzone = static_cast<double>(yaw_pid["pid"]["deadzone"]);

    // 初始化偏航调整PID控制器
    yaw_adjust_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
    // yaw_adjust_flag_ = true; // （被注释掉）标记偏航调整功能开启
  }

  // 定义XmlRpcValue对象，用于读取yolo视觉调整PID参数
  XmlRpc::XmlRpcValue yolo_pid;

  // 如果参数服务器中存在"basket_adjust"配置
  if(controller_nh.getParam("basket_adjust", yolo_pid))
  {
    // 提取yolo PID的参数
    double p = static_cast<double>(yolo_pid["pid"]["p"]);
    double i = static_cast<double>(yolo_pid["pid"]["i"]);
    double d = static_cast<double>(yolo_pid["pid"]["d"]);
    double i_max = static_cast<double>(yolo_pid["pid"]["i_max"]);
    double output_max = static_cast<double>(yolo_pid["pid"]["output_max"]);
    double deadzone = static_cast<double>(yolo_pid["pid"]["deadzone"]);

    // 初始化yolo视觉调整PID控制器
    yolo_adjust_.init(p, i, d, i_max, output_max, deadzone);
  }

  /* 订阅速度控制命令 */
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      command_source_frame_,  // 订阅的话题名（由参数ctrl_cmd指定）
      1,                      // 消息队列长度
      &OmniChassisController::cmdVelCallback, // 回调函数
      this                    // 绑定到当前类实例
  );

  // 订阅里程计信息
  odom_sub_ = root_nh.subscribe<nav_msgs::Odometry>(
      "odom", 1, &OmniChassisController::OdomCallback, this);

  // 订阅位置传感器数据
  position_sub_ = root_nh.subscribe<ros_ecat_msgs::PositionSensorMsg>(
      "position_sensor/position_sensor_data", 1, &OmniChassisController::PositionCallback, this);

  // 订阅遥控器状态信息
  remote_state_sub_ = root_nh.subscribe<ros_ecat_msgs::RemoteState>(
      "/remote_state", 1, &OmniChassisController::RemoteStateCallback, this);

  // 订阅yolo检测到的偏移量信息
  yolo_sub_ = root_nh.subscribe<std_msgs::Float32>(
      "/yolo/offset", 1, &OmniChassisController::YoloCallback, this);
```

#### 5. 绑定硬件关节的 “控制句柄”（核心！获得控制权）

```cpp
// 第一步：往joints_容器里插入一个“关节名-空句柄”的键值对
joints_.insert(std::make_pair(joint_name, hardware_interface::JointHandle()));
// 第二步：用硬件接口获取真实的关节句柄，覆盖掉空句柄
joints_[joint_name] = effort_joint_interface->getHandle(joint_name);
```

- 先明确`joints_`：它是控制器类里的一个容器（通常是`std::map<std::string, hardware_interface::JointHandle>`），作用是 “保存所有轮子的关节控制句柄，方便后续调用”。
- 逐句拆解：
  1. `joints_.insert(...)`：先在`joints_`里 “占个坑”—— 把当前关节名作为键，插入一个 “空的关节句柄”（`hardware_interface::JointHandle()`），确保后续赋值时键一定存在（避免报错）。
  2. `effort_joint_interface->getHandle(joint_name)`：
     - `effort_joint_interface`：是传入`init`函数的 “力矩关节硬件接口”（来自硬件驱动，比如电机驱动节点）。
     - `getHandle(joint_name)`：向硬件接口 “索要” 当前关节的 “控制句柄”—— 这个句柄相当于 “控制轮子的遥控器”，后续通过这个句柄可以：
       - 读取轮子当前的转速、位置（`handle.getVelocity()`）；
       - 给轮子发送力矩指令（`handle.setCommand(力矩值)`）。
  3. `joints_[joint_name] = ...`：把从硬件接口拿到的 “真实控制句柄”，存到`joints_`里对应关节名的位置。
- **通俗理解**：这两步相当于 “先在通讯录里记下朋友的名字（关节名），再拿到他的手机号（控制句柄），后续想联系他（控制轮子），直接查通讯录（joints_）就行”。

```cpp
std::unordered_map<std::string, hardware_interface::JointHandle>
```

### 1. `std::unordered_map`

- C++11 引入的标准库容器
- 实现了 **哈希表（hash table）** 数据结构
- 存储 **键值对（key-value pairs）**
- 查找、插入、删除的平均时间复杂度是 **O(1)**
- 与 `std::map` 不同：
  - `std::map` 基于红黑树，元素是有序的，查找 O (log n)
  - `std::unordered_map` 基于哈希表，元素无序，查找更快（平均 O (1)）


