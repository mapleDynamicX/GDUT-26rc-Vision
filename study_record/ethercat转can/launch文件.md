# test_swerve.launch

```xml
<launch>

  <!--<rosparam file="$(find rc_ecat_master)/config/hw/test_ecat_hw.yaml" command="load" />-->
  <rosparam file="$(find rc_ecat_master)/config/hw/kobe_ecat_hw.yaml" command="load" />
  <!--<rosparam file="$(find rc_ecat_master)/config/limits/kobe_swerve_joint_limits.yaml" command="load" />-->
  <rosparam file="$(find rc_ecat_master)/config/limits/kobe_joint_limits.yaml" command="load" />
  <param name="robot_description" command="$(find xacro)/xacro $(find joint_description)/urdf/kobe/swerve_chassis.urdf.xacro"/>

  <!-- launch-prefix表示在launch启动节点时，会在执行命令前预先插入ethercat_grant命令 -->
  <node name="rc_ecat_hw_node" pkg="rc_ecat_master" type="rc_ecat_hw_node" output="screen">

  </node>
</launch>
```

### 1. `<rosparam>` 标签：加载参数文件到 ROS 参数服务器

`<rosparam>` 用于将.yaml 格式的参数文件加载到 ROS 参数服务器，供节点运行时读取。`command="load"` 表示执行 “加载” 操作。

- `<!--<rosparam file="$(find rc_ecat_master)/config/hw/test_ecat_hw.yaml" command="load" />-->`

- 这是一行被注释的配置，用于加载**测试用的 EtherCAT 硬件参数文件**。`test_ecat_hw.yaml` 通常包含测试环境下的硬件配置（如电机 ID、通信参数等），当前未启用（被`<!-- -->`注释）。

- `<rosparam file="$(find rc_ecat_master)/config/hw/kobe_ecat_hw.yaml" command="load" />`

- 加载**Kobe 机器人的 EtherCAT 硬件参数文件**。`kobe_ecat_hw.yaml` 是实际运行时使用的硬件配置文件，内容可能包括：EtherCAT 从站设备的 ID、类型、控制模式（如位置 / 速度模式）、通信周期等，是节点与硬件（如电机、传感器）通信的核心配置。

- `<!--<rosparam file="$(find rc_ecat_master)/config/limits/kobe_swerve_joint_limits.yaml" command="load" />-->`

- 被注释的关节限制参数文件，用于**麦克纳姆轮（swerve）底盘的关节限制**。文件中可能定义麦克纳姆轮相关关节的位置、速度、加速度极限（如最大旋转角度、最快转速），当前未启用。

- `<rosparam file="$(find rc_ecat_master)/config/limits/kobe_joint_limits.yaml" command="load" />`

- 加载**Kobe 机器人通用的关节限制参数文件**。`kobe_joint_limits.yaml` 定义了机器人各关节（如机械臂关节、底盘驱动轮关节）的运动安全阈值（如位置范围、最大速度），用于保护硬件避免超限损坏，是运动控制中的关键安全配置。

### 2. `<param name="robot_description" ...>`：定义机器人模型描述

- `name="robot_description"`：ROS 中约定的参数名，用于存储机器人的**URDF/Xacro 模型描述**（机器人的结构、关节连接、传感器位置等）。

- `command="$(find xacro)/xacro $(find joint_description)/urdf/kobe/swerve_chassis.urdf.xacro"`：通过`xacro`工具解析`.urdf.xacro`文件（一种支持参数化的 URDF 扩展格式），生成标准 URDF 格式的机器人模型，并将其内容存入`robot_description`参数。
  
  作用：`robot_description`是机器人可视化（如 RViz）、运动学计算（如 IK 解算）、碰撞检测等功能的基础，所有需要了解机器人结构的节点（如 MoveIt!、导航栈）都会读取该参数。

### 3. `<node>` 标签：启动 EtherCAT 硬件控制节点

- `name="rc_ecat_hw_node"`：节点的名称（在 ROS 网络中唯一标识该节点）。

- `pkg="rc_ecat_master"`：节点所在的功能包（package）名称。

- `type="rc_ecat_hw_node"`：节点的可执行文件名称（编译后生成的二进制文件）。

- `output="screen"`：节点的日志输出会打印到终端（方便调试）。
  
  作用：`rc_ecat_hw_node`是核心控制节点，负责通过 EtherCAT 总线与硬件设备（如电机驱动器）通信，根据参数服务器中的配置（如`kobe_ecat_hw.yaml`和关节限制）初始化硬件，并提供 ROS 接口（如话题、服务）供上层控制逻辑（如运动规划节点）发送控制指令。
