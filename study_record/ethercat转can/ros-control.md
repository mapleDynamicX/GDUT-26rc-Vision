# ros-control

在 ROS 1（Robot Operating System）中，`ros_control` 是一套用于机器人控制的标准化框架，其核心作用是作为机器人控制器与硬件之间的中间层，提供统一的接口和工具集，简化机器人控制逻辑的开发与硬件集成过程。

`ros_control` 的设计初衷是解决不同机器人硬件（如机械臂、移动底盘等）与控制算法之间的兼容性问题。它通过抽象硬件细节，让开发者可以专注于控制逻辑的实现，而无需关心具体硬件的通信协议或驱动方式，同时也让硬件开发者只需实现标准化接口，即可兼容各类上层控制器。

**硬件接口（Hardware Interfaces）**- 定义了控制器与硬件交互的标准化接口，是连接控制算法与物理硬件的桥梁。常见接口包括：

- `JointStateInterface`：用于读取关节状态（位置、速度、力 / 力矩等）；

- `EffortJointInterface`：用于向关节发送力 / 力矩控制指令；

- `VelocityJointInterface`：用于向关节发送速度控制指令；

- `PositionJointInterface`：用于向关节发送位置控制指令。
  
  硬件开发者需根据实际硬件能力，实现对应接口以支持数据读写。
  
  - **控制器（Controllers）**

- 实现具体的控制逻辑，基于硬件接口提供的抽象层工作。`ros_control` 提供了多种预定义控制器（如位置控制器、速度控制器、力控制器等），也支持用户自定义控制器。常见控制器包括：
  
  - `joint_state_controller`：发布关节状态到 ROS 话题（如 `/joint_states`）；
  - `position_controllers/JointPositionController`：关节位置控制；
  - `effort_controllers/JointEffortController`：关节力 / 力矩控制。

- **控制器管理器（controller_manager）**

- 负责控制器的生命周期管理（加载、启动、停止、卸载），并协调控制器与硬件接口的数据交互。它通过服务（如 `/controller_manager/load_controller`）和动作接口接受外部指令，是 `ros_control` 的核心调度组件。

- **传输层（Transmissions）**

用于处理关节指令与执行器（如电机）之间的转换（如减速比、力矩换算等），将控制器输出的 “关节空间” 指令转换为 “执行器空间” 指令，或反之。


