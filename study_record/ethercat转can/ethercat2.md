# ethercat2

rc_ecat_hw_node.cpp

```cpp
/**
 * @file rc_ecat_hw_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief ros-control hw 主节点
 * @version 0.1
 * @date 2025-05-05
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

// 包含自定义的EtherCAT硬件接口类头文件，提供硬件交互的核心功能
#include <rc_ecat_master/RcEcatHardwareInterface.h>

// 包含线程化发布器类头文件，用于在独立线程中处理消息发布
#include <any_node/ThreadedPublisher.hpp>
// 包含工作者类头文件，用于创建周期性执行的任务
#include <any_worker/Worker.hpp>

// 主函数，程序入口点
int main(int argc, char** argv) {
  // 初始化ROS节点，指定节点名称为"rc_ecat_hw_node"
  // argc和argv用于传递命令行参数，ROS会解析与ROS相关的参数
  ros::init(argc, argv, "rc_ecat_hw_node");

  // 创建公共ROS节点句柄，用于访问全局命名空间的参数和话题
  ros::NodeHandle nh;

  // 创建私有ROS节点句柄，用于访问当前节点私有命名空间（~）的参数和话题
  ros::NodeHandle nh_private("~");

  // 实例化EtherCAT硬件接口对象，该对象封装了硬件通信的具体实现
  rc_ecat_master::RcEcatHW rc_ecat_hw;

  // 初始化硬件接口，传入公共和私有节点句柄用于参数配置
  // 若初始化失败，输出错误信息并返回-1表示程序异常退出
  if (!rc_ecat_hw.init(nh, nh_private)) {
    ROS_ERROR("Failed to initialize RcEcatHW");
    return -1;
  }

  // 初始化成功，输出信息提示
  ROS_INFO_STREAM("RC ECAT hardware interface initialized successfully");

  // 创建一个工作者对象，用于周期性执行任务
  // 参数分别为：工作者名称"printfWorker"、执行周期0.01秒（100Hz）、回调函数
  // 回调函数捕获硬件接口对象，打印其状态信息（需类重载<<运算符支持）
  // 返回true表示任务执行成功，继续下一次周期
  any_worker::Worker printfWorker(
      "printfWorker", 0.01, [&](const any_worker::WorkerEvent& event) -> bool {
        std::cout << rc_ecat_hw;
        return true;
      });

  // 注释：启动工作者线程，参数45表示线程优先级（此处被注释，暂不启用）
  // printfWorker.start(45);

  // 进入ROS事件循环，处理回调函数、话题订阅等事件，保持节点运行
  ros::spin();

  // 事件循环退出后，程序正常结束，返回0
  return 0;
}
```

## OmniChassisController.cpp

```cpp
/**
 * @file OmniChassisController.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 全向底盘控制器实现（基于ros-control框架），负责运动学解算、PID控制、指令订阅
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

// 包含全向底盘控制器类的头文件，声明OmniChassisController及相关方法
#include <chassis_controller/OmniChassisController.h>

// 包含pluginlib宏定义头文件，用于将控制器注册为ROS插件（让ros-control能识别加载）
#include <pluginlib/class_list_macros.hpp>

// 定义控制器所属命名空间，避免类名冲突
namespace chassis_controller {

// 控制器初始化函数：完成参数读取、硬件接口绑定、订阅器创建
// 参数1：力控关节硬件接口（用于获取关节控制句柄，后续控制电机力矩）
// 参数2：根节点句柄（用于订阅全局话题，如cmd_vel）
// 参数3：控制器私有节点句柄（用于读取私有参数，如底盘尺寸、PID参数）
// 返回值：true=初始化成功，false=初始化失败
bool OmniChassisController::init(
    hardware_interface::EffortJointInterface *effort_joint_interface,
    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
  // 打印初始化提示，用于调试确认init函数被调用
  ROS_INFO("this ok");
  // 保存控制器私有节点句柄到类成员，方便后续（如动态更新参数）使用
  controller_nh_ = controller_nh;

  // 从私有参数服务器读取"chassis_radius"（底盘中心到车轮的距离，运动学计算核心参数）
  // 若参数未设置，打印错误信息并返回false（缺少该参数无法计算轮速）
  if (!controller_nh.getParam("chassis_radius", chassis_radius_)) {
    ROS_ERROR("Chassis radius not set");
    return false;
  }
  // 从私有参数服务器读取"wheel_radius"（车轮半径，用于线速度转角速度）
  // 若参数未设置，报错返回
  if (!controller_nh.getParam("wheel_radius", wheel_radius_)) {
    ROS_ERROR("Wheel radius not set");
    return false;
  }
  // 从私有参数服务器读取"ctrl_cmd"（速度指令话题名，如"cmd_vel"）
  // 若参数未设置，报错返回
  if (!controller_nh.getParam("ctrl_cmd", command_source_frame_)) {
    ROS_ERROR("Command source frame not set");
    return false;
  }
  // 注释：清空关节容器joints_（当前无实际代码，仅为逻辑提示）
  // clear the joints_

  // 定义XmlRpcValue对象存储"wheels"参数（配置所有车轮的关节名、PID等信息）
  XmlRpc::XmlRpcValue wheels;
  // 从私有参数服务器读取"wheels"配置到wheels变量
  controller_nh.getParam("wheels", wheels);
  // 遍历wheels中的每个车轮配置（键=车轮标识，值=该车轮的详细参数）
  for (auto &wheel : wheels) {
    // 为当前车轮创建专属节点句柄（路径：控制器私有命名空间/wheels/车轮标识）
    ros::NodeHandle nh_wheel =
        ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
    // 存储当前车轮的ROS关节名（需与URDF中定义一致）
    std::string joint_name;
    // 从车轮专属节点句柄读取"joint"参数（关节名），失败则报错返回
    if (!nh_wheel.getParam("joint", joint_name)) {
      ROS_ERROR("Joint name not set");
      return false;
    }

    // 打印当前车轮的关节名，用于调试确认关节名正确读取
    ROS_INFO_STREAM("name:"<<joint_name);
    // 在joints_（关节句柄映射表）中插入空的键值对（关节名→JointHandle）
    joints_.insert(std::make_pair(joint_name,hardware_interface::JointHandle()));
    // 从力控关节硬件接口获取该关节的控制句柄（后续用于读取轮速、设置力矩指令）
    joints_[joint_name] =effort_joint_interface->getHandle(joint_name);

    // 定义XmlRpcValue对象存储当前车轮的PID参数（P/I/D/积分上限等）
    XmlRpc::XmlRpcValue wheel_pid;
    // 从车轮专属节点句柄读取"pid"参数（PID配置），失败则报错返回
    if (!nh_wheel.getParam("pid", wheel_pid)) {
      ROS_ERROR("Joint motor pid not set");
      return false;
    }
    // 从wheel_pid中读取PID参数并转为double类型（XmlRpcValue需显式转换）
    double p = static_cast<double>(wheel_pid["p"]);
    double i = static_cast<double>(wheel_pid["i"]);
    double d = static_cast<double>(wheel_pid["d"]);
    double i_max = static_cast<double>(wheel_pid["i_max"]); // 积分项上限（防止积分饱和）
    double output_max = static_cast<double>(wheel_pid["output_max"]); // PID输出上限（限制力矩）
    double deadzone = static_cast<double>(wheel_pid["deadzone"]); // 死区（小误差不响应，避免抖动）
    // 在wheel_pid_map_（车轮PID映射表）中初始化增量式PID对象
    wheel_pid_map_[joint_name] = ctrl_common::IncrementalPID();
    // 调用PID的init方法，传入参数完成初始化
    wheel_pid_map_[joint_name].init(p, i, d, i_max, output_max, deadzone);
  }

  // 定义XmlRpcValue对象存储yaw（偏航角）调整的PID参数（用于方向锁定）
  XmlRpc::XmlRpcValue yaw_pid;
  // 从私有参数服务器读取"yaw_adjust"参数，存在则初始化yaw PID
  if(controller_nh.getParam("yaw_adjust", yaw_pid))
  {
    // 读取yaw PID的详细参数（路径：yaw_adjust→pid→p/i/d等）
    double yaw_p = static_cast<double>(yaw_pid["pid"]["p"]);
    double yaw_i = static_cast<double>(yaw_pid["pid"]["i"]);
    double yaw_d = static_cast<double>(yaw_pid["pid"]["d"]);
    double yaw_i_max = static_cast<double>(yaw_pid["pid"]["i_max"]);
    double yaw_output_max = static_cast<double>(yaw_pid["pid"]["output_max"]);
    double yaw_deadzone = static_cast<double>(yaw_pid["pid"]["deadzone"]);
    // 初始化yaw调整的PID控制器（yaw_adjust_为类成员变量）
    yaw_adjust_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
    // 注释：设置yaw调整使能标志（当前通过遥控器状态控制，此处注释）
    // yaw_adjust_flag_ = true;
  }

  // 定义XmlRpcValue对象存储yolo（目标检测）调整的PID参数（用于对准目标）
  XmlRpc::XmlRpcValue yolo_pid;
  // 从私有参数服务器读取"basket_adjust"参数（yolo调整PID），存在则初始化
  if(controller_nh.getParam("basket_adjust", yolo_pid))
  {
    // 读取yolo PID的详细参数
    double p = static_cast<double>(yolo_pid["pid"]["p"]);
    double i = static_cast<double>(yolo_pid["pid"]["i"]);
    double d = static_cast<double>(yolo_pid["pid"]["d"]);
    double i_max = static_cast<double>(yolo_pid["pid"]["i_max"]);
    double output_max = static_cast<double>(yolo_pid["pid"]["output_max"]);
    double deadzone = static_cast<double>(yolo_pid["pid"]["deadzone"]);
    // 初始化yolo调整的PID控制器（yolo_adjust_为类成员变量）
    yolo_adjust_.init(p, i, d, i_max, output_max, deadzone);
  }

  // 注释：订阅速度指令话题（cmd_vel）
  /* sub the cmd_vel */
  // 订阅根节点下的速度指令话题（话题名=command_source_frame_，类型=Twist）
  // 队列大小1（保证只处理最新指令），回调函数=cmdVelCallback，绑定this指针
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      command_source_frame_, 1, &OmniChassisController::cmdVelCallback, this);
  // 订阅里程计话题"odom"（类型=Odometry），回调函数=OdomCallback
  odom_sub_ = root_nh.subscribe<nav_msgs::Odometry>(
      "odom", 1, &OmniChassisController::OdomCallback, this);
  // 订阅位置传感器话题（自定义消息PositionSensorMsg），回调函数=PositionCallback
  position_sub_ = root_nh.subscribe<ros_ecat_msgs::PositionSensorMsg>(
      "position_sensor/position_sensor_data", 1, &OmniChassisController::PositionCallback, this);
  // 订阅遥控器状态话题"/remote_state"（自定义消息RemoteState），回调=RemoteStateCallback
  remote_state_sub_=root_nh.subscribe<ros_ecat_msgs::RemoteState>(
      "/remote_state", 1, &OmniChassisController::RemoteStateCallback, this);
  // 订阅yolo目标偏移话题"/yolo/offset"（类型=Float32），回调=YoloCallback
  yolo_sub_=root_nh.subscribe<std_msgs::Float32>("/yolo/offset",1,&OmniChassisController::YoloCallback,this);
    // 注释：调试用的单个车轮速度指令订阅（当前注释，未启用）
  // //调试所用话题
  // lf_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "left_front_wheel_joint/cmd_vel", 1, &OmniChassisController::lfVelCallback, this);
  // rf_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "right_front_wheel_joint/cmd_vel", 1, &OmniChassisController::rfVelCallback, this);
  // lb_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "left_back_wheel_joint/cmd_vel", 1, &OmniChassisController::lbVelCallback, this);
  // rb_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "right_back_wheel_joint/cmd_vel", 1, &OmniChassisController::rbVelCallback, this);

  // 初始化全部完成，返回true
  return true;
}

// 控制器周期更新函数：ros-control会按固定频率调用，实现核心控制逻辑
// 参数1：当前时间（time），参数2：更新周期（period，两次update的时间间隔）
void OmniChassisController::update(const ros::Time &time,
                                   const ros::Duration &period) {

  // 定义向量存储四个车轮的目标角速度（顺序：左前、左后、右后、右前）
  std::vector<double> joint_cmd;


  // 调用MoveJoint函数，基于运动学和控制指令计算车轮目标角速度
  joint_cmd = MoveJoint();

  // 读取当前四个车轮的实际角速度，保存到wheel_vel_map_（用于PID反馈）
  wheel_vel_map_["left_front_wheel_joint"] = joints_["left_front_wheel_joint"].getVelocity();
  wheel_vel_map_["left_back_wheel_joint"] = joints_["left_back_wheel_joint"].getVelocity();
  wheel_vel_map_["right_back_wheel_joint"] = joints_["right_back_wheel_joint"].getVelocity();
  wheel_vel_map_["right_front_wheel_joint"] = joints_["right_front_wheel_joint"].getVelocity();

  // 注释：从参数服务器动态更新PID参数（当前注释，未启用）
  //从参数服务器更新当前pid参数，用于动态调参
  // UpdateParameters();


  // 调用每个车轮的PID控制器，计算目标力矩（输入：实际速度vs目标速度，输出：力矩指令）
  // 左前车轮：PID计算结果存入wheel_cmd_map_（车轮力矩指令映射表）
  wheel_cmd_map_["left_front_wheel_joint"] = 
      wheel_pid_map_["left_front_wheel_joint"].PIDCalculate(wheel_vel_map_["left_front_wheel_joint"], joint_cmd[0]);
  // 左后车轮：PID计算力矩指令
  wheel_cmd_map_["left_back_wheel_joint"] =
      wheel_pid_map_["left_back_wheel_joint"].PIDCalculate(wheel_vel_map_["left_back_wheel_joint"], joint_cmd[1]);
  // 右后车轮：PID计算力矩指令
  wheel_cmd_map_["right_back_wheel_joint"] =
      wheel_pid_map_["right_back_wheel_joint"].PIDCalculate(wheel_vel_map_["right_back_wheel_joint"], joint_cmd[2]);
  // 右前车轮：PID计算力矩指令
  wheel_cmd_map_["right_front_wheel_joint"] =
      wheel_pid_map_["right_front_wheel_joint"].PIDCalculate(wheel_vel_map_["right_front_wheel_joint"], joint_cmd[3]);

  // 遍历所有关节，为每个关节设置PID计算出的力矩指令（下发给电机）
  for (const auto &entry : joints_) {
    // 获取当前关节名
    const std::string &joint_name = entry.first;
    // 检查该关节是否有对应的力矩指令（避免无指令关节报错）
    if (wheel_cmd_map_.find(joint_name) != wheel_cmd_map_.end()) {
      // 为关节设置力矩指令，硬件接口会将指令转发给电机驱动
      joints_[joint_name].setCommand(wheel_cmd_map_[joint_name]);
      // 注释：关节自身的更新方法（若关节类需周期更新，此处注释未启用）
      // joints_[joint_name].update(time, period);
    }
  }
}

// 计算车轮目标速度的核心函数：整合运动学、速度指令、yolo/yaw调整
// 返回值：四个车轮的目标角速度（向量，顺序：左前、左后、右后、右前）
std::vector<double> OmniChassisController::MoveJoint() {
  // 初始化四个车轮的目标速度为0.0（默认停止）
  std::vector<double> joint_cmd(4, 0.0);
  {
    // 创建互斥锁，保护cmd_vel_buffer_的读写（防止回调与update线程数据竞争）
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    // 从实时安全缓冲区读取速度指令（readFromRT()确保实时线程安全）
    geometry_msgs::Twist *cmd_vel = cmd_vel_buffer_.readFromRT();
    // 从缓冲区读取位置传感器数据（如机器人当前yaw角、篮筐相对位置）
    ros_ecat_msgs::PositionSensorMsg *position_data = position_buffer_.readFromRT();
    // 从缓冲区读取遥控器状态（如yaw锁定使能）
    ros_ecat_msgs::RemoteState *remote_state = remote_state_buffer_.readFromRT();
    // 从缓冲区读取yolo目标偏移量（如目标与图像中心的偏差）
    std_msgs::Float32* yolo_error = yolo_offset_buffer_.readFromRT();


    // 定义变量存储偏航角相关数据
    double Yaw;          // 当前机器人偏航角（从位置传感器获取）
    double yaw_target_pos;// yaw目标位置（未实际使用）
    float yaw_target_yolo;// yolo目标偏航角（未实际使用）
    double yaw_adjust_cmd;// yaw调整的附加角速度指令
    // 从遥控器状态获取yaw锁定使能标志（决定是否启用方向调整）
    yaw_adjust_flag_ = remote_state->yaw_lock;

    // 获取当前时间（用于判断yolo消息新鲜度，当前注释逻辑未启用）
    ros::Time yolo_now = ros::Time::now();
    double yolo_now_time = yolo_now.sec + yolo_now.nsec * 1000000;
    double yolo_nsec_time = yolo_time_stamp_.sec + yolo_time_stamp_.nsec*1000000;

    // 若启用yaw锁定（方向保持/对准目标）
    if(yaw_adjust_flag_)
    {
      // 若yolo偏移量不为0（有有效目标检测结果），用yolo PID计算调整指令
      if(yolo_error->data!=0)
      // 注释：判断yolo消息是否在0.1秒内（新鲜），当前注释
      // if(yolo_now_time - yolo_nsec_time <= 0.1)
      {
        // 调用yolo PID的按误差计算方法（负号调整偏移方向）
        yaw_adjust_cmd = yolo_adjust_.PIDCalculateByError(-yolo_error->data);
        // 打印yolo调整指令，用于调试
        ROS_INFO_STREAM("YOLO_cmd:"<<yaw_adjust_cmd);
      }
      // 若yolo无有效偏移，用位置传感器计算篮筐方向，通过yaw PID调整
      else
      {
        // 从位置传感器获取当前机器人yaw角，取负（调整角度方向与实际一致）
        Yaw = position_data->yaw_angle;
        Yaw = -Yaw;
        // 计算篮筐相对于机器人的偏航角（atan2(篮筐y坐标, 篮筐x坐标)）
        double basket_yaw = atan2(position_data->pose_y, position_data->pose_x);

        // 根据篮筐x坐标正负，调整basket_yaw到机器人坐标系下的方向
        if(position_data->pose_x>0)basket_yaw= -(M_PI/2.0 + basket_yaw);
        else if(position_data->pose_x<0) basket_yaw= -(basket_yaw+M_PI/2.0);
        else basket_yaw=0;

        // 将basket_yaw从弧度转为角度（方便调试查看）
        basket_yaw = basket_yaw/M_PI * 180.0;
        // 打印篮筐偏航角，用于调试
        ROS_INFO_STREAM("basket yaw:"<<basket_yaw);


        // 调用yaw PID的偏航角调整方法（输入：当前yaw、目标yaw（篮筐方向））
        yaw_adjust_cmd = yaw_adjust_.YawAdjustCalculate(Yaw,basket_yaw);
      }
      // 清零原速度指令中的角速度分量，避免干扰yaw调整（仅用调整指令控制方向）
      cmd_vel->angular.z=0;//avoid angular controller
      // 注释：打印当前yaw和调整指令，用于调试（当前注释）
      // ROS_INFO_STREAM("yaw:"<<Yaw<<"yaw_cmd:"<<yaw_adjust_cmd);
    }
    // 若未启用yaw锁定，调整指令置0（不附加方向控制）
    else yaw_adjust_cmd = 0.0;


    // 定义循环变量（未实际使用，可删除）
    int i = 0;

    // 【左前车轮】目标角速度计算：全向底盘运动学公式
    // 公式意义：将机器人整体运动（x/y线速度、总角速度）分解为左前轮角速度
    // 总角速度 = 原指令角速度 + yaw调整角速度；除以轮径将线速度转角速度
    joint_cmd[0] =
        static_cast<double>((-1.) *
                            (cmd_vel->linear.x * (-1.) * cos(M_PI / 4) +
                             cmd_vel->linear.y * sin(M_PI / 4) +
                             (cmd_vel->angular.z+yaw_adjust_cmd) * chassis_radius_) /
                            wheel_radius_);

    // 【左后车轮】目标角速度计算：全向底盘运动学公式（符号与左前不同，因安装角度）
    joint_cmd[1] =
        static_cast<double>((-1.) *
                            (cmd_vel->linear.x * (-1.) * cos(M_PI / 4) -
                             cmd_vel->linear.y * sin(M_PI / 4) +
                             (cmd_vel->angular.z + yaw_adjust_cmd) * chassis_radius_) /
                            wheel_radius_);

    // 【右后车轮】目标角速度计算：全向底盘运动学公式
    joint_cmd[2] = static_cast<double>((-1.) *
                                       (cmd_vel->linear.x * cos(M_PI / 4) -
                                        cmd_vel->linear.y * sin(M_PI / 4) +
                                        (cmd_vel->angular.z + yaw_adjust_cmd) * chassis_radius_) /
                                       wheel_radius_);

    // 【右前车轮】目标角速度计算：全向底盘运动学公式
    joint_cmd[3] = static_cast<double>((-1.) *
                                       (cmd_vel->linear.x * cos(M_PI / 4) +
                                        cmd_vel->linear.y * sin(M_PI / 4) +
                                        (cmd_vel->angular.z+yaw_adjust_cmd) * chassis_radius_) /
                                       wheel_radius_);
  }
  // 返回四个车轮的目标角速度，供update函数后续PID计算使用
  return joint_cmd;
}

// 四元数转偏航角函数：将里程计的四元数姿态转为角度制yaw（方向角）
// 返回值：角度制的偏航角（-180°~180°或0°~360°，取决于转换逻辑）
double OmniChassisController::QuaternionYaw()
{
  // 从实时缓冲区读取里程计数据
  nav_msgs::Odometry *odom_data = odom_buffer_.readFromRT();
  // 将里程计的四元数（x/y/z/w）转为tf2库的Quaternion对象
  tf2::Quaternion q(odom_data->pose.pose.orientation.x,odom_data->pose.pose.orientation.y,odom_data->pose.pose.orientation.z,odom_data->pose.pose.orientation.w);
  // 四元数归一化（确保是单位四元数，避免旋转矩阵计算误差）
  q.normalize();

  // 定义变量存储滚转角（roll）、俯仰角（pitch）、偏航角（Yaw）
  double roll,pitch,Yaw;
  // 将四元数转为3x3旋转矩阵，再从矩阵中提取RPY角（弧度制）
  tf2::Matrix3x3(q).getRPY(roll,pitch,Yaw);
  // 将Yaw从弧度转为角度（注释描述反了，实际是rad→deg）
  Yaw = (double)(Yaw/PI*180.0);//change to rad
  // 返回角度制的偏航角
  return Yaw;
}

// 计算篮筐相对于机器人的偏航角（弧度制）
// 返回值：篮筐偏航角（0~2π弧度，确保范围在正角度）
double OmniChassisController::GetYawFromBasket()
{
  // 注释：确保机器人在篮筐下方时重置定位码盘（仅逻辑提示，无实际代码）
  //确保机器人在篮筐下方reset定位码盘
  // 从实时缓冲区读取里程计数据（含篮筐相对位置）
  nav_msgs::Odometry *odom_data = odom_buffer_.readFromRT();

  // 计算篮筐相对于机器人的偏航角（atan2(篮筐y, 篮筐x)，弧度制）
  double basket_yaw = atan2(odom_data->pose.pose.position.y, odom_data->pose.pose.position.x);
  // 若角度为负（-π~0），加2π转为0~2π的正角度范围
  if (basket_yaw < 0) {
    basket_yaw += 2 * M_PI; // 确保角度在0到2π之间
  }
  // 返回篮筐相对于机器人的偏航角
  return basket_yaw;
}

// 从yolo获取偏航角函数：当前未实现（空函数，预留接口）
float OmniChassisController::GetYawFromYolo()
{

}

// 动态更新PID参数函数：从参数服务器重新读取配置，更新所有PID控制器参数
void OmniChassisController::UpdateParameters() {

  // 定义XmlRpcValue对象存储wheels参数
  XmlRpc::XmlRpcValue wheels;
  // 从控制器私有参数服务器读取wheels配置
  controller_nh_.getParam("wheels", wheels);
  // 遍历每个车轮的配置
  for(auto &wheel:wheels)
  {
    // 从车轮配置中读取关节名
    std::string joint_name = wheel.second["joint"];
    // 从车轮配置中读取PID参数
    XmlRpc::XmlRpcValue wheel_pid=wheel.second["pid"];
    // 读取PID的P/I/D/积分上限/输出上限/死区
    double p = static_cast<double>(wheel_pid["p"]);
    double i = static_cast<double>(wheel_pid["i"]);
    double d = static_cast<double>(wheel_pid["d"]);
    double i_max = static_cast<double>(wheel_pid["i_max"]);
    double output_max = static_cast<double>(wheel_pid["output_max"]);
    double deadzone = static_cast<double>(wheel_pid["deadzone"]);
    // 重新初始化该车轮的PID控制器（覆盖旧参数，实现动态更新）
    wheel_pid_map_[joint_name].init(p, i, d, i_max, output_max, deadzone);
  }

  // 若yaw调整使能，更新yaw PID参数
  if(yaw_adjust_flag_)
  {
    // 定义XmlRpcValue对象存储yaw PID参数
    XmlRpc::XmlRpcValue yaw_pid;
    // 从私有参数服务器读取yaw_adjust配置
    controller_nh_.getParam("yaw_adjust", yaw_pid);
    // 读取yaw PID参数（注：原代码路径可能有误，应为yaw_pid["pid"]["p"]，需与配置文件匹配）
    double yaw_p = static_cast<double>(yaw_pid["p"]);
    double yaw_i = static_cast<double>(yaw_pid["i"]);
    double yaw_d = static_cast<double>(yaw_pid["d"]);
    double yaw_i_max = static_cast<double>(yaw_pid["i_max"]);
    double yaw_output_max = static_cast<double>(yaw_pid["output_max"]);
    double yaw_deadzone = static_cast<double>(yaw_pid["deadzone"]);
    // 重新初始化yaw PID控制器
    yaw_adjust_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
  }
}

// cmd_vel话题回调函数：接收速度指令，线程安全写入缓冲区
// 参数：Twist类型的常量智能指针（避免拷贝，确保消息不被修改）
void OmniChassisController::cmdVelCallback(
    const geometry_msgs::TwistConstPtr &msg) {
  // 互斥锁保护cmd_vel_buffer_，防止回调与update线程同时读写
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  // 将收到的速度指令写入非实时缓冲区（writeFromNonRT()适用于非实时回调线程）
  cmd_vel_buffer_.writeFromNonRT(*msg);
}

// odom话题回调函数：接收里程计数据，写入缓冲区
void OmniChassisController::OdomCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  // 互斥锁保护odom_buffer_，防止数据竞争
  std::lock_guard<std::mutex> lock(odom_mutex_);
  // 将里程计数据写入非实时缓冲区
  odom_buffer_.writeFromNonRT(*msg);
  // 记录当前时间为yolo时间戳（用于判断yolo消息新鲜度，逻辑未启用）
  yolo_time_stamp_ = ros::Time::now();
}

// 位置传感器话题回调函数：接收位置数据，写入缓冲区
void OmniChassisController::PositionCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg)
{
  // 将位置传感器数据写入非实时缓冲区（注：此处未加互斥锁，多线程访问需补充）
  position_buffer_.writeFromNonRT(*msg);
}

// 遥控器状态话题回调函数：接收遥控器数据，写入缓冲区
void OmniChassisController::RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg) 
{
  // 将遥控器状态数据写入非实时缓冲区（注：此处未加互斥锁，多线程访问需补充）
  remote_state_buffer_.writeFromNonRT(*msg);
}

// yolo偏移话题回调函数：接收yolo数据，写入缓冲区
void OmniChassisController::YoloCallback(const std_msgs::Float32ConstPtr &msg)
{
  // 将yolo偏移数据写入非实时缓冲区（注：此处未加互斥锁，多线程访问需补充）
  yolo_offset_buffer_.writeFromNonRT(*msg);
}

// 正向运动学函数：未实现（功能应为"由车轮速度计算机器人整体速度"，预留接口）
geometry_msgs::Twist forwardKinematics(void) {}

// 左前车轮速度指令回调函数（调试用）：直接控制单个车轮
void OmniChassisController::lfVelCallback(const std_msgs::Float64ConstPtr &msg)
{
  // 注释：将左前车轮指令写入缓冲区（未启用）
  // left_front_buffer_.writeFromNonRT(*msg);
  // 读取左前车轮当前实际速度（注释，未启用）
  // wheel_vel_map_["left_front_wheel_joint"] = joints_["left_front_wheel_joint"].getVelocity();
  // PID计算左前车轮力矩指令（注释，未启用）
  // double lf_cmd_vel = wheel_pid_map_["left_front_wheel_joint"].PIDCalculate(wheel_vel_map_["left_front_wheel_joint"], msg->data);
  // 打印力矩指令（注释，未启用）
  // ROS_INFO_STREAM("vel_cmd:"<<lf_cmd_vel);
  // 为左前车轮设置力矩指令（注释，未启用）
  // joints_["left_front_wheel_joint"].setCommand(lf_cmd_vel);
  // 注释：将目标速度赋值给joint_cmd[0]（未启用）
  // joint_cmd[0]=msg->data;

}

// 右前车轮速度指令回调函数（调试用）：直接控制单个车轮
void OmniChassisController::rfVelCallback(const std_msgs::Float64ConstPtr &msg)
{
  // 读取右前车轮当前实际速度
  wheel_vel_map_["right_front_wheel_joint"] = joints_["right_front_wheel_joint"].getVelocity();
  // PID计算右前车轮力矩指令（目标速度=msg->data）
  double rf_cmd_vel = wheel_pid_map_["right_front_wheel_joint"].PIDCalculate(wheel_vel_map_["right_front_wheel_joint"], msg->data);
  // 为右前车轮设置力矩指令
  joints_["right_front_wheel_joint"].setCommand(rf_cmd_vel);
}

// 左后车轮速度指令回调函数（调试用）：直接控制单个车轮
void OmniChassisController::lbVelCallback(const std_msgs::Float64ConstPtr &msg)
{
  // 读取左后车轮当前实际速度
  wheel_vel_map_["left_back_wheel_joint"] = joints_["left_back_wheel_joint"].getVelocity();
  // PID计算左后车轮力矩指令
  double lb_cmd_vel = wheel_pid_map_["left_back_wheel_joint"].PIDCalculate(wheel_vel_map_["left_back_wheel_joint"], msg->data);
  // 为左后车轮设置力矩指令
  joints_["left_back_wheel_joint"].setCommand(lb_cmd_vel);
}

// 右后车轮速度指令回调函数（调试用）：直接控制单个车轮
void OmniChassisController::rbVelCallback(const std_msgs::Float64ConstPtr &msg)
{
  // 读取右后车轮当前实际速度
  wheel_vel_map_["right_back_wheel_joint"] = joints_["right_back_wheel_joint"].getVelocity();
  // PID计算右后车轮力矩指令
  double rb_cmd_vel = wheel_pid_map_["right_back_wheel_joint"].PIDCalculate(wheel_vel_map_["right_back_wheel_joint"], msg->data);
  // 为右后车轮设置力矩指令
  joints_["right_back_wheel_joint"].setCommand(rb_cmd_vel);
}

}
```

## OmniChassisController.h

```cpp
/**
 * @file OmniChassisController.h
 * @author Keten (2863861004@qq.com)
 * @brief 四全向轮底盘控制器的头文件，定义控制器类及成员
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

// 防止头文件重复包含
#pragma once

// 包含XmlRpcValue头文件，用于解析ROS参数服务器中的XML格式参数
#include <XmlRpcValue.h>
// 包含关节控制器状态消息头文件，用于发布控制器状态（未直接使用，预留）
#include <control_msgs/JointControllerState.h>
// 包含ROS控制工具箱的PID头文件，提供PID控制基础功能（未直接使用，自定义PID替代）
#include <control_toolbox/pid.h>
// 包含控制器接口基类头文件，所有ROS控制器需继承此类
#include <controller_interface/controller.h>
// 包含Twist消息头文件，用于接收和处理速度指令（x/y线速度、z角速度）
#include <geometry_msgs/Twist.h>
// 包含关节命令接口头文件，提供力控关节的控制接口
#include <hardware_interface/joint_command_interface.h>
// 包含实时缓冲区头文件，用于线程安全地存储和访问实时数据（避免数据竞争）
#include <realtime_tools/realtime_buffer.h>
// 包含互斥锁头文件，用于多线程同步（保护共享数据）
#include <mutex>
// 包含里程计消息头文件，用于接收机器人位置和姿态信息
#include <nav_msgs/Odometry.h>
// 包含数学函数库头文件，提供三角函数等数学运算
#include <cmath>
// 包含自定义的EtherCAT硬件接口头文件，用于与硬件交互
#include <rc_ecat_master/RcEcatHardwareInterface.h>
// 包含tf2库的矩阵头文件，用于处理3x3旋转矩阵（四元数转欧拉角）
#include <tf2/LinearMath/Matrix3x3.h>
// 包含tf2库的四元数头文件，用于处理姿态的四元数表示
#include <tf2/LinearMath/Quaternion.h>
// 包含Float64消息头文件，用于调试时的单个车轮速度指令
#include <std_msgs/Float64.h>
// 包含Float32消息头文件，用于接收YOLO目标检测的偏移量
#include <std_msgs/Float32.h>
// 包含自定义的位置传感器消息头文件，用于接收机器人位置和姿态传感器数据
#include <ros_ecat_msgs/PositionSensorMsg.h>
// 包含自定义的遥控器状态消息头文件，用于接收遥控器控制指令（如使能航向角锁定）
#include <ros_ecat_msgs/RemoteState.h>

// 包含自定义的增量式PID控制器头文件，用于车轮速度闭环控制
#include "ctrl_common/PID/IncrementalPID.h"
// 包含自定义的位置式PID控制器头文件，用于YOLO目标偏移调整
#include "ctrl_common/PID/PositionPID.h"
// 包含自定义的航向角调整类头文件，用于处理航向角锁定逻辑
#include "ctrl_common/MoveBase/YawAdjust.h"


// 定义控制器所属命名空间，避免类名冲突
namespace chassis_controller {

/*
  后续可以使用多接口控制器，只要换个继承就行了
*/

// 定义全向底盘控制器类，继承自ROS控制框架的Controller基类
// 模板参数指定使用力控关节接口（hardware_interface::EffortJointInterface）
class OmniChassisController : public controller_interface::Controller<
                                  hardware_interface::EffortJointInterface> {
 public:
  // 默认构造函数：使用编译器生成的默认构造函数
  OmniChassisController() = default;
  // 析构函数：使用编译器生成的默认析构函数，添加override确保重写基类析构
  ~OmniChassisController() override = default;

  // 初始化函数：重写基类的init方法，完成控制器初始化
  // 参数1：力控关节接口（用于获取关节控制句柄）
  // 参数2：根节点句柄（用于订阅全局话题）
  // 参数3：控制器私有节点句柄（用于读取私有参数）
  // 返回值：bool（true=初始化成功，false=失败）
  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  // 更新函数：重写基类的update方法，按固定周期执行控制逻辑
  // 参数1：当前时间，参数2：控制周期（两次update的时间间隔）
  void update(const ros::Time &time, const ros::Duration &period) override;

  // 运动学解算函数：根据速度指令和附加调整量，计算四个车轮的目标速度
  // 返回值：存储四个车轮目标速度的向量（顺序：左前、左后、右后、右前）
  std::vector<double> MoveJoint();

  // 四元数转偏航角函数：将里程计的四元数姿态转换为偏航角（yaw）
  // 返回值：角度制的偏航角
  double QuaternionYaw();

  // 从篮筐位置计算偏航角函数：根据篮筐相对位置计算目标偏航角
  // 返回值：篮筐相对机器人的偏航角（弧度制）
  double GetYawFromBasket();

  // 从YOLO数据获取偏航角函数：预留接口，用于从YOLO检测结果计算偏航角
  // 返回值：YOLO检测得到的偏航角（浮点型）
  float GetYawFromYolo();

  // 速度指令回调函数：处理订阅的cmd_vel话题消息，更新速度指令缓冲区
  // 参数：Twist类型的常量智能指针（指向速度指令消息）
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);

  // 里程计回调函数：处理订阅的odom话题消息，更新里程计缓冲区
  // 参数：Odometry类型的常量智能指针（指向里程计消息）
  void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

  // 位置传感器回调函数：处理订阅的位置传感器消息，更新位置缓冲区
  // 参数：PositionSensorMsg类型的常量智能指针（指向位置传感器消息）
  void PositionCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg);

  // 遥控器状态回调函数：处理订阅的遥控器状态消息，更新遥控器状态缓冲区
  // 参数：RemoteState类型的常量智能指针（指向遥控器状态消息）
  void RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg);

  // YOLO偏移回调函数：处理订阅的YOLO目标偏移消息，更新YOLO缓冲区
  // 参数：Float32类型的常量智能指针（指向偏移量消息）
  void YoloCallback(const std_msgs::Float32ConstPtr &msg);

  // 左前车轮速度指令回调函数（调试用）：处理左前车轮的调试速度指令
  // 参数：Float64类型的常量智能指针（指向速度指令）
  void lfVelCallback(const std_msgs::Float64ConstPtr &msg);

  // 右前车轮速度指令回调函数（调试用）：处理右前车轮的调试速度指令
  void rfVelCallback(const std_msgs::Float64ConstPtr &msg);

  // 左后车轮速度指令回调函数（调试用）：处理左后车轮的调试速度指令
  void lbVelCallback(const std_msgs::Float64ConstPtr &msg);

  // 右后车轮速度指令回调函数（调试用）：处理右后车轮的调试速度指令
  void rbVelCallback(const std_msgs::Float64ConstPtr &msg);

  /**
   * 调参阶段用于获取动态更改的参数
   */
  // 动态更新参数函数：从参数服务器重新读取参数，更新PID等配置
  void UpdateParameters();

  // 正向运动学函数：预留接口，用于从车轮速度计算机器人整体速度
  // 返回值：Twist类型（包含x/y线速度和z角速度）
  geometry_msgs::Twist forwardKinematics(void);

 private:
  // 控制器私有节点句柄：用于访问控制器私有命名空间的参数和话题
  ros::NodeHandle controller_nh_;

  // 关节句柄映射表：键=关节名，值=关节控制句柄（用于读取轮速和设置力矩）
  std::unordered_map<std::string,hardware_interface::JointHandle> joints_;

  // 底盘半径：底盘中心到车轮旋转中心的距离（运动学计算核心参数）
  double chassis_radius_;

  // 车轮半径：用于将线速度转换为角速度（v = ω * r → ω = v / r）
  double wheel_radius_;

  // 速度指令订阅者：订阅底盘速度指令话题（如cmd_vel）
  ros::Subscriber cmd_vel_sub_;

  // 车轮命令映射表：键=关节名，值=PID计算出的力矩指令（用于下发给电机）
  std::unordered_map<std::string, double> wheel_cmd_map_;

  // 车轮速度映射表：键=关节名，值=当前车轮实际角速度（用于PID反馈）
  std::unordered_map<std::string, double> wheel_vel_map_;

  // 速度指令变量：存储当前底盘速度指令（x/y线速度、z角速度）
  geometry_msgs::Twist cmd_vel_;

  // 命令源坐标系：存储速度指令的坐标系名称（从参数服务器读取）
  std::string command_source_frame_;

  // 速度指令互斥锁：保护cmd_vel_buffer_的读写，避免多线程数据竞争
  std::mutex cmd_vel_mutex_;

  // 速度指令实时缓冲区：线程安全地存储速度指令（实时更新线程与回调线程共享）
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;

  // 车轮PID映射表：键=关节名，值=增量式PID控制器（用于车轮速度闭环控制）
  std::unordered_map<std::string, ctrl_common::IncrementalPID> wheel_pid_map_;

  // 航向角调整使能标志：是否启用航向角锁定功能（默认关闭）
  bool yaw_adjust_flag_ = false;//是否启用航向角调整，默认为关

  // 圆周率常量：用于角度弧度转换和运动学计算
  double PI = 3.1415926;

  // 航向角调整控制器：用于航向角锁定的专用控制器（含PID逻辑）
  ctrl_common::YawAdjust yaw_adjust_; //yaw轴调整pid

  // YOLO调整PID：位置式PID，用于基于YOLO目标偏移的方向调整
  ctrl_common::PositionPID yolo_adjust_;

  // 里程计订阅者：订阅里程计话题，获取机器人位置和姿态
  ros::Subscriber odom_sub_;//获取当前机器人坐标及姿态

  // 遥控器状态订阅者：订阅遥控器状态话题，获取使能指令（如航向角锁定）
  ros::Subscriber remote_state_sub_;//获取遥控器状态

  // 位置传感器订阅者：订阅位置传感器话题，获取机器人位置和姿态传感器数据
  ros::Subscriber position_sub_;

  // YOLO偏移订阅者：订阅YOLO目标偏移话题，获取目标与中心的偏差
  ros::Subscriber yolo_sub_;

  // 目标航向角：目标偏航角（预留，用于规划器设置目标方向）
  double target_yaw_ = 0.0;//目标航向角，因暂时无规划器，现先直接赋值调试

  // 里程计数据：存储当前里程计信息（位置、姿态、速度等）
  nav_msgs::Odometry odom_;

  // 动作数据：存储机器人动作指令（未直接使用，预留）
  ros_ecat_msgs::ActionData action_;

  // 遥控器状态数据：存储当前遥控器状态（如按键、使能标志等）
  ros_ecat_msgs::RemoteState remote_state_;

  // 里程计实时缓冲区：线程安全地存储里程计数据
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;

  // 位置传感器实时缓冲区：线程安全地存储位置传感器数据
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::PositionSensorMsg> position_buffer_;

  // 左前车轮调试缓冲区：线程安全地存储左前车轮的调试速度指令（预留）
  realtime_tools::RealtimeBuffer<std_msgs::Float64> left_front_buffer_;

  // 遥控器状态实时缓冲区：线程安全地存储遥控器状态数据
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> remote_state_buffer_;

  // YOLO偏移实时缓冲区：线程安全地存储YOLO目标偏移数据
  realtime_tools::RealtimeBuffer<std_msgs::Float32> yolo_offset_buffer_;

  // YOLO时间戳：存储最新YOLO消息的接收时间（用于判断消息新鲜度）
  ros::Time yolo_time_stamp_;

  // 里程计互斥锁：保护odom_buffer_的读写，避免多线程数据竞争
  std::mutex odom_mutex_;

  // 航向角调整值：存储航向角调整的附加角速度（用于修正方向）
  double yaw_adjust_value_;

  // 调试用订阅者：分别订阅四个车轮的调试速度指令话题
  //To Debug
  ros::Subscriber lf_cmd_vel_sub_;
  ros::Subscriber rf_cmd_vel_sub_;
  ros::Subscriber lb_cmd_vel_sub_;
  ros::Subscriber rb_cmd_vel_sub_;

  // 左前车轮调试指令：存储左前车轮的调试速度指令（预留）
  double lf_cmd_debug;
  // 注释：四个车轮的命令数组（未使用，预留）
  // double joint_cmd[4];
};

}  // namespace chassis_controller
```
