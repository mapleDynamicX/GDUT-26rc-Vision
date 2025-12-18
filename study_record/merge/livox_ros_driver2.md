# livox_ros_driver2

```cpp
// 标准输入输出流头文件，用于控制台打印日志、信息等
#include <iostream>
// 时间处理头文件，提供高精度时间类型、线程休眠、时间判断等功能
#include <chrono>
// 向量容器头文件，虽未直接使用，但为驱动预留容器支持
#include <vector>
// 信号处理头文件，用于处理系统信号（如退出信号），驱动中预留扩展
#include <csignal>
// 线程库头文件，用于创建和管理多线程
#include <thread>

// Livox ROS驱动2核心头文件，包含驱动核心定义、枚举、宏等
#include "include/livox_ros_driver2.h"
// ROS相关头文件汇总（自定义），包含ros/ros.h、sensor_msgs/PointCloud2.h等
#include "include/ros_headers.h"
// DriverNode类头文件，封装ROS节点、数据轮询线程等核心逻辑
#include "driver_node.h"
// Lddc（Lidar Data Distribute Control）类头文件，负责激光雷达数据分发控制
#include "lddc.h"
// LdsLidar（Lidar Data Source Lidar）类头文件，激光雷达原始数据读取源
#include "lds_lidar.h"

// 使用livox_ros命名空间，避免重复书写命名空间前缀
using namespace livox_ros;

/**
 * @brief 主函数，Livox激光雷达ROS驱动入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态码
 */
int main(int argc, char **argv) {
  /** Ros related */
  // 设置ROS控制台默认日志器的级别为Debug，开启调试日志输出
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    // 通知ROS系统日志级别已变更，使设置生效
    ros::console::notifyLoggerLevelsChanged();
  }

  // 初始化ROS节点，节点名称为"livox_lidar_publisher"，argc/argv传递命令行参数
  ros::init(argc, argv, "livox_lidar_publisher");

  // 注释掉的原生ROS节点句柄，改用封装后的DriverNode类
  // ros::NodeHandle livox_node;
  // 实例化DriverNode对象，封装了ROS节点、参数管理、线程管理等功能
  livox_ros::DriverNode livox_node;

  // 打印驱动版本信息（DRIVER_INFO为自定义日志宏，输出节点关联信息）
  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  // 数据传输格式：默认值kPointCloud2Msg（ROS PointCloud2消息格式）
  int xfer_format = kPointCloud2Msg;
  // 多话题发布开关：默认0（关闭，所有激光雷达数据发布到同一个话题）
  int multi_topic = 0;
  // 数据来源：默认值kSourceRawLidar（原始激光雷达数据，直接从硬件读取）
  int data_src = kSourceRawLidar;
  // 数据发布频率：默认10.0Hz
  double publish_freq  = 10.0; /* Hz */
  // 输出类型：默认值kOutputToRos（输出到ROS话题）
  int output_type      = kOutputToRos;
  // 坐标系ID：默认"livox_frame"，用于ROS消息的header.frame_id
  std::string frame_id = "livox_frame";
  // 是否保存激光雷达数据到ROS Bag：默认true
  bool lidar_bag = true;
  // 是否保存IMU数据到ROS Bag：默认false
  bool imu_bag   = false;

  // 从ROS参数服务器读取"xfer_format"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("xfer_format", xfer_format);
  // 从ROS参数服务器读取"multi_topic"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  // 从ROS参数服务器读取"data_src"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("data_src", data_src);
  // 从ROS参数服务器读取"publish_freq"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  // 从ROS参数服务器读取"output_data_type"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("output_data_type", output_type);
  // 从ROS参数服务器读取"frame_id"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("frame_id", frame_id);
  // 从ROS参数服务器读取"enable_lidar_bag"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  // 从ROS参数服务器读取"enable_imu_bag"参数，若不存在则保留默认值
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  // 打印数据来源参数值，用于调试确认参数配置
  printf("data source:%u.\n", data_src);

  // 校验发布频率，限制最大值为100.0Hz（避免频率过高导致系统负载过大）
  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } 
  // 校验发布频率，限制最小值为0.5Hz（避免频率过低导致数据延迟过大）
  else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } 
  // 频率在合法范围则保留原值
  else {
    publish_freq = publish_freq;
  }

  // 将exit_signal_（std::promise）的future对象赋值给future_，用于线程退出判断
  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  // 创建Lddc智能指针，初始化数据分发控制器
  // 传入参数：传输格式、多话题开关、数据来源、输出类型、发布频率、坐标系ID、bag开关
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  // 将DriverNode对象指针设置到Lddc中，使Lddc能访问ROS节点的发布器等资源
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  // 判断数据来源是否为原始激光雷达（硬件直连模式）
  if (data_src == kSourceRawLidar) {
    // 打印日志：确认数据来源为原始激光雷达
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    // 定义用户配置文件路径字符串
    std::string user_config_path;
    // 从ROS参数服务器读取用户配置文件路径（配置激光雷达IP、端口、型号等）
    livox_node.getParam("user_config_path", user_config_path);
    // 打印配置文件路径，用于调试确认配置文件位置
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    // 获取LdsLidar单例实例，传入发布频率（单例模式保证全局唯一数据源）
    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    // 将LdsLidar数据源注册到Lddc，使Lddc能获取激光雷达原始数据
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    // 初始化激光雷达数据源（读取配置文件、建立硬件连接、启动数据接收）
    if ((read_lidar->InitLdsLidar(user_config_path))) {
      // 初始化成功，打印日志
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      // 初始化失败，打印错误日志
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    // 数据来源参数无效，打印错误日志并提示检查launch文件
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  // 创建点云数据轮询线程，执行DriverNode的PointCloudDataPollThread成员函数
  // 线程智能指针管理，避免内存泄漏
  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  // 创建IMU数据轮询线程，执行DriverNode的ImuDataPollThread成员函数
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);

  // ROS主循环：只要ROS节点正常运行（未收到退出信号），就休眠10ms（10000微秒）
  // 保持节点存活，等待线程处理数据
  while (ros::ok()) { usleep(10000); }

  // 程序正常退出，返回0
  return 0;
}

/**
 * @brief DriverNode类成员函数：点云数据轮询线程入口
 * @details 循环分发点云数据，直到收到退出信号
 */
void DriverNode::PointCloudDataPollThread()
{
  // 定义future状态变量，用于判断是否收到退出信号
  std::future_status status;
  // 线程先休眠3秒，等待激光雷达硬件初始化、网络连接建立完成
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // 循环分发点云数据
  do {
    // 调用Lddc的分发函数，处理并发布点云数据（转换格式、发布到ROS话题、保存Bag等）
    lddc_ptr_->DistributePointCloudData();
    // 检查future_状态（非阻塞等待，超时时间0微秒），判断是否有退出信号
    status = future_.wait_for(std::chrono::microseconds(0));
  // 只要未收到退出信号（状态为timeout），就继续循环
  } while (status == std::future_status::timeout);
}

/**
 * @brief DriverNode类成员函数：IMU数据轮询线程入口
 * @details 循环分发IMU数据，直到收到退出信号
 */
void DriverNode::ImuDataPollThread()
{
  // 定义future状态变量，用于判断是否收到退出信号
  std::future_status status;
  // 线程先休眠3秒，等待IMU硬件初始化、数据链路建立完成
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // 循环分发IMU数据
  do {
    // 调用Lddc的分发函数，处理并发布IMU数据（转换格式、发布到ROS话题、保存Bag等）
    lddc_ptr_->DistributeImuData();
    // 检查future_状态（非阻塞等待，超时时间0微秒），判断是否有退出信号
    status = future_.wait_for(std::chrono::microseconds(0));
  // 只要未收到退出信号（状态为timeout），就继续循环
  } while (status == std::future_status::timeout);
}
```

### C++11 「Promise-Future（承诺 - 未来）」全解析

C++11 引入的 `std::promise` 和 `std::future` 是**异步编程的核心机制**，旨在解决多线程间**安全传递异步操作结果（值 / 异常）** 的问题，替代了传统共享内存 + 互斥锁 / 条件变量的繁琐写法。二者基于「共享状态（shared state）」协作，属于 `<future>` 头文件。

---

## 一、核心背景与设计目标

C++11 前，线程间通信主要依赖：

- 共享内存 + 互斥锁：需手动管理锁的生命周期，易出现死锁 / 数据竞争；
- 条件变量：需结合互斥锁，仅能传递 “就绪信号”，无法直接传递值 / 异常。

**Promise-Future 的目标**：

1. 解耦「生产结果的线程」和「消费结果的线程」；
2. 安全传递值或异常，无需手动管理同步原语；
3. 支持超时等待、异步结果获取等高级特性。

---

## 二、核心概念与角色分工

### 1. 共享状态（Shared State）

Promise 和 Future 共享一个线程安全的「共享状态」，用于存储：

- 待传递的值（或异常）；
- 状态标记（未就绪 / 就绪 / 已获取）；
- 同步原语（用于阻塞 / 唤醒线程）。

### 2. 核心角色

| 组件                      | 角色（写入端）          | 核心能力                                   |
| ----------------------- | ---------------- | -------------------------------------- |
| `std::promise<T>`       | 承诺者：生成结果 / 异常    | 设置值（`set_value`）、设置异常（`set_exception`） |
| `std::future<T>`        | 未来：消费结果 / 异常（独占） | 获取值（`get`）、等待就绪（`wait`）、超时等待           |
| `std::shared_future<T>` | 未来：消费结果 / 异常（共享） | 可拷贝，允许多线程同时获取结果                        |

**核心关系**：

- 一个 `promise` 仅能关联一个 `future`（通过 `promise.get_future()` 建立）；
- `promise` 写入结果后，共享状态变为「就绪」，`future` 可立即获取结果。

---

## 三、关键 API 详解

### 1. `std::promise<T>` 核心接口

| 函数                             | 功能说明                                                              |
| ------------------------------ | ----------------------------------------------------------------- |
| `get_future()`                 | 返回关联的 `std::future<T>`，**仅能调用一次**（否则抛 `future_already_retrieved`） |
| `set_value(const T&/T&&)`      | 设置值并将共享状态置为「就绪」，**仅能调用一次**（否则抛 `promise_already_satisfied`）       |
| `set_value()`（T=void）          | 无参数，仅标记 “完成信号”（无值传递）                                              |
| `set_exception(exception_ptr)` | 设置异常（如 `std::make_exception_ptr`），状态置为就绪                          |
| 析构函数                           | 若析构时共享状态未就绪，会设置 `broken_promise` 异常（后续 `future.get()` 抛此异常）       |
| 移动语义                           | 仅支持移动（不可拷贝），需用 `std::move` 传递                                     |

### 2. `std::future<T>` 核心接口

| 函数                       | 功能说明                                                   |
| ------------------------ | ------------------------------------------------------ |
| `get()`                  | 获取值 / 抛出异常，**仅能调用一次**（调用后 `valid()` 变为 `false`）；未就绪则阻塞 |
| `valid()`                | 检查是否关联有效共享状态（未调用 `get()` 且状态未失效）                       |
| `wait()`                 | 阻塞线程直到共享状态就绪（不获取值）                                     |
| `wait_for(duration)`     | 超时等待，返回 `future_status`（`ready`/`timeout`/`deferred`）  |
| `wait_until(time_point)` | 等待到指定时间点，返回值同 `wait_for`                               |
| `share()`                | 转换为 `std::shared_future<T>`，原 `future` 失效              |
| 移动语义                     | 仅支持移动（不可拷贝）                                            |

### 3. `std::shared_future<T>` 核心差异

- 可拷贝、可赋值，允许多线程共享同一个共享状态；
- `get()` 可调用多次（返回值的拷贝 /const 引用）；
- 其余接口与 `std::future<T>` 一致。

## 四、典型使用场景与示例

### 场景 1：基础线程间结果传递

```cpp
#include <iostream>
#include <thread>
#include <future>
#include <chrono>

// 生产结果的线程函数（接收promise，移动语义）
void compute_result(std::promise<int> prom) {
    try {
        // 模拟耗时计算
        std::this_thread::sleep_for(std::chrono::seconds(2));
        int result = 42;
        prom.set_value(result); // 设置结果，状态就绪
    } catch (...) {
        // 捕获所有异常并传递给future
        prom.set_exception(std::current_exception());
    }
}

int main() {
    // 1. 创建promise
    std::promise<int> calc_prom;
    // 2. 获取关联的future
    std::future<int> calc_fut = calc_prom.get_future();

    // 3. 启动线程，传递promise（必须move，不可拷贝）
    std::thread calc_thread(compute_result, std::move(calc_prom));

    // 4. 消费结果（阻塞直到就绪）
    std::cout << "等待计算结果..." << std::endl;
    try {
        int res = calc_fut.get(); // 获取值，仅能调用一次
        std::cout << "计算结果：" << res << std::endl; // 输出42
    } catch (const std::exception& e) {
        std::cerr << "异常：" << e.what() << std::endl;
    }

    // 等待线程结束
    calc_thread.join();
    return 0;
}

```

这段代码是 C++11 Promise-Future 机制的**非阻塞轮询 + 优雅退出**典型用法，核心是利用 `std::future::wait_for(0)` 实现 “持续执行业务逻辑，直到外部触发退出信号”。下面从**核心逻辑、关键 API、设计意图、潜在风险、完整上下文**等维度详细解析：

### 一、代码核心结构与背景

先明确代码的基本上下文：

- 这是类 `DriverNode` 的成员函数 `PointCloudDataPollThread`，是一个**独立线程的执行函数**（大概率通过 `std::thread` 启动）；
- 类成员变量：`lddc_ptr_`（点云数据分发的核心对象）、`future_`（`std::future` 类型，关联到某个 `std::promise`）；
- 核心逻辑：线程先休眠 3 秒，然后持续调用点云分发接口，直到 `future_` 对应的共享状态就绪，才终止循环。

### 二、逐行解析核心逻辑

```cpp
void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  // 步骤1：初始化延迟（工程化设计）
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  // 步骤2：do-while循环（先执行、后检查）
  do {
    // 核心业务：分发点云数据（比如从传感器读取/转发点云）
    lddc_ptr_->DistributePointCloudData();
    
    // 步骤3：非阻塞检查future_的状态
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout); // 未就绪则继续循环
}

```

```cpp
#include <iostream>
#include <thread>
#include <future>
#include <chrono>
#include <memory>

// 模拟点云分发类（仅示例）
class LDDC {
public:
    void DistributePointCloudData() {
        // 实际场景：从传感器读取点云、转发给其他模块、写入缓存等
        static int count = 0;
        std::cout << "[点云分发] 第" << ++count << "次分发" << std::endl;
        // 加短延迟避免CPU占满（工程必加！）
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
};

class DriverNode {
private:
    std::unique_ptr<LDDC> lddc_ptr_;
    std::future<void> future_;       // 用于检查退出信号
    std::promise<void> promise_;     // 关联的promise（触发退出）
    std::thread poll_thread_;        // 点云分发线程

public:
    DriverNode() : lddc_ptr_(std::make_unique<LDDC>()) {
        // 关联promise和future（核心：共享状态绑定）
        future_ = promise_.get_future();
        // 启动点云分发线程
        poll_thread_ = std::thread(&DriverNode::PointCloudDataPollThread, this);
    }

    // 外部调用：触发线程退出（设置共享状态就绪）
    void StopPolling() {
        promise_.set_value(); // 无值传递，仅标记“退出信号”
    }

    // 等待线程退出
    void JoinPollThread() {
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
    }

    // 析构：保证线程优雅退出
    ~DriverNode() {
        StopPolling();
        JoinPollThread();
    }

    // 核心轮询线程函数（用户提供的代码）
    void PointCloudDataPollThread() {
        std::future_status status;
        // 初始化等待：等硬件/驱动就绪
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        do {
            // 核心业务：分发点云
            lddc_ptr_->DistributePointCloudData();
            // 非阻塞检查：是否收到退出信号
            status = future_.wait_for(std::chrono::microseconds(0));
        } while (status == std::future_status::timeout);

        std::cout << "[退出] 点云分发线程终止" << std::endl;
    }
};

// 测试主函数
int main() {
    DriverNode node;
    // 主线程运行5秒后，触发点云线程退出
    std::this_thread::sleep_for(std::chrono::seconds(5));
    node.StopPolling();
    node.JoinPollThread();
    return 0;
}

```

### 心设计逻辑补充

1. **信号量同步**：`pcd_semaphore_.Wait()` 是典型的 “生产者 - 消费者” 模型 ——Lds（生产者）读取硬件数据后释放信号量，Lddc（消费者）等待信号量后处理数据，避免无数据时的空循环，大幅降低 CPU 占用。
2. **多雷达兼容**：通过遍历 `lidar_count_` 和 `lidars_` 数组，支持多雷达组网场景，每个雷达独立处理数据，保证数据流隔离。
3. **状态校验**：多层校验（数据源注册、退出请求、连接状态、队列有效性）是工程上的 “防御性编程”，避免硬件异常（如突然断开、初始化失败）导致程序崩溃。
4. **职责拆分**：`DistributePointCloudData` 仅负责 “遍历 + 状态检查 + 分发入口”，具体的数据转换 / 发布逻辑交给 `PollingLidarPointCloudData`，符合 “单一职责原则”，便于维护和扩展。

```c
/**
 * @brief Lddc类的核心方法：分发所有已连接激光雷达的点云数据
 * @details 遍历所有注册的激光雷达设备，检查设备状态后处理并分发点云数据，是数据从硬件到ROS话题的核心流转入口
 */
void Lddc::DistributePointCloudData(void) {
  // 1. 检查激光雷达数据源（Lds）是否已注册
  // lds_是Lddc类中保存的Lds（Lidar Data Source）实例指针，负责读取硬件/文件的原始数据
  // 若未注册（空指针），说明数据源未初始化，打印提示并直接返回，避免空指针访问
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }

  // 2. 检查数据源是否触发退出请求
  // IsRequestExit()是Lds类的方法，用于判断是否收到数据源退出指令（如ROS节点关闭、硬件断开）
  // 若已请求退出，打印提示并返回，终止本次数据分发
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }
  
  // 3. 阻塞等待点云数据信号量
  // pcd_semaphore_是Lds类中的信号量（Semaphore），用于同步原始数据的生产/消费：
  // - 数据源（Lds）读取到新的点云数据后，会释放信号量；
  // - 此处Wait()会阻塞当前线程，直到有新数据可用，避免无意义的空循环，减少CPU占用
  lds_->pcd_semaphore_.Wait();

  // 4. 遍历所有已识别的激光雷达设备（lidar_count_是雷达总数）
  // 支持多雷达组网场景，逐个处理每个雷达的点云数据
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    // 5. 将循环索引转换为雷达ID（语义化命名，提升代码可读性）
    // lidar_id与硬件注册的雷达ID一一对应，用于区分不同雷达的数据流
    uint32_t lidar_id = i;

    // 6. 获取当前雷达设备的指针
    // lds_->lidars_是存储所有雷达设备的数组（LidarDevice类型），每个元素对应一个雷达的状态和数据
    LidarDevice *lidar = &lds_->lidars_[lidar_id];

    // 7. 获取当前雷达的点云数据队列指针
    // LidarDevice::data是LidarDataQueue类型的队列，存储该雷达的原始点云数据（生产-消费模型）
    LidarDataQueue *p_queue = &lidar->data;

    // 8. 检查雷达连接状态和数据队列有效性
    // - kConnectStateSampling：雷达处于"采样状态"（已连接且正常输出数据），非该状态则跳过（如未连接、初始化中）
    // - p_queue == nullptr：防御性检查，避免空队列访问
    // 满足任一条件则跳过当前雷达，处理下一个
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }

    // 9. 处理并分发当前雷达的点云数据
    // PollingLidarPointCloudData是核心处理函数：
    // - 从p_queue中读取原始点云数据；
    // - 转换为ROS标准的PointCloud2消息；
    // - 发布到对应ROS话题（单话题/多话题由配置决定）；
    // - 若开启Bag保存，将数据写入Bag文件
    PollingLidarPointCloudData(lidar_id, lidar);    
  }
}

```

### 核心设计逻辑补充

1. **双层空指针校验**：
- 上层 `DistributePointCloudData` 已检查过队列有效性，但本函数仍做二次校验 —— 这是 “防御性编程” 的典型体现，避免上层逻辑修改后导致本函数空指针访问（比如上层漏检、多线程修改队列指针）。

- **循环条件的优先级**：

- `!lds_->IsRequestExit()` 放在循环条件首位，保证 “退出请求” 能被立即响应（即使队列还有数据），避免退出指令被阻塞，符合线程 “优雅退出” 的设计目标。

- **传输格式的可扩展性**：

- 通过 `if-else` 分支适配三种主流点云格式，后续若新增格式（如 `kRosBagMsg`），只需新增分支即可，符合 “开闭原则”（对扩展开放、对修改关闭）。

- **职责单一性**：

- 本函数仅负责 “格式判断 + 调用对应发布函数”，不处理具体的格式转换 / 发布逻辑 —— 转换和发布交给 `PublishPointcloud2`/`PublishCustomPointcloud` 等函数，降低代码耦合度，便于维护（比如修改 PointCloud2 格式的字段，只需改 `PublishPointcloud2`）。

- **队列消费逻辑**：

`while` 循环会持续消费队列中的所有数据，直到队列为空或收到退出请求 —— 保证原始数据包不积压，避免内存占用持续升高。

```cpp
/**
 * @brief 处理单个激光雷达的点云数据队列，按配置的传输格式发布数据
 * @param index 激光雷达的索引（对应多雷达组网中的唯一标识）
 * @param lidar 指向当前激光雷达设备的指针（包含数据队列、设备状态等信息）
 */
void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  // 1. 获取当前雷达设备的点云数据队列指针
  // lidar->data 是LidarDevice类中存储原始点云数据包的队列（LidarDataQueue类型）
  // 后续所有操作基于该队列，先缓存指针提升代码可读性，避免重复写 lidar->data
  LidarDataQueue *p_queue = &lidar->data;

  // 2. 防御性空指针检查：避免访问无效队列导致程序崩溃
  // - p_queue == nullptr：队列本身为空（理论上不会触发，上层已检查，但双层校验更安全）
  // - p_queue->storage_packet == nullptr：队列的核心存储缓冲区为空（无可用数据包）
  // 满足任一条件则直接返回，终止当前雷达的数据处理
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  // 3. 循环处理队列中的点云数据，循环终止条件：
  // - !lds_->IsRequestExit()：数据源未收到退出请求（如ROS节点关闭、硬件断开）
  // - !QueueIsEmpty(p_queue)：数据队列非空（还有未处理的原始数据包）
  // 设计逻辑：优先响应退出请求，保证线程优雅退出；队列为空时停止循环，避免无意义空转
  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    // 4. 根据配置的传输格式（transfer_format_），选择对应的发布函数
    // transfer_format_是Lddc类的成员变量，由ROS参数（xfer_format）初始化，支持三种格式：
    // - kPointCloud2Msg：ROS标准PointCloud2消息（最常用，兼容ROS生态）
    if (kPointCloud2Msg == transfer_format_) {
      // 将原始数据包转换为PointCloud2格式，发布到对应ROS话题
      PublishPointcloud2(p_queue, index);
    } 
    // - kLivoxCustomMsg：Livox自定义点云消息（保留Livox原始数据特征，如时间戳、线数）
    else if (kLivoxCustomMsg == transfer_format_) {
      // 将原始数据包转换为Livox自定义格式，发布到对应ROS话题
      PublishCustomPointcloud(p_queue, index);
    } 
    // - kPclPxyziMsg：PCL库的XYZI格式消息（兼容PCL点云处理库）
    else if (kPclPxyziMsg == transfer_format_) {
      // 将原始数据包转换为PCL XYZI格式，发布到对应ROS话题
      PublishPclMsg(p_queue, index);
    }
  }
}

```

### 核心设计逻辑补充

1. **数据消费模型**：
- 采用 “循环 + 弹出 + 处理” 的消费模式，保证队列无数据积压 —— 原始数据包由 `Lds`（数据源）生产并推入队列，本函数作为消费者全量处理，符合经典的 “生产者 - 消费者” 设计模式。

- **Livox 自定义消息的价值**：

- ROS 标准 `PointCloud2` 消息会丢失 Livox 雷达的特有属性（如单一点的高精度时间戳、激光线号、测距置信度等），而 `CustomMsg` 完整保留这些信息，适合需要精细化分析雷达数据的场景（如多雷达时间同步、运动补偿）。

- **分层处理思想**：

- 函数内部分为 “数据校验→消息初始化→数据填充→消息发布” 四步，每一步职责单一：
  
  - 校验：过滤无效数据；
  
  - 初始化：填充元数据；
  
  - 填充：转换原始点数据；
  
  - 发布：完成 ROS 话题推送；
    
    分层设计降低了代码耦合，便于单独修改某一步逻辑（如调整坐标转换规则只需改 `FillPointsToCustomMsg`）。

- **容错设计**：

对空数据包的校验是工程上的关键容错手段 —— 硬件偶尔会产生空包（如网络抖动、雷达启动初期），直接跳过可避免程序崩溃，保证驱动的鲁棒性。

```cpp
/**
 * @brief 将激光雷达原始数据包转换为Livox自定义格式消息，并发布到ROS话题
 * @param queue 指向当前雷达点云数据队列的指针（存储待处理的原始数据包）
 * @param index 雷达索引（多雷达场景下区分不同雷达的话题/消息标识）
 * @note Livox自定义消息（CustomMsg）保留了硬件原始数据特征（如高精度时间戳、线号、测距值等），
 *       相比ROS标准PointCloud2更适配Livox雷达的特有属性
 */
void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  // 1. 循环消费数据队列中的所有原始数据包，直到队列为空
  // 设计逻辑：保证队列中的数据包被全部处理，避免数据积压；
  // 外层未加退出检查（如IsRequestExit），因上层PollingLidarPointCloudData已做退出校验
  while(!QueueIsEmpty(queue)) {
    // 2. 定义原始数据包对象，用于接收从队列弹出的数据
    // StoragePacket是Livox驱动自定义的结构体，存储单帧原始点云数据（包含points数组、时间戳、包长度等）
    StoragePacket pkg;

    // 3. 从数据队列中弹出一个原始数据包，写入pkg对象
    // QueuePop是队列的消费接口，弹出后队列中该数据包被移除（生产-消费模型的核心消费操作）
    QueuePop(queue, &pkg);

    // 4. 检查数据包中的点云数据是否为空
    // pkg.points是存储原始点的容器（如vector），为空说明数据包无效，打印错误日志并跳过当前包
    // 防御性校验：避免空数据导致后续消息初始化/填充崩溃
    if (pkg.points.empty()) {
      printf("Publish custom point cloud failed, the pkg points is empty.\n");
      continue;
    }

    // 5. 定义Livox自定义消息对象，作为最终发布的消息载体
    // CustomMsg是Livox驱动封装的ROS消息类型（非ROS标准），包含：
    // - header：ROS标准消息头（frame_id、时间戳）
    // - points：自定义点结构体（含x/y/z/intensity/line_id/timestamp等）
    // - lidar_id：雷达编号（多雷达组网标识）
    CustomMsg livox_msg;

    // 6. 初始化自定义消息的基础字段（消息头、雷达ID、时间戳等）
    // 参数说明：
    // - livox_msg：待初始化的消息对象
    // - pkg：原始数据包（提供时间戳、雷达属性等基础信息）
    // - index：雷达索引（映射为lidar_id，区分多雷达）
    // 核心操作：填充frame_id、消息时间戳、雷达ID等元数据，保证消息的ROS兼容性
    InitCustomMsg(livox_msg, pkg, index);

    // 7. 将原始数据包中的点云数据填充到自定义消息中
    // 核心转换逻辑：
    // - 遍历pkg.points中的原始点数据；
    // - 将原始点的x/y/z/强度/线号等属性映射到livox_msg.points的对应字段；
    // - 处理坐标转换（如雷达坐标系到ROS世界坐标系）、数据格式校准（如毫米转米）
    FillPointsToCustomMsg(livox_msg, pkg);

    // 8. 将填充完成的自定义消息发布到对应的ROS话题
    // 核心发布逻辑：
    // - 根据index（雷达ID）确定发布话题（单话题：/livox/custom；多话题：/livox/custom_0、/livox/custom_1）；
    // - 调用ROS发布器（publisher）的publish方法发送消息；
    // - 若开启Bag保存，同时将消息写入Bag文件
    PublishCustomPointData(livox_msg, index);
  }
}

```
