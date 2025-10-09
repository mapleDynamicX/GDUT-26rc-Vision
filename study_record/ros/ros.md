# ROS

## ROS1 中，指定 `rosbag` 文件的发布频率

```
rosbag play -r <倍率> your_bag.bag
```

> - **作用​**​：对整个 bag 文件的播放速度进行缩放。
> - ​**​示例​**​：
>   - `-r 0.5`：以原始速度的 50% 慢速播放。
>   - `-r 2.0`：以原始速度的 2 倍快速播放。
> - ​**​原理​**​：修改所有消息的时间戳间隔，实现整体加速/减速。

示例：播放`example.bag`，并以2倍速播放， rosbag play -r 2.0 example.bag

## 回调函数

当多个消息同时到达时，​**​回调函数默认不会并行执行​**​

### **多线程处理​**​

```cpp
ros::AsyncSpinner spinner(N); // N = 线程数（建议与回调数量匹配）
spinner.start();
ros::waitForShutdown();
```

共享数据需用​**​互斥锁（`std::mutex`）​**​ 保护

## 类和对象

**发布者（Publisher）**

> 发布者只需要一个话题名称和消息类型，它负责发布消息。当我们调用发布者的`publish()`方法时，通常是在类的某个成员函数中主动调用，比如在定时器回调或接收到订阅消息后的处理函数中。因此，发布者本身不需要绑定回调函数，所以不需要特别绑定类对象（但发布者作为类的成员变量，自然属于该对象）

**订阅者（Subscriber）**

> 订阅者需要指定一个回调函数，当接收到消息时，ROS会调用这个回调函数。如果我们在类中定义订阅者，我们通常希望回调函数能够访问类的成员（变量或函数）。因此，我们需要将这个回调函数绑定到类的实例（对象）上。如果不这样做，回调函数将无法访问类的非静态成员（因为非静态成员需要通过对象来访问）。

**举例**

```cpp
// 构造函数初始化
MyNode::MyNode() {
    // 1. 发布者初始化 (简单初始化)
    pub = nh.advertise<std_msgs::String>("output_topic", 10);
    // 2. 订阅者初始化 - 关键绑定步骤！
    sub = nh.subscribe(
        "input_topic",              // 话题名
        10,                         // 队列长度
        &MyNode::subCallback,       // 成员函数指针
        this                        // ★★★ 绑定当前对象 ★★★
    );
}
// 订阅回调函数
void MyNode::subCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());

    // 使用发布者（直接通过this调用）
    std_msgs::String new_msg;
    new_msg.data = "Echo: " + msg->data;
    pub.publish(new_msg);  // ✓ 可直接访问成员pub
}
```

# ROS1 定时器详解：`nh.createTimer`

```cpp
timer_ = nh.createTimer(ros::Duration(1 / freq_localization_),
                        std::bind(&GlobalLocalization::threadLocalization, this),
                        false, true);
```

这是ROS1中创建定时器的标准方法，功能是​**​以指定频率周期性地调用成员函数​

### 1. 时间间隔（`ros::Duration`）

```cpp
ros::Duration(1 / freq_localization_)​
```

> - **作用​**​：定义定时器触发的时间间隔（单位：秒）
> - ​**​计算​**​：
>   - `freq_localization_`：期望的调用频率（单位：Hz）
>   - `1 / freq_localization_`：计算两次调用间的时间间隔
> - ​**​示例​**​：
>   - 若 `freq_localization_ = 10.0` (10Hz)，则间隔为0.1秒
>   - 若 `freq_localization_ = 1.0` (1Hz)，则间隔为1.0秒

### 2. 回调函数（`std::bind`）

```cpp
std::bind(&GlobalLocalization::threadLocalization, this)
```

> - ​**​作用​**​：绑定定时器触发时要调用的成员函数
> - ​**​解析​**​：
>   - `&GlobalLocalization::threadLocalization`：成员函数指针
>   - `this`：绑定当前对象实例
> - ​**​特殊说明​**​：
>   - 这种写法表示`threadLocalization`函数​**​不需要任何参数​**​
>   - 如果需要带参数（如默认的`TimerEvent`），需使用占位符：

​**​普通函数​**​

```cpp
// 函数名可自动转换为函数指针
void freeFunc(int); 
subscribe("topic", freeFunc); // ✅ 合法
```

**成员函数​**​：

```cp
class MyClass {
public:
   void memberFunc(int);
};
// 成员函数隐含`this`参数
MyClass obj;
subscribe("topic", obj.memberFunc); // ❌ 错误！不能直接使用
//为何需要 & 和 this
_cloud_sub = _nh.subscribe(_cloud_sub_name, 10, 
                         &registration2::cloudCallback, // 必须加&
                         this);                         // 必须提供对象实例
```

> - ​**​类型特殊性​**​：
>   
>   - 成员函数指针类型为 `ReturnType (Class::*)(Params...)`（类作用域限定）
>   - 普通函数指针类型为 `ReturnType (*)(Params...)`（全局作用域）

### 3. 一次性执行标志（布尔值）

```cpp
false
```

> - **作用​**​：控制定时器触发模式
> - ​**​值解析​**​：
>   - `true`：只执行一次（类似`setTimeout`）
>   - `false`：周期性执行（类似`setInterval`）← ​**​本例使用模式​**​
> - ​**​应用场景​**​：
>   - `true`：适用于初始化后只需要执行一次的任务
>   - `false`：适用于需要持续周期性处理的任务

### 4. 自动启动标志（布尔值）

```cpp
true
```

> - **作用​**​：控制定时器创建后是否立即启动
> - ​**​值解析​**​：
>   - `true`：创建后立即启动（默认启用）
>   - `false`：需要手动调用`timer.start()`启动
> - ​**​应用场景​**​：
>   - `true`：默认情况，创建即工作
>   - `false`：需要等待其他条件满足后才启动的场景

## 定时器操作完整方法

### 创建后操作定时器：

```cpp
timer_.stop();      // 停止定时器
timer_.start();     // 重启定时器
timer_.setPeriod(ros::Duration(0.2));  // 动态修改时间间隔
```

## 定时器的生命周期管理

### 作用域要求

```cpp
class GlobalLocalization {
private:
    ros::Timer timer_;  // 必须作为成员变量！不能是局部变量
public:
    void init() {
        // ❌ 错误：局部变量会导致回调无法持续
        // ros::Timer local_timer = nh.createTimer(...);

        // ✅ 正确：作为成员变量
        timer_ = nh.createTimer(...);
    }
};
```

> ### 生命周期注意事项
> 
> - ​**​保持有效​**​：定时器对象需要在所有回调执行期间保持有效
> - ​**​节点关闭​**​：定时器会随着节点关闭自动销毁
> - ​**​提前销毁​**​：可通过`timer_.stop()`停止并释放资源

### ROS1 中的服务（Service）消息详解

#### 1. 服务消息的作用

> 服务消息是 ROS1 中实现 ​**​同步请求/响应（Request/Response）通信机制​**​的核心组件，与异步的发布/订阅（Topic）机制形成互补：
> 
> 1. ​**​同步通信​**​：客户端发出请求后阻塞等待服务端响应
> 2. ​**​精确控制​**​：适用于需要确认执行结果的操作
> 3. ​**​任务型交互​**​：完成特定任务后返回结果
> 4. ​**​避免轮询​**​：替代通过 Topic 不断查询状态的低效方式

#### 2. 通信流程

![](/home/maple/笔记/images/2025-07-10-10-37-07-ros1服务.png)

### **Eigen (强烈推荐)**

> - **核心优势​**​：ROS和PCL默认依赖的线性代数库，无额外安装成本
> - ​**​特点​**​：
>   - 头文件库（无需编译）
>   - 支持矩阵运算、分解、几何变换等（如SVD、QR分解）
>   - 与ROS/PCL深度集成
> - ​**​ROS1集成​**​：
>   - `tf`和`tf2`直接使用Eigen处理坐标变换
>   - `geometry_msgs/Pose`与Eigen转换工具：`tf2_eigen`
> - ​**​PCL集成​**​：
>   - 点云类型（如`pcl::PointXYZ`）与Eigen互相转换
>   - 特征值计算（如法线估计）、ICP配准基于Eigen

## .toSec()

```cpp
ros::Time start = ros::Time::now();
// 执行某些操作
ros::Time end = ros::Time::now();
ros::Duration duration = end - start;
double duration_sec = duration.toSec(); // 转换为秒
```

### 1. ​**​消息分发机制​**​

- 当发布者向话题发送一条消息时，ROS核心（TCPROS/UDPROS层）会​**​同时复制​**​该消息到所有连接的订阅者。

- 理论上，所有订阅节点应​**​同时收到消息​**​，因为ROS不会主动为订阅者排序。

### 2. ​**​实际接收顺序的差异​**​

- ​**​网络延迟​**​：不同节点可能因网络路径不同导致消息到达时间略有差异（尤其在分布式系统中）。

- ​**​节点负载​**​：如果订阅者节点的CPU繁忙，处理消息的Callback被调用的时间可能有微小延迟。

- ​**​线程调度​**​：ROS使用多线程处理回调，线程调度可能导致某个订阅者的回调函数稍晚执行（例如：使用`AsyncSpinner`时）。

## 在 ROS1（特别是 C++）中，​可以在子线程中使用 ros::spin()，但这需要谨慎处理，因为不正确的使用会导致阻塞或资源冲突问题。以下是关键原因和注意事项：

### 1. ​**​主线程与 `ros::spin()`的典型用法​**​

- ROS 的默认设计中，`ros::spin()`是一个​**​阻塞函数​**​，它会循环处理回调队列直到节点关闭。

- 如果直接在 `main()`的主线程调用 `ros::spin()`，主线程会被阻塞，无法执行其他任务。

### 2. ​**​潜在问题与风险​**​

- ​**​阻塞子线程​**​：`ros::spin()`会独占子线程，导致该线程无法处理其他任务。

- ​**​多线程竞争​**​：
  
  - ROS 的回调是线程安全的，但​**​您的回调函数本身需要确保线程安全​**​。
  
  - 如果回调函数访问共享资源（如全局变量、类成员），需使用互斥锁（如 `std::mutex`）。

- ​**​资源泄漏​**​：未正确关闭线程可能导致 ROS 回调队列积压。

### 3. ​**​更推荐的做法：`AsyncSpinner`​**​

```cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    // 创建 AsyncSpinner 使用 4 个线程处理回调
    ros::AsyncSpinner spinner(4); 
    spinner.start();  // 非阻塞，后台处理回调

    // 主线程继续执行其他任务
    while (ros::ok()) {
        // ... 主线程逻辑 ...
    }
    ros::waitForShutdown(); // 等待节点关闭
    return 0;
}
```

```cpp
// ROS头文件，包含ROS系统的基本功能和类声明
#include <ros/ros.h>

// 主函数，ROS节点的标准入口点
int main(int argc, char** argv) {
  // 初始化ROS系统，设置节点名称为"node_name"
  // 节点名在ROS网络中必须是唯一的标识符
  ros::init(argc, argv, "node_name");

  // 创建节点句柄，用于管理ROS资源（如订阅者/发布者/服务）
  // 这是与ROS系统通信的主要接口点
  ros::NodeHandle nh;

  // 创建异步spinner对象，指定使用4个工作线程处理回调
  // 多线程模式允许同时处理多个订阅消息和服务请求
  ros::AsyncSpinner spinner(4); // 或 MultiThreadedSpinner

  // 启动异步spinner，使回调函数能在后台线程执行
  // 开启后不会阻塞主线程的执行
  spinner.start();

  // ... 此处添加ROS对象的初始化代码 ...
  // 例如：ros::Subscriber, ros::Publisher, ros::ServiceServer 等
  // 创建的所有消息收发对象将共享工作线程池

  // 阻塞主线程直到节点被关闭（如Ctrl+C或ros::shutdown()调用）
  // 保证节点运行时不会立即退出
  ros::waitForShutdown();

  // 当ros::shutdown()被调用时，spinner会自动停止
  // 所有工作线程将有序退出，无需手动停止
  return 0;
}
```

## 法二

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
void callback1(const std_msgs::String::ConstPtr& msg)
{
    // 处理消息的逻辑
    ROS_INFO("Received message in callback1: %s", msg->data.c_str());
}

void callback2(const std_msgs::String::ConstPtr& msg)
{
    // 处理消息的逻辑
    ROS_INFO("Received message in callback2: %s", msg->data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_thread_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("topic1", 10, callback1);
    ros::Subscriber sub2 = nh.subscribe("topic2", 10, callback2);

    ros::MultiThreadedSpinner spinner(2); // 2个线程
    spinner.spin();

    return 0;
}
```

在 ROS 1（C++）中，`ros::AsyncSpinner`和 `ros::MultiThreadedSpinner`是两种用于异步处理回调队列（Callback Queue）的工具，主要区别在于​**​线程管理策略​**​。以下是它们的作用和核心区别的详细说明：

### **​一、作用概述​**​

ROS 中的节点通过订阅者（Subscriber）、服务端（Service Server）等对象注册回调函数（如 `callback()`），这些回调会被存入节点的​**​回调队列​**​中。`Spinner`的核心作用是​**​驱动回调队列的执行​**​，确保回调函数被及时调用。

`ros::AsyncSpinner`和 `ros::MultiThreadedSpinner`均为 `ros::Spinner`的派生类，旨在解决单线程 `ros::spin()`可能导致的阻塞问题（例如主线程被回调处理卡住，无法执行其他任务），但通过不同的线程策略实现更灵活的回调执行。

### ​**​二、核心区别​**​

#### ​**​1. ros::AsyncSpinner​**​

- ​**​线程模型​**​：
  
  仅使用​**​1个后台线程​**​处理回调队列。主线程可以继续执行其他任务（如计算、交互），而回调函数在独立的后台线程中被顺序执行（类似单线程 `ros::spin()`，但与主线程解耦）。

​**​关键特性​**​：

- 回调函数按​**​入队顺序​**​被处理（单线程串行），无并行性。

- 主线程与回调线程分离，适合需要主线程保持响应的场景（例如 GUI 或实时控制）。

- - 需显式调用 `start()`启动后台线程，并可通过 `stop()`终止（需注意线程安全）。

- ​**​典型场景​**​：
  
  主线程需要执行耗时操作（如图像渲染、用户输入处理），同时希望回调函数被及时处理，但对回调并行无需求。

#### ​**​2. ros::MultiThreadedSpinner​**​

- ​**​线程模型​**​：
  
  使用​**​多个线程​**​（默认线程数为 CPU 核心数，可通过构造函数指定）并行处理回调队列。多个回调函数可同时在不同的线程中执行（取决于主题/服务的订阅关系和回调的独立性）。

​**​关键特性​**​：

- 回调函数​**​并行执行​**​（多线程），可充分利用多核 CPU，提升处理效率（尤其适合计算密集型回调）。

- 若多个回调访问共享资源（如全局变量），需手动添加互斥锁（`std::mutex`）避免竞态条件（Race Condition）。

- - 调用 `spin()`时会阻塞当前线程（通常是主线程），直到 `stop()`被调用或节点关闭（与单线程 `ros::spin()`类似，但内部用多线程处理回调）。

- ​**​典型场景​**​：
  
  回调函数计算量大（如图像特征提取、SLAM 计算），或需要同时处理多个独立主题的消息（如同时订阅摄像头和激光雷达数据）。

### ​**​三、对比总结​**​

| ​**​特性​**​     | `ros::AsyncSpinner`    | `ros::MultiThreadedSpinner` |
| -------------- | ---------------------- | --------------------------- |
| ​**​线程数​**​    | 固定1个后台线程               | 可指定（默认 CPU 核心数）             |
| ​**​回调执行方式​**​ | 单线程串行                  | 多线程并行（取决于回调独立性）             |
| ​**​主线程阻塞​**​  | 不阻塞（`start()`后主线程自由执行） | 阻塞（`spin()`调用线程等待回调处理完成）    |
| ​**​并行能力​**​   | 无                      | 有（适合多核优化）                   |
| ​**​线程安全风险​**​ | 低（单线程无竞态）              | 高（需手动处理共享资源同步）              |
| ​**​典型用途​**​   | 主线程需保持响应（如 GUI）        | 回调计算密集或需并行处理多消息             |

此前可能存在的误解需要澄清：​**​`ros::AsyncSpinner`并非固定使用 1 个线程​**​，而是通过构造函数参数指定线程池的大小（即后台线程的数量）。例如 `ros::AsyncSpinner spinner(4);`会创建一个包含 4 个线程的线程池，这些线程共同从节点的回调队列中取出回调函数并执行。

**​二、手动创建线程​**​

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
//​​步骤二：定义回调函数（同第一种方法类似）​​
void callback1(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message in callback1: %s", msg->data.c_str());
}

void callback2(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message in callback2: %s", msg->data.c_str());
}
//​​步骤三：定义线程函数，在线程函数中创建NodeHandle并订阅话题​​
void thread1_function()
{
    ros::NodeHandle nh1;
    ros::Subscriber sub1 = nh1.subscribe("topic1", 10, callback1);
    ros::spin();
}

void thread2_function()
{
    ros::NodeHandle nh2;
    ros::Subscriber sub2 = nh2.subscribe("topic2", 10, callback2);
    ros::spin();
}
//​步骤四：在main函数中创建并运行线程​​
int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_multi_thread_subscriber");

    std::thread t1(thread1_function);
    std::thread t2(thread2_function);

    t1.join();
    t2.join();

    return 0;
}
```

## 将geometry_msgs::TransformStamped::ConstPtr& msg  转化为变化矩阵

### 示例1.

```cpp
// 将TransformStamped转换为4x4齐次变换矩阵
Eigen::Matrix4d transformToMatrix(const geometry_msgs::TransformStamped& transform)
{
    // 提取平移分量
    Eigen::Vector3d translation;
    translation.x() = transform.transform.translation.x;
    translation.y() = transform.transform.translation.y;
    translation.z() = transform.transform.translation.z;

    // 提取旋转四元数
    Eigen::Quaterniond rotation;
    rotation.x() = transform.transform.rotation.x;
    rotation.y() = transform.transform.rotation.y;
    rotation.z() = transform.transform.rotation.z;
    rotation.w() = transform.transform.rotation.w;

    // 创建4x4齐次变换矩阵
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    // 设置左上角3x3旋转部分，（四元数转欧拉角）
    matrix.block<3,3>(0,0) = rotation.toRotationMatrix();
    // 设置右上角3x1平移部分
    matrix.block<3,1>(0,3) = translation;

    return matrix;
}
```

### 示例2.

```cpp
void transformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // 1. 解析时间戳和坐标系ID
    ROS_INFO("Received transform from [%s] to [%s]", 
             msg->header.frame_id.c_str(), 
             msg->child_frame_id.c_str());

    // 2. 提取平移和旋转数据
    Eigen::Vector3d translation(
        msg->transform.translation.x,
        msg->transform.translation.y,
        msg->transform.translation.z
    );

    Eigen::Quaterniond rotation(
        msg->transform.rotation.w,
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z
    );

    // 3. 创建4x4齐次变换矩阵
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = translation;
    transform.rotate(rotation);

    // 获取变换矩阵的4x4矩阵形式
    Eigen::Matrix4d transformation_matrix = transform.matrix();

}
```

## 扫描IP地址

```bas
sudo arp-scan -l
```

## launch文件示例

```xml
<launch>

  <node pkg="yolopkg" type="yolo_detector_node" name="yolo_detector" output="screen">

    <param name="model_path" value="/home/robocon/RC/project6_ws/src/yolopkg/best_openvino_model/best"/>
    <param name="conf_threshold" value="0.7"/>
    <param name="iou_threshold" value="0.5"/>


    <param name="image_topic" value="/camera/color/image_raw"/>


    <param name="queue_size" value="2"/>
    <param name="num_threads" value="2"/>
  </node> 
  <node pkg="publishpkg" type="publisher.py" name="publisher" output="screen"/>
  <node pkg="slampkg" type="serial_node" name="serial_node" output="screen"/>
  <node pkg="bridgepkg" type="protected_node" name="protected_node" output="screen"/>
  <!--<node pkg="yolopkg" type="realsense_publisher.py" name="realsense_publisher" output="screen"/>--> 
  <!--<node pkg="yolopkg" type="sub_bbox_node" name="sub_bbox_node" output="screen"/>-->
</launch>
```

## 启动rviz(并加载已有.rviz文件)

```xml
<node pkg="rviz" type="rviz" name="rviz" args="-d /home/maple/study2/world_ws4/src/navigation/nav.rviz" />
```

## 将话题数据保存到txt

```bash
rostopic echo /map >> map1.txt
```

### **​消息同步策略与同步器​**​

```cpp
// 定义近似时间同步策略：同步两个sensor_msgs/Image消息（RGB和深度图）
// 允许时间戳在指定容差内近似匹配（默认容差由策略参数决定）
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;  

// 定义同步器类型，使用上述策略同步两个图像话题
typedef message_filters::Synchronizer<SyncPolicy> Sync;
class bridge
{
public:
    // 构造函数：初始化节点句柄、图像传输、订阅器、同步器和发布器
    bridge():_camerainfo(), _nh("/"), _it(_nh)  // 初始化成员变量（_camerainfo使用默认构造）
    {
        // 广告点云话题"rgb_cloud"，队列大小10（发布点云消息）
        _pub_cloud = _nh.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 10);  

        get_param();  // 从参数服务器获取RGB/深度图像话题名等参数

        // 订阅RGB图像话题：使用image_transport的SubscriberFilter（支持消息过滤）
        // 第二个参数是话题名（_rgb_sub_name），第三个是队列大小
        _rgb_sub.subscribe(_it, _rgb_sub_name, 10);  

        // 订阅深度图像话题，参数同上
        _depth_sub.subscribe(_it, _depth_sub_name, 10);  

        // 创建同步器实例：使用ApproximateTime策略，队列大小10，绑定两个订阅器
        sync_.reset(new Sync(SyncPolicy(10), _rgb_sub, _depth_sub));  

        // 注册同步回调函数：当两个图像时间同步时，触发imageCallback
        // boost::bind绑定成员函数，_1和_2是占位符，对应rgb_msg和depth_msg
        sync_->registerCallback(boost::bind(&bridge::imageCallback, this, _1, _2));  
    }

    // 声明函数：从参数服务器获取RGB/深度图像话题名等参数
    void get_param();  

    // 声明回调函数：处理同步后的RGB和深度图像，生成点云
    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg);  

private:
    camerainfo _camerainfo;  // 相机信息处理对象（用于获取内参、畸变等）
    ros::NodeHandle _nh;  // ROS节点句柄
    image_transport::ImageTransport _it;  // 图像传输工具（管理图像话题）
    std::string _rgb_sub_name;  // RGB图像话题名（从参数服务器读取）
    std::string _depth_sub_name;  // 深度图像话题名（从参数服务器读取）
    ros::Publisher _pub_cloud;  // 点云消息发布器
    image_transport::SubscriberFilter _rgb_sub;  // RGB图像订阅器（支持同步过滤）
    image_transport::SubscriberFilter _depth_sub;  // 深度图像订阅器（支持同步过滤）
    boost::shared_ptr<Sync> sync_;  // 同步器智能指针（管理时间同步）
};
```

## launch文件编写相对路径

```xml
<!-- launch/my_launch.launch -->
<launch>
    <node 
        name="my_node" 
        pkg="my_pkg" 
        type="my_node" 
        output="screen"
    >
        <!-- 传递包内data/config.yaml的绝对路径 -->
        <param name="config_file" value="$(find my_pkg)/data/config.yaml" />
    </node>
</launch>
```

在 ROS1 中使用 C++ 手动关闭和开启订阅话题的核心在于管理 `ros::Subscriber`对象的生命周期。以下是详细步骤和示例代码：

### ​**​一、关闭订阅​**​

通过调用 `ros::Subscriber`对象的 `shutdown()`方法可以停止当前订阅，不再接收新消息。

### ​**​二、重新开启订阅​**​

关闭后，原 `Subscriber`对象会失效，需​**​重新创建​**​一个新的 `Subscriber`对象（使用相同的节点句柄、话题、回调函数等参数）。

### ​**​三、完整示例（含动态控制）​**​

以下示例展示如何在类中动态管理订阅的开启和关闭，包含线程安全的考虑

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>  // 用于智能指针

class TopicSubscriber {
private:
    ros::NodeHandle nh_;
    std::unique_ptr<ros::Subscriber> sub_;  // 智能指针管理订阅者
    std::mutex mtx_;  // 互斥锁保证线程安全（回调可能在其他线程执行）

public:
    TopicSubscriber() : nh_("~") {}  // 私有节点句柄（可根据需求改为全局）

    // 启动订阅
    void startSubscribe() {
        std::lock_guard<std::mutex> lock(mtx_);  // 加锁防止并发操作
        if (!sub_) {  // 仅当未订阅时创建
            // 创建订阅者（参数：节点句柄、话题名、队列大小、回调函数、绑定对象）
            sub_ = std::make_unique<ros::Subscriber>(
                nh_.subscribe("target_topic", 10, &TopicSubscriber::callback, this)
            );
            ROS_INFO("订阅已开启，话题：/target_topic");
        }
    }

    // 关闭订阅
    void stopSubscribe() {
        std::lock_guard<std::mutex> lock(mtx_);  // 加锁防止并发操作
        if (sub_) {
            sub_->shutdown();  // 停止订阅
            sub_.reset();      // 释放资源
            ROS_INFO("订阅已关闭");
        }
    }

    // 回调函数（示例：打印接收的消息）
    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("接收到消息: %s", msg->data.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_subscriber_node");
    TopicSubscriber subscriber;

    // 示例：运行 5 秒后关闭订阅，再过 3 秒重新开启
    ros::Rate rate(1);  // 1Hz 循环
    int count = 0;
    while (ros::ok()) {
        if (count == 5) {
            subscriber.stopSubscribe();
        } else if (count == 8) {
            subscriber.startSubscribe();
        }

        ros::spinOnce();  // 处理回调
        rate.sleep();
        count++;
    }

    return 0;
}
```

## sensor_msgs/imu

```bash
eader: 
  seq: 22055
  stamp: 
    secs: 1757948819
    nsecs: 557148218
  frame_id: "livox_frame"
orientation: 
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: -0.006201430223882198
  y: -0.00400785356760025
  z: -0.0030598677694797516
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: 0.007072287146002054
  y: -0.020632069557905197
  z: 0.9883446097373962
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### **1. header（消息头）​**​

ROS 消息的标准元数据，用于描述消息的基本信息：

- ​**​seq​**​: 消息序列号（整数）。每次发布新消息时递增，用于跟踪消息顺序或检测丢包（例如：若 `seq`不连续，说明中间有消息丢失）。

​**​stamp​**​: 消息的时间戳（`ros::Time`类型），包含两个子字段：

- ​**​secs​**​: 时间戳的秒数（整数）。

- - ​**​nsecs​**​: 时间戳的纳秒数（整数，范围 0-999,999,999）。
    
    组合起来表示消息的​**​采集时间​**​（注意：可能与实际接收时间不同，取决于传感器同步方式）。本例中 `secs: 1757948819`对应 UTC 时间约为 2025-09-16 17:46:59（需根据具体时区转换）。

- ​**​frame_id​**​: 坐标系标识符（字符串）。表示该 IMU 数据所属的坐标系（例如：`livox_frame`可能是 Livox 激光雷达或自定义传感器的坐标系名称）。

### ​**​2. orientation（方向）​**​

表示传感器在三维空间中的​**​姿态（朝向）​**​，用​**​四元数（Quaternion）​**​ 表示（避免欧拉角的“万向节锁”问题）。四元数格式为 `(x, y, z, w)`，满足 `x² + y² + z² + w² = 1`（归一化后）。

​**​x/y/z/w​**​: 四元数的四个分量。本例中全为 `0.0`，可能表示：

- 传感器未初始化（未获取到有效姿态数据）；

- - 姿态被默认设置为初始位姿（如无旋转时，四元数为 `(0,0,0,1)`，但此处可能驱动未正确填充）。

### ​**​3. orientation_covariance（方向协方差矩阵）​**​

描述​**​姿态估计的不确定性​**​（协方差矩阵），是一个 3×3 的对称矩阵，按​**​行优先顺序​**​展开为 9 个元素（`[xx, xy, xz, yx, yy, yz, zx, zy, zz]`）。

- 对角线元素（如 `xx`）：对应姿态分量（如四元数 x 分量）的方差（不确定性的平方）；

- 非对角线元素（如 `xy`）：不同姿态分量之间的协方差（相关性）。

本例中全为 `0.0`，通常表示：

- 方向的不确定性未知（传感器未提供误差估计）；

- 或驱动程序未正确填充该字段（实际应用中需关注是否为有效值）。

### ​**​4. angular_velocity（角速度）​**​

表示传感器绕各坐标轴的​**​旋转角速度​**​，单位通常为 ​**​弧度/秒（rad/s）​**​。

- ​**​x/y/z​**​: 分别对应绕传感器坐标系 x 轴、y 轴、z 轴的角速度。
  
  例如：`x: -0.0062...`表示绕 x 轴的逆时针旋转角速度约为 0.0062 rad/s（负号表示顺时针方向，具体取决于坐标系定义）。
  
  本例中数值很小，可能是传感器静止或缓慢旋转时的微小噪声。

### ​**​5. angular_velocity_covariance（角速度协方差矩阵）​**​

描述​**​角速度估计的不确定性​**​，同样是 3×3 的对称矩阵（9 元素，行优先）。

- 对角线元素（如 `xx`）：角速度 x 分量的方差；

- 非对角线元素：不同角速度分量间的协方差。

本例中全为 `0.0`，含义同 `orientation_covariance`（不确定度未知或未填充）。

### ​**​6. linear_acceleration（线加速度）​**​

表示传感器在三维空间中的​**​线性加速度​**​，单位通常为 ​**​米/二次方秒（m/s²）​**​。

- ​**​x/y/z​**​: 分别对应沿传感器坐标系 x 轴、y 轴、z 轴的加速度。
  
  本例中 `z: 0.9883...`接近重力加速度（约 9.8 m/s²），可能是传感器静止时，z 轴与重力方向一致（如传感器竖直放置，z 轴向下），此时测量的加速度主要为重力分量；`x/y`分量很小（约 ±0.02 m/s²），可能是测量噪声或轻微振动。

### ​**​7. linear_acceleration_covariance（线加速度协方差矩阵）​**​

描述​**​线加速度估计的不确定性​**​，3×3 对称矩阵（9 元素，行优先）。

- 对角线元素（如 `xx`）：线加速度 x 分量的方差；

- 非对角线元素：不同线加速度分量间的协方差。

本例中全为 `0.0`，含义同前（不确定度未知或未填充）。

## 在 C++ 类中使用成员函数作为回调函数时，需要绑定类的实例（`this`指针），因为类的非静态成员函数包含一个隐含的`this`参数。对于 ROS 中的消息订阅场景，通常使用`boost::bind`或 C++11 的`std::bind`来完成绑定

```cpp
class map_sever {
private:
    // 假设使用image_transport
    image_transport::ImageTransport _it;
    image_transport::Subscriber _sub;

public:
    // 构造函数（需要初始化ImageTransport）
    map_sever(ros::NodeHandle& nh) : _it(nh) {
        // 订阅消息时绑定回调函数与当前实例
        _sub = _it.subscribe(
            "/camera/color/image_raw", 
            2, 
            &map_sever::imageCallback,  // 成员函数指针
            this                        // 绑定当前实例
        );
    }

    // 回调函数定义（参数类型需匹配订阅的消息类型）
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // 处理图像消息
    }
};
```

**使用`bind`的场景**：如果需要传递额外参数，可以使用`boost::bind`或`std::bind`：

```cpp
#include <boost/bind.hpp>

// 在构造函数中
_sub = _it.subscribe(
    "/camera/color/image_raw", 
    2, 
    boost::bind(&map_sever::imageCallback, this, _1, extra_param), 
    this
);

// 对应的回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg, int extra_param) {
    // 处理逻辑
}
```

- `&map_sever::imageCallback`：要绑定的成员函数；
- `this`：绑定当前类实例（确保成员函数能访问类的成员）；
- `_1`：占位符，表示 “这里的参数会在实际调用时由外部传入”（这里特指 ROS 传递的 `msg`）；
- `extra_param`：绑定阶段就确定的固定参数（你的自定义参数）。

为什么需要 `_1`？**ROS 的订阅机制在触发回调时，只会自动传递一个参数（即消息本身，这里是 `msg`）。而你的回调函数需要两个参数（`msg` + `extra_param`），因此需要用 `_1` 占用 `msg` 的位置，告诉 `boost::bind`：“这个位置的参数我不现在指定，等 ROS 调用时会传入，你到时把它放到第一个参数位置即可”。

## 2. 构造新的 `NodeHandle` 时传入另一个 `NodeHandle` 的作用

```cpp
ros::NodeHandle nh_parent("my_namespace");
ros::NodeHandle nh_child(nh_parent, "sub_namespace");
```

这样做的效果是：

- `nh_child` 的命名空间 = `nh_parent` 的命名空间 + 你传入的相对路径
- 所以上面的例子中：
  - `nh_parent` 的命名空间是 `/my_namespace`
  - `nh_child` 的命名空间是 `/my_namespace/sub_namespace`

**好处**：可以方便地在不同模块之间传递一个 “基础命名空间”，子模块只关心自己的相对命名空间，不用管全局路径。
