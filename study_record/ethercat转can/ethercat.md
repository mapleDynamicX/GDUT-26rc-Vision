# ethercat

EtherCAT（Ethernet for Control Automation Technology，以太网控制自动化技术）是一种专为工业自动化设计的​**​实时以太网通信协议​**​，由德国倍福自动化（Beckhoff Automation）于2003年推出，现为​**​EtherCAT技术协会（ETG）​**​维护的开放标准（IEC 61158/61784）。它以高实时性、低延迟和高带宽利用率著称，广泛应用于工业机器人、数控系统、包装机械、半导体制造等对时间敏感的场景。

工业自动化对通信的核心需求是​**​确定性实时性​**​（即数据传输时间可预测、延迟极低）和​**​高精度同步​**​（多设备协同工作时的时钟一致性）。传统以太网（如TCP/IP）因协议栈开销大、非确定性延迟（如MAC层冲突、TCP重传）无法满足工业控制需求，而EtherCAT通过创新机制解决了这一问题

### ​**​二、关键技术特性​**​

#### 1. ​**​“飞读飞写”（On-the-Fly）通信机制​**​

EtherCAT的核心创新是​**​数据帧的单次遍历处理​**​，彻底颠覆了传统以太网“存储-转发”的逐节点处理模式：

- •
  
  主站向网络发送一个​**​循环数据帧​**​（包含所有从站的输入/输出数据）。

- •
  
  数据帧沿物理链路依次经过每个从站时，从站​**​无需缓存整个帧​**​，而是直接“读取”属于自己的输入数据，并将输出数据“写入”帧中预留的位置，随后立即转发给下一个节点。

- •
  
  所有从站处理完成后，数据帧最终回到主站，完成一次全网络的数据交换。

这一机制使数据传输延迟仅取决于​**​物理链路长度​**​和​**​从站处理时间​**​（通常为几微秒），而非节点数量。例如，100个从站的链路延迟与10个从站几乎相同，带宽利用率接近100%（传统以太网因报文分片和冲突，带宽利用率常低于50%）。

#### 2. ​**​分布式时钟（Distributed Clocks, DC）​**​

工业设备协同工作（如机械臂多轴联动）需要​**​亚微秒级时钟同步​**​。EtherCAT通过以下机制实现：

- •
  
  主站内置高精度时钟（如晶体振荡器），作为系统时间基准。

- •
  
  每个从站通过硬件同步模块（ESC，EtherCAT Slave Controller）同步到主站时钟，误差可控制在​**​±100纳秒​**​内。

- •
  
  支持“主从时钟偏移补偿”和“网络延迟校准”，即使链路存在延迟波动，仍能保持全局同步。

## 专业名词和术语[](#id4 "Permalink to this heading")

### SDO/PDO[](#sdo-pdo "Permalink to this heading")

PDO（Process Data Object）和SDO（Service Data Object）是EtherCAT通信中用于数据交换的两种不同类型的数据对象。 PDO是一种实时数据通信方式，主要用于在EtherCAT网络中进行==实时数据交换==。PDO直接读取和写入从站的I/O数据，支持高速实时通信，但是数据格式和长度是固定的，不太适合于复杂的数据交换。 SDO是一种用于配置和参数传输的通信方式，支持灵活的数据格式和长度，适用于==对从站进行配置和参数传输==等操作。SDO通过从站对象字典（OD）来进行配置，能够实现动态修改从站参数，提高了系统的灵活性。】 综合来说，PDO适用于实时数据传输，SDO适用于配置和参数传输。在EtherCAT通信中，一般都会同时使用这两种数据对象来完成数据交换。

**二者区别**

SDO（Service Data Object）是一种基于==点对点通信==的方式，数据直接在从设备和主设备之间传输。主设备可以发送读或写请求来访问从设备中的各种对象字典（Object Dictionary）的参数。每个请求都包含一个索引和一个子索引，以标识对象字典中的唯一参数。在工作时，主站需要寻址具体的某个从设备。SDO还可以提供元数据，如读取或写入的数据类型和数据长度。

相反，PDO（Process Data Object）是一种通过主设备和所有从设备之间预定义的I/O映射表实现的数据交换方式。I/O映射表包含了进程数据（Process Data）的来源和目标信息，这些进程数据以及配置如何在通信周期内传输数据的各种参数，如数据类型、尺寸和传输模式等。与SDO不同，PDO是以==广播==的形式，即同时向所有从设备发送数据，一个PDO帧中包含所有目标从设备的数据，然后每个从设备通过ID指令来提取自己所需的数据。

总的来说，SDO是一种点对点通信，能够读写对象字典中的各个字段。PDO则是广播式通信，用于主从设备之间的实时数据交换。两者在应用中的使用会有所不同，具体取决于需要通信的数据类型、频率和实时性要求等。

## 代码学习

```cpp
//作用​​：包含EtherCAT主站库的核心头文件。ethercat.h是EtherCAT通信协议的基础头文件，
//提供了与EtherCAT主站交互的函数（如状态机控制、数据读写等）和数据结构（如EC_STATE枚举）#include <ethercat.h>
#include <ethercat.h>
//作用​​：包含自定义的从站和总线管理类的头文件
#include <soem_interface_examples/ExampleSlave.hpp>
#include <soem_interface_rsl/EthercatBusBase.hpp>
int main(int argc, char** argv) {
//指定EtherCAT总线的物理网络接口名称。在Linux系统中，
//eth1通常对应第二块以太网网卡（eth0为第一块），主站通过该接口与EtherCAT从站通信
  const std::string busName = "eth1";
//slaveName：从站设备的逻辑名称（用于软件标识），可能与实际从站的硬件名称或配置一致
  const std::string slaveName = "ExampleSlave";
//slaveAddress：从站的站地址（Station Address），是EtherCAT总线上从站的唯一标识（范围通常为0-1023），
//用于主站区分不同的从站。此处设为0，可能是默认或第一个从站
  const uint32_t slaveAddress = 0;
//作用​​：创建一个EtherCAT总线管理对象的智能指针。EthercatBusBase是总线操作的核心类，
//负责底层网络接口初始化、从站发现、状态机等。std::unique_ptr是智能指针，自动管理内存，避免手动释放
  std::unique_ptr<soem_interface_rsl::EthercatBusBase> bus = std::make_unique<soem_interface_rsl::EthercatBusBase>(busName);
//作用​​：创建具体从站对象的智能指针。ExampleSlave是从站的具体实现类，构造函数参数包括
//slaveName：从站名称（用于标识）
//bus.get()：总线对象的原始指针（从站需要通过总线与主站通信）
//slaveAddress：从站的站地址（与总线上的物理地址对应）。std::shared_ptr允许从站对象被多个模块共享（如总线和其他从站）
 std::shared_ptr<soem_interface_examples::ExampleSlave> slave =
      std::make_shared<soem_interface_examples::ExampleSlave>(slaveName, bus.get(), slaveAddress);
//​作用​​：将创建的从站对象添加到总线管理器中。总线需要知道所有连接的从站，以便后续启动、状态管理和数据通信
  bus->addSlave(slave);
  //bus->startup();
//作用​​：启动EtherCAT总线。startup方法的参数
//true：启用尺寸检查（可能是在通信过程中验证输入/输出数据的长度是否符合从站描述，避免数据溢出或错误）。
//10：启动过程中遇到错误时的最大重试次数（例如，从站未响应时，最多重试10次）
  bus->startup(true, 10);// true: 启用尺寸检查；10: 最大重试次数（可按需调整）
//作用​​：将EtherCAT总线的状态切换为OPERATIONAL（运行态）。EtherCAT总线有多个状态
//如INIT初始化、PRE-OP预操作、SAFE-OP安全操作、OPERATIONAL运行），只有进入OPERATIONAL状态后，
//主站和从站才会开始周期性的实时数据交换（PDO传输）。
  bus->setState(EC_STATE_OPERATIONAL);
//作用​​：阻塞等待指定从站（slaveAddress=0）进入OPERATIONAL状态。
//如果超时仍未达到目标状态（例如从站故障、通信中断），返回错误码1并退出程序
  if (!bus->waitForState(EC_STATE_OPERATIONAL, slaveAddress)) {
    // Something is wrong
    return 1;
  }
//作用​​：主循环，持续处理EtherCAT数据的实时读写。
//updateRead()：从所有从站读取输入数据（Process Data Input, PDO）。
//PDO是EtherCAT中实时传输的关键数据（如传感器值、控制指令），
//主站通过此函数从从站的输入邮箱（Input Mailbox）读取数据。
//updateWrite()：向所有从站写入输出数据（Process Data Output, PDO）。
//主站通过此函数将控制指令等数据写入从站的输出邮箱（Output Mailbox），
//从站会在下一个通信周期读取这些数据。
  while (true) {
    bus->updateRead();
    bus->updateWrite();
  }
//当主循环退出时（如用户中断或程序终止），调用shutdown()关闭总线
  bus->shutdown();
  return 0;
}
```

### **​一、总线名称的核心作用​**​

EtherCAT 总线名称的核心目的是​**​在网络中唯一或逻辑化地标识设备​**​，方便主站管理从站、开发者调试或系统集成时快速定位设备。它不同于物理层的 MAC 地址（全球唯一），但可能与 MAC 地址、设备序列号或厂商自定义信息关联。

### **​二、主站的总线名称​**​

EtherCAT 主站（Master）的总线名称通常指主站实例的标识，用于区分同一网络中可能存在的多个主站（尽管工业场景中通常单主站）。其表现形式和命名规则因主站软件而异：

```cpp
// 包含ROS核心功能头文件，用于节点初始化、消息通信等
#include <ros/ros.h>
// 包含SOEM库（EtherCAT主站接口）相关头文件，用于EtherCAT总线通信
#include <soem_rsl/ethercat.h>
// 包含C++标准线程库，用于多线程操作
#include <thread>
// 包含自定义线程发布器头文件，提供异步消息发布功能
#include "any_node/ThreadedPublisher.hpp"
// 包含自定义工作线程头文件，用于创建周期性任务
#include "any_worker/Worker.hpp"
// 包含自定义EtherCAT从站设备类头文件，具体实现RcEcatSlave功能
#include "rc_ecat_master/RcEcatSlave.hpp"
// 包含ROS时间头文件，用于时间戳处理
#include "ros/time.h"
// 包含ROS传感器消息头文件，定义关节状态消息类型
#include "sensor_msgs/JointState.h"

// 主函数，程序入口
int main(int argc, char** argv) {
  // 初始化ROS节点，节点名称为"rc_ecat_node"，自动处理命令行参数
  ros::init(argc, argv, "rc_ecat_node");
  // 创建ROS节点句柄，用于与ROS系统交互
  ros::NodeHandle nh;

  // 定义EtherCAT总线名称（对应Linux系统中的网络接口名）
  const std::string busName = "enx10e04c360200";
  // 定义从站设备名称（自定义标识，用于区分不同类型从站）
  const std::string slaveName = "RcEcatSlave";
  // 定义从站在总线上的地址（EtherCAT从站通过站号唯一标识，范围0-15）
  const uint32_t slaveAddress = 1;

  /* 实例化EtherCAT主站基类 */
  // 使用std::make_unique创建EthercatBusBase智能指针，传入总线名称初始化
  // EthercatBusBase是EtherCAT主站的基础类，负责总线初始化、状态管理等核心操作
  std::unique_ptr<soem_interface_rsl::EthercatBusBase> bus =
      std::make_unique<soem_interface_rsl::EthercatBusBase>(busName);

  /* 实例化从站基类（使用get得到智能指针的原地址） */
  // 使用std::make_shared创建RcEcatSlave共享指针，传入从站名称、总线指针和站号
  // RcEcatSlave是具体从站设备的类，继承自基类，实现该从站的通信协议解析
  std::shared_ptr<rc_ecat_master::RcEcatSlave> slave =
      std::make_shared<rc_ecat_master::RcEcatSlave>(slaveName, bus.get(),
                                                    slaveAddress);

  /* 往主站添加从站设备 */
  // 调用总线的addSlave方法，将实例化的从站设备注册到总线中
  // 主站通过此方法管理所有连接的从站设备
  bus->addSlave(slave);

  /* 启动EtherCAT总线初始化流程 */
  // 调用startup方法启动总线，参数true表示执行完整的初始化（包括硬件初始化）
  bus->startup(true);

  /* 设置总线目标状态为操作状态（OPERATIONAL） */
  // EtherCAT从站需要经历INIT→PRE-OP→SAFE-OP→OP状态，此方法触发状态切换
  bus->setState(EC_STATE_OPERATIONAL);

  /* 等待从站进入操作状态，超时或失败则退出程序 */
  // waitForState方法阻塞等待指定从站（slaveAddress）达到目标状态（EC_STATE_OPERATIONAL）
  // 若超时未达到操作状态，返回false，程序返回1退出
  if (!bus->waitForState(EC_STATE_OPERATIONAL, slaveAddress)) {
    return 1;
  }

  /* 创建线程安全的ROS消息发布器（事件驱动，发布频率由调用频率决定） */
  // 使用std::make_shared创建ThreadedPublisher智能指针
  // 模板参数为消息类型（sensor_msgs::JointState），构造参数包括：
  // - nh.advertise：ROS的广告发布器（指定话题名"m3508_state"和队列大小10）
  // - 50：发布器内部缓存的消息数量上限
  // - false：是否启用异步发布（此处关闭）
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState> joint_state_publisher;
  joint_state_publisher =
      std::make_shared<any_node::ThreadedPublisher<sensor_msgs::JointState>>(
          nh.advertise<sensor_msgs::JointState>("m3508_state", 10), 50, false);

  /* 初始化关节状态消息对象 */
  sensor_msgs::JointState joint_state_msg;
  // 设置消息中关节的名称（此处只有一个名为"m3508_1"的关节）
  joint_state_msg.name = {"m3508_1"};

  /* 创建周期性工作线程，执行EtherCAT数据读写和消息发布 */
  // 实例化Worker对象"master_controller"，构造参数：
  // - "master_controller"：工作线程名称
  // - 0.0005：线程周期（约0.5ms，实际可能受系统调度影响）
  // - 回调函数：定义每个工作周期执行的具体操作（lambda表达式）
  any_worker::Worker master_controller(
      "master_controller", 0.0005,
      [&](const any_worker::WorkerEvent& event) -> bool {  // 回调函数
        // 获取总线最后一次更新读取操作的时间戳（高精度时间）
        auto tp = bus->getUpdateReadStamp();
        // 将时间戳转换为秒数（std::chrono::duration转换）
        double secs =
            std::chrono::duration<double>(tp.time_since_epoch()).count();
        // 提取秒和纳秒部分，用于填充ROS消息的时间戳
        uint32_t sec = static_cast<uint32_t>(secs);
        uint32_t nsec = static_cast<uint32_t>((secs - sec) * 1e9);
        // 设置关节状态消息的时间戳（ROS消息需要时间戳标识数据时效性）
        joint_state_msg.header.stamp.sec = sec;
        joint_state_msg.header.stamp.nsec = nsec;

        /* 从总线读取从站最新数据（从从站EEPROM/寄存器读取到主站内存） */
        bus->updateRead();

        // （以下代码被注释，实际未执行）
        // 读取从站电机0的位置数据（指定CAN总线接口CAN0和电机索引0）
        // joint_state_msg.position = {slave->getMotorPosition(rc_ecat_master::RcEcatSlave::CanBus::CAN0, 0)};
        // 读取从站电机0的速度数据
        // joint_state_msg.velocity = {slave->getMotorVelocity(rc_ecat_master::RcEcatSlave::CanBus::CAN0, 0)};
        // 读取从站电机0的力矩（电流）数据
        // joint_state_msg.effort = {slave->getMotorEffort(rc_ecat_master::RcEcatSlave::CanBus::CAN0, 0)};
        // 使用线程发布器发布关节状态消息（异步发布，保证实时性）
        // joint_state_publisher->publish(joint_state_msg);
        // 发送ROS消息（可能触发实际网络传输）

        /* 向总线写入主站数据（从主站内存写入到从站寄存器） */
        // 例如发送控制指令（如目标位置、速度等），当前代码未实现具体写入逻辑
        bus->updateWrite();

        // 回调函数返回true表示正常运行，若返回false工作线程会终止
        return true;
      });

  /* 启动工作线程，优先级设为47（Linux实时优先级范围0-99，值越大优先级越高） */
  // start方法参数为线程优先级，启动后工作线程会按设定周期执行回调函数
  master_controller.start(47);

  /* 主循环：持续打印从站状态，保持程序运行 */
  while (1) {
    // 输出从站对象的字符串表示（需RcEcatSlave类重载<<运算符）
    // 用于实时监控从站的关键状态（如位置、速度、错误码等）
    std::cout << *slave;
    // 线程休眠10毫秒，降低循环频率，减少CPU占用
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 程序理论上不会执行到这里（无限循环），返回0表示正常退出
  return 0;
}
```
