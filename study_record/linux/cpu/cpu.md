# cpu

在 Ubuntu 系统下的 C++ ROS 开发中，`ros::Rate` 的 `sleep()` 方法**会主动释放 CPU 资源**，而非 “忙等”（busy waiting）占用 CPU，这也是 ROS 设计该接口的核心目的之一。

### 配置优先级

```cpp
// 补充必需的头文件（解决PRIO_PROCESS未定义问题）
#include <sys/resource.h>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

/**
 * @brief 将当前调用该函数的线程设置为高优先级（分配更多CPU资源）
 * @param realtime_priority 实时调度优先级（1-99，建议10-50，默认50）
 * @param nice_value 普通调度的nice值（-20~19，越小优先级越高，默认-10）
 * @return int 0=成功（实时调度）, 1=降级为nice值调整成功, -1=所有尝试失败
 */
int set_thread_as_important(int realtime_priority = 50, int nice_value = -10) {
    // 1. 校验参数合法性
    if (realtime_priority < 1 || realtime_priority > 99) {
        std::cerr << "[错误] 实时优先级必须在1-99之间，已自动修正为50" << std::endl;
        realtime_priority = 50;
    }
    if (nice_value < -20 || nice_value > 19) {
        std::cerr << "[错误] nice值必须在-20~19之间，已自动修正为-10" << std::endl;
        nice_value = -10;
    }

    // 2. 获取当前线程ID
    pthread_t tid = pthread_self();

    // 3. 优先尝试设置实时调度策略（SCHED_FIFO，最高优先级）
    struct sched_param param;
    std::memset(&param, 0, sizeof(param)); // 改用std::memset，避免命名空间问题
    param.sched_priority = realtime_priority;

    int ret = pthread_setschedparam(tid, SCHED_FIFO, &param);
    if (ret == 0) {
        // 验证设置结果并输出
        int policy;
        pthread_getschedparam(tid, &policy, &param);
        std::cout << "[成功] 当前线程已设为实时调度(SCHED_FIFO)，优先级：" 
                  << param.sched_priority << std::endl;
        return 0; // 实时调度设置成功
    }

    // 4. 实时调度失败，降级调整nice值（普通调度的优先级）
    std::cerr << "[提示] 实时调度设置失败(" << std::strerror(ret) 
              << ")，尝试调整nice值..." << std::endl;
    
    if (setpriority(PRIO_PROCESS, 0, nice_value) == 0) {
        std::cout << "[成功] 当前线程nice值已设为：" << nice_value 
                  << "（普通调度高优先级）" << std::endl;
        return 1; // nice值调整成功
    }

    // 5. 所有尝试失败
    std::cerr << "[失败] 调整nice值也失败：" << std::strerror(errno) << std::endl;
    std::cerr << "  可能原因：1. 未使用sudo运行 2. 系统权限限制" << std::endl;
    return -1; // 全部失败
}

```

### 函数核心说明

表格

| 函数参数                | 作用                                           |
| ------------------- | -------------------------------------------- |
| `realtime_priority` | 实时调度优先级（1-99），默认 50（建议 10-50，避免抢占系统核心线程）     |
| `nice_value`        | 普通调度的 nice 值（-20~19），默认 - 10（数值越小，普通调度优先级越高） |

### 运行

##### 方式 2：sudo roslaunch（多节点）

创建一个启动脚本（如 `launch_with_sudo.sh`），内容如下：

```bash
#!/bin/bash
# 加载 ROS 环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# 运行 launch 文件
roslaunch your_package your_launch.launch

```

添加可执行权限：

```bash
chmod +x launch_with_sudo.sh

```

然后 sudo 运行脚本：

```bash
sudo -E ./launch_with_sudo.sh

```

而 `-E` 参数会告诉 `sudo`：**不要重置环境变量，把当前普通用户的所有环境变量完整传递给 `root` 身份的执行环境**。

##### 二、为什么 ROS 脚本常用 `sudo -E`？

ROS 开发中大量依赖自定义环境变量（比如你在 `~/.bashrc` 中配置的）：

- `ROS_MASTER_URI`：指定 ROS Master 的地址；
- `ROS_PACKAGE_PATH`：ROS 功能包的搜索路径；
- `LD_LIBRARY_PATH`：ROS 库文件的加载路径；
- `PATH`：ROS 可执行文件（如 `rosrun`、`roslaunch`）的搜索路径。

如果不用 `-E`，直接执行 `sudo ./launch_with_sudo.sh`，`root` 环境中这些 ROS 相关的环境变量会丢失，导致脚本报错：

### 计时

```cpp
#include <iostream>
#include <chrono>   // 核心头文件
#include <thread>   // 用于模拟耗时操作（可选）

// 1. 简单计时：计算代码块执行时间（毫秒级）
void test_simple_timer() {
    // 记录开始时间（steady_clock 避免系统时间干扰）
    auto start = std::chrono::steady_clock::now();

    // 模拟耗时操作（比如你的业务逻辑）
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 休眠500ms

    // 记录结束时间
    auto end = std::chrono::steady_clock::now();

    // 计算耗时（转换为毫秒）
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "简单计时 - 耗时：" << duration_ms << " 毫秒" << std::endl;
}

// 2. 高精度计时：微秒/纳秒级（适合短代码块）
void test_high_precision_timer() {
    auto start = std::chrono::high_resolution_clock::now();

    // 模拟极短耗时操作（循环计算）
    long long sum = 0;
    for (long long i = 0; i < 1000000; ++i) {
        sum += i;
    }

    auto end = std::chrono::high_resolution_clock::now();

    // 转换为微秒/纳秒
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "高精度计时 - 耗时：" << duration_us << " 微秒 / " << duration_ns << " 纳秒" << std::endl;
}

// 3. 封装成通用计时函数（复用性强）
template <typename Func>
long long measure_time_ms(Func&& func) {
    auto start = std::chrono::steady_clock::now();
    func(); // 执行传入的任意函数
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

// 测试封装的计时函数
void test_wrapper_timer() {
    // 定义要计时的业务函数
    auto my_business_func = []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "业务函数执行中..." << std::endl;
    };

    // 调用封装的计时函数
    long long cost = measure_time_ms(my_business_func);
    std::cout << "封装计时 - 业务函数耗时：" << cost << " 毫秒" << std::endl;
}

int main() {
    std::cout << "=== 简单计时测试 ===" << std::endl;
    test_simple_timer();

    std::cout << "\n=== 高精度计时测试 ===" << std::endl;
    test_high_precision_timer();

    std::cout << "\n=== 封装计时函数测试 ===" << std::endl;
    test_wrapper_timer();

    return 0;
}

```

```cpp
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <ros/ros.h>

/**
 * @brief 绑定当前线程到指定CPU核心（解决CPU密集型线程核心漂移/缓存失效问题）
 * @param cpu_core 目标CPU核心编号（从0开始，如0/1/2...，需小于系统总核心数）
 * @return bool true=绑定成功，false=绑定失败（核心非法/权限不足）
 */
bool bind_thread_to_cpu(int cpu_core) {
    // 检查核心编号合法性
    int cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_core < 0 || cpu_core >= cpu_count) {
        std::cerr << "[错误] CPU核心" << cpu_core << "非法，当前CPU核心数：" << cpu_count << std::endl;
        return false;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    pthread_t tid = pthread_self();
    int ret = pthread_setaffinity_np(tid, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "[错误] 绑定CPU核心失败：" << std::strerror(ret) << std::endl;
        return false;
    }

    std::cout << "[成功] CPU密集线程已绑定到核心：" << cpu_core << std::endl;
    return true;
}

/**
 * @brief 将当前CPU密集型线程设置为低优先级实时调度（避免抢占系统线程，提升计算效率）
 * @param realtime_priority 实时调度优先级（仅支持10-20，默认15，CPU密集型安全范围）
 * @param nice_value 普通调度的nice值（仅降级时生效，默认-5，CPU密集型无需依赖）
 * @return int 0=成功（SCHED_RR实时调度）, 1=降级为nice值调整成功, -1=所有尝试失败
 * @note 仅适用于CPU密集型线程（纯计算/算法推理），IO密集型线程请勿调用
 * @note 需sudo运行程序，且系统已配置/etc/security/limits.conf放开rtprio权限
 */
int set_thread_as_important(int realtime_priority = 15, int nice_value = -5) {
    // 1. 校验实时优先级（限定10-20，避免抢占系统核心线程）
    if (realtime_priority < 10 || realtime_priority > 20) {
        std::cerr << "[调整] 实时优先级超出CPU密集型安全范围（10-20），已自动修正为15" << std::endl;
        realtime_priority = 15;
    }
    // 2. 校验nice值（限定-10~0，避免普通调度优先级过高）
    if (nice_value < -10 || nice_value > 0) {
        std::cerr << "[调整] nice值超出安全范围（-10~0），已自动修正为-5" << std::endl;
        nice_value = -5;
    }

    // 3. 获取当前线程ID
    pthread_t tid = pthread_self();

    // 4. 设置SCHED_RR实时调度（时间片轮转，适合CPU密集型并行）
    struct sched_param param;
    std::memset(&param, 0, sizeof(param));
    param.sched_priority = realtime_priority;

    int ret = pthread_setschedparam(tid, SCHED_RR, &param);
    if (ret == 0) {
        // 验证调度策略设置结果
        int policy;
        pthread_getschedparam(tid, &policy, &param);
        const char* policy_name = (policy == SCHED_RR) ? "SCHED_RR" : "UNKNOWN";
        std::cout << "[成功] CPU密集线程已设为实时调度(" << policy_name << ")，优先级：" 
                  << param.sched_priority << std::endl;
        return 0; // 实时调度设置成功
    }

    // 5. 实时调度失败，降级调整nice值（CPU密集型效果有限，仅兜底）
    std::cerr << "[提示] 实时调度设置失败(" << std::strerror(ret) 
              << ")，尝试调整nice值..." << std::endl;
    
    if (setpriority(PRIO_PROCESS, 0, nice_value) == 0) {
        std::cout << "[成功] CPU密集线程nice值已设为：" << nice_value 
                  << "（普通调度高优先级）" << std::endl;
        return 1; // nice值调整成功
    }

    // 6. 所有尝试失败
    std::cerr << "[失败] 调整nice值也失败：" << std::strerror(errno) << std::endl;
    std::cerr << "  可能原因：1. 未使用sudo运行 2. 系统权限限制（需配置limits.conf）" << std::endl;
    return -1; // 全部失败
}

/**
 * @brief CPU密集型线程的核心业务函数（示例：纯计算逻辑）
 * @param arg 线程入参（ROS节点句柄/配置参数等，此处为nullptr）
 * @return void* 线程返回值（此处为nullptr）
 * @note 需先绑定CPU核心，再设置实时优先级，顺序不可颠倒
 */
void* cpu_intensive_thread_func(void* arg) {
    ROS_INFO("CPU密集型线程启动，开始初始化优化配置...");
    
    // 第一步：绑定到专属CPU核心（避开系统核心0，推荐核心2/3）
    if (!bind_thread_to_cpu(2)) {
        ROS_ERROR("CPU核心绑定失败，线程退出！");
        return nullptr;
    }

    // 第二步：设置低优先级实时调度（SCHED_RR + 15）
    int ret = set_thread_as_important(15, -5);
    if (ret < 0) {
        ROS_ERROR("CPU密集线程优先级设置失败，线程退出！");
        return nullptr;
    }

    // CPU密集型核心逻辑（纯计算示例）
    long long sum = 0;
    ros::Rate rate(1); // 低频日志输出，不影响计算
    while (ros::ok()) {
        // 模拟大规模计算（10亿次累加）
        for (long long i = 0; i < 1000000000; ++i) {
            sum += i;
        }
        ROS_INFO_THROTTLE(1, "CPU密集计算完成，sum=%lld，当前运行核心：%d", sum, sched_getcpu());
        sum = 0; // 重置累加值
        rate.sleep();
    }
    return nullptr;
}

```


