# study

### `1.` 函数用于接收命令行参数int main(int argc, char **argv)

**argc**

<div>
    类型：int
    含义：表示命令行参数的数量（包括程序名称本身）。
    值范围：至少为 1（当程序不带参数运行时，argc=1，仅包含程序名）。
    示例：
    命令行 ./myapp input.txt -v
    → argc = 3（程序名 + 2 个参数）。
</div>

**argv**

<div>
类型：char **（或等效的 char *argv[]）
含义：指向字符串数组的指针，每个字符串存储一个命令行参数。
数组结构：
    argv[0]：程序名称（路径），如 "./myapp"。
    argv[1] 到 argv[argc-1]：用户输入的参数。
    argv[argc]：固定为 NULL（结束标志）。
</div>

内存布局：

```
argv[0] → "./myapp"
argv[1] → "input.txt"
argv[2] → "-v"
argv[3] → NULL
```

## C++ 中的​**​线程​**​

### 1.概念

> - 在计算机科学中，​**​线程​**​（Thread）是操作系统能够进行​**​调度和分派的最小执行单元​**​。
> - 它是​**​进程​**​（Process）的一个实体，是 CPU 调度和分派的基本单位。
> - 一个进程可以包含多个线程。这些线程共享进程的内存空间（如全局变量、堆内存、打开的文件描述符等），但每个线程拥有自己独立的栈空间和程序计数器（PC），用于记录执行位置。
> - ​**​C++11 标准​**​正式引入了对多线程编程的原生支持，主要提供了 `std::thread` 类和相关同步原语（如互斥锁 `std::mutex`、条件变量 `std::condition_variable` 等）。

### 2.作用

> - **并发执行：​**​ 允许程序同时执行多个任务，充分利用多核 CPU 的计算能力，提高程序的执行效率。例如：
>   - UI 程序：主线程处理界面渲染和用户输入，后台线程进行耗时计算或网络请求，防止界面卡顿。
>   - 服务器：同时处理多个客户端请求。
>   - 计算密集型应用：将大型任务分解成多个可并行计算的小任务。
> - ​**​响应性：​**​ 在图形用户界面或实时系统中，将耗时的操作（如文件 I/O、复杂计算）放到后台线程执行，保持主线程（如 UI 事件循环）的响应能力。
> - ​**​结构化设计：​**​ 可以将逻辑上独立的任务设计为单独的线程，提高代码的模块化和可维护性。
> - ​**​建模：​**​ 更容易地模拟现实世界中并发发生的事件

### 3.示例

```cpp
#include <iostream>
#include <thread>
void helloFunction() {
    std::cout << "Hello from thread!" << std::endl;
}
int main() {
    // 创建一个新线程，执行 helloFunction
    std::thread worker(helloFunction); 
    // 主线程继续执行...
    std::cout << "Hello from main thread!" << std::endl;
    // 等待 worker 线程执行完毕
    worker.join(); 
    return 0;
}
```

## C++ 中的​线程​池

### 1.概念

> - **线程池​**​（Thread Pool）是一种​**​并发编程的设计模式​**​。
> - 其核心思想是​**​预先创建好一组线程​**​，并将它们放入一个“池”（Pool）中进行管理。
> - 程序将要执行的任务（通常封装为函数对象、Lambda 表达式等）提交到线程池的一个​**​任务队列​**​中。
> - 池中的​**​空闲线程​**​会主动从任务队列中获取任务并执行。
> - 执行完成后，线程并不会被销毁，而是​**​返回池中​**​，等待执行下一个任务。

### 2.作用

> - **减少线程创建销毁的开销：​**​ 线程的创建和销毁涉及与操作系统的频繁交互，代价相对较高（CPU 时间、内存）。线程池通过复用已创建好的线程，​**​显著降低了频繁创建销毁线程带来的性能损耗​**​。这对于需要处理大量短生命周期任务（如网络服务器处理请求）的场景尤为重要。
> - ​**​控制并发线程数量：​**​ 无限制地创建线程可能导致系统资源（CPU、内存、上下文切换开销）耗尽，甚至导致程序或系统崩溃。线程池限定了最大线程数量，有效地​**​控制系统资源消耗和并发程度​**​，防止过载。
> - ​**​管理任务队列：​**​ 当任务提交速度超过处理速度时，任务队列提供了缓冲，​**​优雅地处理任务积压​**​，任务不会因为线程不够而被立即拒绝（可以根据策略配置饱和时的行为，如丢弃、阻塞提交者、等待队列空间）。
> - ​**​提高响应性：​**​ 由于线程是预先创建好的，当有任务到达时，可以立即被空闲线程执行，​**​避免了临时创建线程的延迟​**​，加快任务的启动速度。
> - ​**​方便资源管理和监控：​**​ 集中管理一组线程，更容易进行统一的初始化（设置资源限制、优先级）、终止、监控线程状态等

<img title="" src="file:///home/maple/笔记/images/2025-07-11-17-47-23-thread.png" alt="" width="680">

## ​**​Eigen::aligned_allocator 的作用​**​

> - `Eigen::aligned_allocator` 是 Eigen 库提供的分配器，它专门处理内存对齐问题。例如：
>   - 它会根据 `PointType` 的对齐要求（通常由 `alignof(PointType)` 决定），分配对齐的内存块（例如，16 字节、32 字节等）。
>   - 这确保了 `std::vector` 中的每个元素（每个 `PointType` 对象）在内存中都被正确对齐，从而安全高效地用于 Eigen 的操作或 SIMD 指令。
> - 在 `PointVector` 中，它替换了 `std::vector` 的默认分配器（`std::allocator`），提供了一种更安全的替代方案。

### 示例

```cpp
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector> // 提供 Eigen::aligned_allocator
// 定义一个点类型，包含 Eigen 成员
struct PointType {
    Eigen::Vector3f position; // 需要 16 字节对齐
    float intensity;
};
// 使用 typedef 定义 PointVector
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
int main() {
    PointVector points; // 声明一个点向量，使用对齐分配器
    // 添加点（内存分配时会自动对齐）
    points.push_back({Eigen::Vector3f(1.0f, 2.0f, 3.0f), 0.5f});
    points.push_back({Eigen::Vector3f(4.0f, 5.0f, 6.0f), 0.8f});
    // 安全访问：内存对齐确保 Eigen 操作高效
    for (const auto& point : points) {
        Eigen::Vector3f pos = point.position; // 无未对齐错误风险
    }
    return 0;
}
```

## 基本线程创建

```cpp
#include <iostream>
#include <thread>
// 线程执行的函数
void thread_function() {
    std::cout << "子线程ID: " << std::this_thread::get_id() << std::endl;
}
int main() {
    // 创建线程对象并立即执行
    std::thread t(thread_function);    
    std::cout << "主线程ID: " << std::this_thread::get_id() << std::endl;   
    // 等待线程结束（重要！）
    t.join();
    return 0;
}
```

## 带参数的线程函数

```cpp
void print_sum(int a, int b) {
    std::cout << a << " + " << b << " = " << (a + b) << std::endl;
}
int main() {
    std::thread t(print_sum, 3, 5); // 传递参数
    t.join();
}
```

## 线程管理关键点

```cpp
std::thread t(func);
t.detach(); // 主线程不再管理此线程
```

> - **`join()`​**​：阻塞主线程直至子线程结束
> 
> - ​**​`detach()`​**​：分离线程（失去控制权，后台运行）

**在 C++ 中创建空线程并在之后操作指定对象，需要使用线程同步机制（如互斥锁和条件变量）**

> | `thread()`     | 创建空线程对象    | 无关联线程              |
> | -------------- | ---------- | ------------------ |
> | `thread(func)` | 创建并启动线程    | 参数按值传递             |
> | `join()`       | 等待线程结束     | 阻塞调用线程，调用后不可再操作    |
> | `detach()`     | 分离线程（后台运行） | 无法再控制线程，需防范悬空引用    |
> | `joinable()`   | 检查线程是否可操作  | join/detach 前的必要检查 |

**​结束方式​**​：

- 线程对象销毁前必须调用 ​**​`join()`或 `detach()`​**​

- 未调用时触发 `std::terminate()`终止程序

## 互斥锁

### 示例：

在 C++ 中，当多个线程需要访问共享的 int 类型变量时，必须使用互斥锁来防止数据竞争和不一致。下面是一个完整的实现方案：

```cpp
#include <iostream>
#include <mutex>
int main() {
    // 1. 创建需要保护的整型变量和对应的互斥锁
    int a = 0;                 // 需要保护的共享变量
    std::mutex a_mutex;        // 保护变量a的互斥锁
    std::cout << "初始值: a = " << a << "\n";    
    // 2. 访问前锁定互斥锁
    // 使用lock_guard自动管理锁的生命周期
    {
        // 自动加锁，作用域结束后解锁
        std::lock_guard<std::mutex> lock(a_mutex);
        std::cout << "已获得锁 - 进入临界区\n";        
        // 3. 在锁保护下安全地访问和修改变量
        std::cout << "当前值: a = " << a << "\n";
        std::cout << "将a增加10\n";
        a += 10;       
        // 模拟一些操作（在真实场景中可能是复杂操作）
        for (int i = 0; i < 3; ++i) {
            std::cout << "操作中... ";
            a += i;
        }        
        std::cout << "\n操作完成，新值: a = " << a << "\n";
    } // lock_guard在此离开作用域，自动释放锁    
    std::cout << "锁已自动释放\n";   
    // 4. 验证最终结果
    std::cout << "最终值: a = " << a << "\n";   
    return 0;
}
```

> ## 互斥锁的核心作用原理
> 
> 互斥锁(Mutex, Mutual Exclusion)的核心原理是​**​控制访问权分配​**​：
> 
> 1. ​**​互斥性原理​**​：
>    
>    - 当线程获取锁时，它获得"独占访问权"
>    
>    - 其他线程若尝试获取同一把锁，会被阻塞或返回失败
>    
>    - 持有锁的线程执行完临界区代码后释放锁
>    
>    - 阻塞线程队列中的一个被唤醒并获得锁
> 
> 2. ​**​硬件支持​**​：
>    
>    - 现代CPU提供​**​原子操作指令​**​ (如x86的`LOCK`前缀)
>    
>    - 锁定过程实质是硬件级原子比较交换(CAS)操作
>    
>    - 确保锁状态变更不可分割、不可中断
> 
> 3. ​**​操作系统支持​**​：
>    
>    - 内核提供调度机制管理阻塞/唤醒线程
>    
>    - Linux使用`futex`(fast userspace mutex)
>    
>    - Windows使用同步原语对象

```cpp
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
// 1. 定义全局共享变量和对应的互斥锁
int shared_data = 0;        // 主线程和子线程共享的变量
std::mutex shared_data_mutex; // 保护shared_data的互斥锁
// 线程函数：在子线程中操作共享数据
void thread_work() {
    for (int i = 0; i < 5; i++) {
        // 3. 创建锁保护临界区
        {
            // 使用lock_guard自动加锁/解锁
            std::lock_guard<std::mutex> lock(shared_data_mutex);            
            // 4. 在锁保护下安全操作共享数据
            std::cout << "[子线程] 读取共享数据: " << shared_data << std::endl;            
            // 计算新值
            int new_value = shared_data + 2;
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 模拟耗时操作            
            // 更新共享数据
            shared_data = new_value;            
            std::cout << "[子线程] 更新共享数据为: " << new_value << std::endl;
        } // 锁自动释放        
        // 非临界区操作（不需要加锁）
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main() {
    // 2. 初始化共享数据
    shared_data = 100; 
    std::cout << "--- 主线程启动前 ---\n";
    std::cout << "共享数据初始值: " << shared_data << "\n\n";   
    // 创建子线程
    std::thread worker(thread_work);    
    // 主线程工作
    for (int i = 0; i < 5; i++) {
        // 3. 创建锁保护临界区
        {
            // 使用lock_guard自动加锁/解锁
            std::lock_guard<std::mutex> lock(shared_data_mutex);            
            // 4. 在锁保护下安全操作共享数据
            std::cout << "[主线程] 读取共享数据: " << shared_data << std::endl;            
            // 计算新值
            int new_value = shared_data + 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(15)); // 模拟耗时操作        
            // 更新共享数据
            shared_data = new_value;          
            std::cout << "[主线程] 更新共享数据为: " << new_value << std::endl;
        } // 锁自动释放        
        // 非临界区操作（不需要加锁）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    
    // 等待子线程结束
    worker.join();  
    std::cout << "\n--- 所有线程完成 ---\n";
    std::cout << "共享数据最终值: " << shared_data << std::endl;  
    return 0;
}
```

## 友元函数的主要作用

> ### 访问私有成员
> 
> - ​**​核心功能​**​：允许外部函数访问类的私有和保护成员
> 
> - ​**​使用场景​**​：需要跨越封装边界的特殊访问权限

### 友元函数关键特性

| 特性     | 说明                                      |
| ------ | --------------------------------------- |
| 单向性    | A 声明 B 为友元 → B 可访问 A 的私有成员 (不是相互的)      |
| 无传递性   | A 声明 B 为友元，B 声明 C 为友元 → C 不能访问 A 的私有成员  |
| 类外定义   | 友元函数在类外部实现，没有类作用域限制                     |
| 访问位置自由 | 可以在类定义的任意位置声明（public/protected/private） |
| 非成员性质  | 友元函数不是类的成员函数，没有 `this`指针                |

### **在 C++ 中，互斥锁 (`std::mutex`) 用于保护共享资源不被多个线程同时访问。以下是加锁和解锁的详细步骤及最佳实践**

> ### **​核心方法​**​
> 
> 1. ​**​`lock()`​**​ - 阻塞式加锁（若锁已被占用，则阻塞等待）
> 
> 2. ​**​`unlock()`​**​ - 手动释放锁
> 
> 3. ​**​`try_lock()`​**​ - 非阻塞尝试加锁（成功返回 `true`，失败返回 `false`）

```cpp
#include <mutex>
std::mutex mtx; // 声明互斥锁
void critical_section() {
    mtx.lock();   // 加锁（阻塞直到获得锁）    
    // ... 操作共享资源（临界区代码）...
    mtx.unlock(); // 解锁
}
```

### 共享资源（Shared Resource）

​**​共享资源​**​是指在多线程或多进程环境中，能被多个执行单元（如线程、进程）同时访问的数据或资源。这些资源通常存储在共享内存区域、文件系统、数据库等地方。由于多个执行单元可能​**​并发​**​地读写同一资源，如果没有适当的同步机制，会导致​**​数据竞争（Data Race）​**​，引发不可预测的结果（如数据损坏、程序崩溃）。

#### 常见的共享资源类型：

| ​**​资源类型​**​ | ​**​示例​**​        |
| ------------ | ----------------- |
| 内存数据         | 全局变量、静态变量、堆内存对象   |
| 文件/外部设备      | 文件、网络套接字、打印机      |
| 操作系统资源       | 数据库连接、GUI控件、系统API |

## c表较字符串

```cpp
#include <cstring>
#include <iostream>

int main() {
    const char* str1 = "hello";
    const char* str2 = "world";
    const char* str3 = "hello";

    // 比较结果:
    // 0 表示相等
    // <0 表示 str1 在字典序中小于 str2
    // >0 表示 str1 在字典序中大于 str2
    int result1 = std::strcmp(str1, str2);  // 返回负值 (h < w)
    int result2 = std::strcmp(str1, str3);  // 返回 0 (相等)

    if (result1 == 0) {
        std::cout << "Strings are equal" << std::endl;
    } else if (result1 < 0) {
        std::cout << str1 << " comes before " << str2 << std::endl;
    } else {
        std::cout << str1 << " comes after " << str2 << std::endl;
    }
}
```

**c++使用运算符重载或成员函数进行比较（推荐）**

```cpp
#include <string>
#include <iostream>

int main() {
    std::string s1 = "apple";
    std::string s2 = "banana";
    std::string s3 = "Apple";  // 注意大小写敏感

    // 相等性比较
    if (s1 == s3) {
        std::cout << "Equal (case-sensitive)" << std::endl;  // 不会执行
    }

    // 字典序比较
    if (s1 < s2) {  // a < b
        std::cout << s1 << " comes before " << s2 << std::endl;  // 会执行
    }

    // 不等式比较
    if (s1 != "orange") {
        std::cout << "Not an orange" << std::endl;
    }
}
```

### 1. ​**​在无竞争（no-contention）情况下的时间估算​**​：

- 假设`shared_data_mutex2`是一个标准的互斥锁（如C++11的`std::mutex`），并且没有其他线程持有或竞争该锁，则锁定和解锁操作通常非常快。

- 在現代多核CPU（如Intel或AMD的x86_64架构，时钟频率约为2-4 GHz）上：
  
  - `lock()`操作：在无竞争时，通常涉及一个原子指令（如compare-and-swap或类似操作），耗时约 ​**​20-100 纳秒​**​（nanoseconds）。
  
  - `unlock()`操作：通常是一个简单的原子存储指令，耗时稍短，约 ​**​10-50 纳秒​**​（因为解锁不需要复杂的状态检查）。
  
  - 总时间（从`lock()`开始到`unlock()`结束）：由于两个操作是连续的，总耗时大约在 ​**​50 到 200 纳秒​**​ 之间。

- 原因：
  
  - 这些操作在用户空间高效执行（例如，使用Linux的futex或Windows的轻量级锁），避免了系统调用。
  
  - 编译器优化（如内联）可以进一步减少函数调用开销。
  
  - 例如，在3 GHz CPU上，一个时钟周期约0.33纳秒，原子操作可能需要10-30个周期。

### 2. ​**​在有竞争（contention）情况下的时间估算​**​：

- 如果有其他线程持有锁或正在等待，`lock()`操作可能导致线程阻塞（等待），时间会显著增加：
  
  - `lock()`时间：可能从几微秒（µs）到几毫秒（ms），甚至更长，具体取决于线程调度、锁的实现（如自旋锁或休眠锁）以及系统负载。
  
  - `unlock()`时间：通常仍然很快（10-50纳秒），因为它只是释放锁，但如果有等待线程，操作系统可能需要唤醒其他线程，增加少量开销。
  
  - 总时间：无法可靠估算，可能从微秒级到数毫秒不等。

- 在实际多线程应用中，竞争是常见情况，所以如果有共享数据访问，应尽量避免锁竞争或使用更高效的并发机制（如原子变量或无锁数据结构）。

## 使用对象引用（推荐），线程

```cpp
void count(registration2& pos)  // 修改为接受引用参数
{
    while(ros::ok())
    {
        ros::Time tic = ros::Time::now();
        pos.calculate();  // 使用传入的对象
        ROS_INFO("total Time: %f", (ros::Time::now() - tic).toSec());
    }
    std::cout<<"out count id: "<< std::this_thread::get_id() << std::endl;
}

int main(int argc, char** argv)
{
    // ... [其他初始化代码] ...
    registration2 position;  // 在主线程中创建对象

    // 使用std::ref传递对象引用
    std::thread worker(count, std::ref(position));

    // ... [其余代码保持不变] ...
}
```

关键修改：

1. 将 `count()`函数改为接受 `registration2&`引用参数

2. 在创建线程时使用 `std::ref(position)`传递对象引用

### 核心原则：没有数据竞争（Data Race）

C++标准规定：​**​当多个线程同时访问同一个内存位置，且至少有一个访问是写操作时​**​，才会导致未定义行为（如崩溃或数据损坏）。而​**​纯读取操作（无任何修改）不会触发数据竞争​**​。

### 在 C++ 中，`static_cast<int>` 是一种​**​显式类型转换操作符​**​，用于将其他类型的数据转换为 `int` 类型

```cpp
int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
```

## C++ 中，`atan` 函数（来自 `<cmath>` 头文件）​**​返回的是弧度制的反正切值​**​，而非角度制

### 关键说明：

1. ​**​数学库的通用设计​**​：C/C++ 的标准数学库（如 `<cmath>`）中，所有三角函数（`sin`、`cos`、`tan`、`atan` 等）均以​**​弧度​**​作为输入和输出的默认单位。这是数学领域的通用约定（例如，微积分中的三角函数也以弧度为基准）。

2. ​**​`atan` 的具体行为​**​：
   
   - 输入：参数是弧度制的数值（表示一个角的正切值）。
   - 输出：返回该角的反正切值，范围是 `(-π/2, π/2)` 弧度（即从 -90 度到 90 度，但不包含端点）。

### 示例：弧度与角度的转换

若需要将 `atan` 的结果从弧度转换为角度，可以通过以下公式：

角度=弧度×π180​

```cpp
#include <iostream>
#include <cmath>

int main() {
    double radian = std::atan(1);  // 结果为 π/4 弧度（约 0.7854 弧度）
    double degree = radian * (180.0 / M_PI);  // 转换为角度（45 度）

    std::cout << "atan(1) 的弧度值: " << radian << std::endl;       // 输出约 0.7854
    std::cout << "对应的角度值: " << degree << " 度" << std::endl;  // 输出 45 度
    return 0;
}
```

## **容器取出头部数据​**​

#### ​**​方式 1：使用 `front()`成员函数​**​

```cpp
#include <vector>

struct Detection {
    // 假设 Detection 的成员（如坐标、类别等）
    int id;
    float confidence;
};

int main() {
    std::vector<Detection> detections;

    // 假设已填充数据（示例）
    detections.push_back({1, 0.9f});
    detections.push_back({2, 0.8f});

    // 取出头部数据（需先检查容器非空！）
    if (!detections.empty()) {
        Detection& front_detection = detections.front(); // 引用第一个元素
        // 使用 front_detection（例如打印 id）
        printf("Head detection id: %d
", front_detection.id); 
    } else {
        printf("detections is empty!
");
    }

    return 0;
}
```

#### **​方式 2：通过索引 `[0]`访问​**

**注意​**​：同样需要先检查容器非空，否则 `detections[0]`在空容器时会触发未定义行为（标准未禁止，但实际会崩溃）。

```cpp
if (!detections.empty()) {
    Detection& front_detection = detections[0]; // 与 front() 等价
    // 使用 front_detection...
}
```

### **​2. 删除头部数据​**​

```cpp
if (!detections.empty()) {
    // 删除头部元素（迭代器指向第一个元素）
    detections.erase(detections.begin()); 

    // 此时容器大小减 1，原第二个元素变为新的第一个元素
    if (!detections.empty()) {
        printf("New head id: %d
", detections.front().id); // 输出原第二个元素的 id
    }
}
```

## vectoer<thread>

```cpp
// 普通函数示例（可替换为Lambda）
void worker_task(int id, const std::string& msg) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟工作
    std::cout << "Thread " << id << " 执行: " << msg << std::endl;
}
std::vector<std::thread> workers;

// 创建3个线程，每个线程执行worker_task
for (int i = 0; i < 3; ++i) {
    workers.emplace_back(
        worker_task,  // 线程函数
        i,            // 第一个参数（线程ID）
        "Hello World" // 第二个参数（消息）
    );
}
// 等待所有线程完成
//线程对象在vector中必须处于​​非连接状态​​（unjoinable）才能安全销毁，
//否则程序会调用std::terminate()崩溃。
for (auto& t : workers) {
    if (t.joinable()) { // 检查是否可连接（避免重复join）
        t.join();       // 阻塞主线程，直到子线程结束
    }
}
std::cout << "所有线程执行完毕！" << std::endl;
```

   清空容器

```cpp
 td::vector<model> filter_;
 filter_.clear();
```

## 一、现代 C++ 核心方案：`<chrono>` 库

`<chrono>` 是 C++11 引入的时间标准库，核心组件包括：

- **时钟（Clock）**：提供获取当前时间的接口，分 3 种常用类型；
- **时间点（time_point）**：表示某个时钟下的具体时刻；
- **时长（duration）**：表示两个时间点的差值（如 1 秒、500 毫秒）

### 1. 先理解 3 种核心时钟

选择正确的时钟是计时的关键，不同时钟用途不同：

| 时钟类型                                 | 特点                                             | 适用场景           |
| ------------------------------------ | ---------------------------------------------- | -------------- |
| `std::chrono::system_clock`          | 关联系统时间，可被手动修改（如调整系统时区、同步 NTP）                  | 获取当前日历时间（如时间戳） |
| `std::chrono::steady_clock`          | 单调递增时钟，**不受系统时间修改影响**，精度最高                     | 测量代码耗时（推荐首选）   |
| `std::chrono::high_resolution_clock` | 通常是 `steady_clock` 或 `system_clock` 的别名，追求最高精度 | 对精度要求极高的计时     |

```cpp
#include <chrono>  // 核心计时库
#include <iostream>
int main() {
    // 1. 记录开始时间点（steady_clock的时间点）
    auto start = std::chrono::steady_clock::now();

    // 2. 待测试的目标代码（示例：执行1e6次循环）
    long long sum = 0;
    for (int i = 0; i < 1'000'000; ++i) {
        sum += i;
    }

    // 3. 记录结束时间点
    auto end = std::chrono::steady_clock::now();

    // 4. 计算时间差（duration类型，默认单位：纳秒）
    auto duration = end - start;

    // 5. 转换为常用单位并输出（毫秒、微秒、秒等）
    // 用 duration_cast 强制转换时长单位
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();

    std::cout << "循环耗时：\n";
    std::cout << ns << " 纳秒\n";
    std::cout << us << " 微秒\n";
    std::cout << ms << " 毫秒\n";

    // 转换为带小数的秒（以纳秒为中间单位计算）
    // duration.count()返回纳秒数，除以1e9得到秒
    double seconds_double = static_cast<double>(duration.count()) / 1e9;

    return 0;
}
```

## 重点

你的代码存在 ​**​ROS 回调队列被多线程并发驱动​**​ 的核心问题，这会直接导致 ROS 内部的竞态条件（Race Condition）和未定义行为。以下是具体分析和解决方案：

### ​问题1：多线程同时调用 `ros::spin()`，导致回调队列竞争​

三个线程同时调用同一个 `NodeHandle nh`的 `ros::spin()`，这会导致 ​**​同一个回调队列被三个线程并发处理​**​。ROS 的回调队列 ​**​不是线程安全的​**​，多个线程同时操作会导致：

- 回调函数被重复执行或遗漏；

- ROS 内部数据结构（如订阅者列表、消息队列）被并发修改，引发崩溃；

触发 ROS 的 `FATAL`错误（如你遇到的 `SingleThreadedSpinner: Attempt to spin a callback queue from two spinners`）。

**

解决办法： ​

```
void worker_task1(ros::NodeHandle nh)
{
    ros::Rate sl(1000);
    ros::Subscriber tf_sub = nh.subscribe("/tf", 10, tfCallback);
    while (ros::ok())
    {
        /* code */

        ros::spinOnce();
        sl.sleep();
    }



}
void worker_task2(ros::NodeHandle nh)
{
    //ros::Subscriber tf2_sub = global.nh.subscribe("/target", 2, tf2Callback);

    ros::Rate sl(1000);
    ros::Subscriber map_sub = nh.subscribe("/serial/map", 10, mapCallback);
    while (ros::ok())
    {
        /* code */

        ros::spinOnce();
        sl.sleep();
    }

}
//回调不能用ros::spin()
void worker_task3(ros::NodeHandle nh)
{

    ros::Rate sl(1000);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2, imuCallback);
    while (ros::ok())
    {
        /* code */

        ros::spinOnce();
        sl.sleep();
    }  
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    // nh.param<int>("image_width", image_width, 640);
    // nh.param<int>("image_height", image_height, 480);
    std::string port;
    //nh.param<std::string>("serial_port", port, "/dev/ttyUSB0");
    nh.param<std::string>("serial_port", port, "/dev/ttyACM0");
    try {
        global.serialComm = new serial_mcu(port);
        ROS_INFO("Serial port initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        return -1;
    }

    std::vector<std::thread> workers;
    //线程传参
    workers.emplace_back(worker_task1, nh);
    workers.emplace_back(worker_task2, nh);
    workers.emplace_back(worker_task3, nh);
}
```
