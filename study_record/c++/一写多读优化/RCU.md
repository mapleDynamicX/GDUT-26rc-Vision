### 4. RCU（Read-Copy-Update）：极致多读性能，写无阻塞

#### 算法思想

RCU 是 Linux 内核的经典算法，核心是：

1. 读取：无锁、无原子操作（仅内存屏障），极致快；

2. 写入：
   
   - 步骤 1：生成数据副本，修改副本；
   - 步骤 2：原子替换指向数据的指针；
   - 步骤 3：等待所有 “旧读线程” 完成后，释放旧数据（“宽限期”）；
- 保证：写线程全程不阻塞读线程，读线程无任何开销。

#### 开源实现

- **liburcu**（用户态 RCU，[GitHub](https://github.com/urcu/userspace-rcu)）：最成熟的用户态 RCU 库，支持 SWMR 场景，适配 C 语言，性能接近内核 RCU；
- **folly::RCU**（Facebook Folly，[GitHub](https://github.com/facebook/folly)）：C++ 封装的 RCU，更易用，支持自动宽限期管理；
- **rcu++**（[GitHub](https://github.com/mpoeter/rcu)）：极简的 C++ RCU 实现，适合学习 / 轻量集成。

#### 性能特点

- 写入速度：替换指针是 O (1)，但宽限期管理有少量开销（可忽略）；
- 读取速度：比原子指针更快（无原子操作，仅内存屏障）；
- 实时性：读线程能立即看到指针替换后的最新数据；
- 适用场景：读极高频、写低频（但写不能被阻塞）的场景（比如配置中心、缓存更新）。

#### 代码示例（liburcu 极简版）

```cpp
#include <urcu/urcu-read-copy.h>
#include <stdio.h>
#include <pthread.h>

struct data {
    int value;
};
struct data *g_data; // 被 RCU 保护的指针

// 写线程
void *writer(void *arg) {
    struct data *new_data = malloc(sizeof(struct data));
    new_data->value = 42;
    // 原子替换指针
    rcu_assign_pointer(g_data, new_data);
    // 等待旧读线程完成，释放旧数据
    synchronize_rcu();
    free(old_data);
    return NULL;
}

// 读线程
void *reader(void *arg) {
    struct data *data;
    // 进入 RCU 读临界区（无锁）
    rcu_read_lock();
    data = rcu_dereference(g_data); // 读取指针（无原子操作）
    if (data) {
        printf("Read value: %d\n", data->value);
    }
    // 退出读临界区
    rcu_read_unlock();
    return NULL;
}
```

## 二、选型建议（按场景匹配）

| 场景                | 推荐方案                    | 核心优势             |
| ----------------- | ----------------------- | ---------------- |
| 数据量小、更新频繁、实时性要求极高 | 原子指针 + 不可变数据            | 写入零阻塞、读取无锁、集成成本低 |
| 数据量大、随机更新         | RCU（liburcu/folly::RCU） | 读无开销、写不阻塞读       |
| 流式数据（日志 / 传感器）    | 无锁环形缓冲区                 | 缓存友好、写入速度接近内存拷贝  |
| 读写都频繁、需兼容传统锁逻辑    | 偏向写的 RWLock（abseil）     | 平衡读写性能、无写饥饿      |

## 三、关键优化点（保证实时性 + 写入速度）

1. **内存序优化**：使用轻量级内存序（比如 `std::memory_order_acquire/release` 而非 `seq_cst`），减少原子操作的开销；
2. **避免拷贝**：用 COW（写时拷贝）、增量更新代替全量快照，降低写入时的拷贝开销；
3. **缓存对齐**：将原子变量 / 指针放在独立缓存行（比如 `alignas(64)`），避免伪共享（False Sharing）；
4. **延迟释放**：旧数据不要立即释放（避免读线程访问时崩溃），用引用计数 / RCU 宽限期管理；
5. **用户态实现**：避免内核态锁（比如 pthread_mutex），减少上下文切换开销。

## 四、总结

GitHub 上最适合 “一写多读、高实时性、不影响写入速度” 的方案：

- 入门首选：**原子指针 + 不可变数据**（零依赖、易实现）；
- 高性能多读：**RCU（liburcu）**（读无开销，写不阻塞）；
- 流式数据：**无锁环形缓冲区（moodycamel）**；
- 平衡读写：**abseil::ReaderWriterMutex**。

```cpp
// 引入标准输入输出库，提供printf/snprintf等打印函数，用于输出线程读写日志
#include <stdio.h>
// 引入标准库，提供malloc/free（内存分配/释放）、usleep（微秒级休眠）等函数
#include <stdlib.h>
// 引入POSIX线程库，提供pthread_create/pthread_join/pthread_cleanup等线程操作函数
#include <pthread.h>
// 引入Userspace RCU的memb模块头文件（核心依赖）
// 提供urcu_memb_read_lock/urcu_memb_read_unlock（读临界区）、urcu_memb_synchronize_rcu（宽限期等待）等核心函数
// 头文件约束：所有包含读临界区的线程必须先调用urcu_memb_register_thread注册
#include <urcu/urcu-memb.h> 

// 定义RCU保护的数据结构：RCU核心原则是"读无锁、写拷贝"，该结构不可直接修改，修改需创建新实例
typedef struct {
    int value;        // 整型业务数据，模拟待读写的数值
    char name[32];    // 字符串业务数据，模拟带格式的名称
} Data;

// 全局共享指针，作为所有读写线程的核心数据入口，由RCU memb机制保护其读写安全性
Data *g_data;

// 读线程清理函数：线程退出时注销RCU线程注册，避免资源泄漏
// 参数arg：无实际意义，仅满足pthread_cleanup_push的函数签名要求
static void reader_cleanup(void *arg) {
    // 注销RCU读线程：必须在线程退出前调用，释放RCU内部的线程资源
    urcu_memb_unregister_thread();
}

// 读线程执行函数：RCU读操作核心逻辑，无锁、无阻塞，可同时运行多个读线程
// 参数arg：传入线程ID（long类型），用于日志区分不同读线程
void *reader_thread(void *arg) {
    // 将void*类型的线程参数转换为long型，作为读线程唯一标识（tid）
    long tid = (long)arg;

    // 【必须】注册RCU读线程：头文件明确要求，读临界区使用前必须注册
    urcu_memb_register_thread();
    // 注册线程清理函数：确保线程异常退出时也能注销RCU线程（pthread_cleanup_pop触发）
    pthread_cleanup_push(reader_cleanup, NULL);

    // 无限循环模拟持续的读操作（实际场景可根据业务逻辑退出）
    while (1) {
        // 1. 进入RCU memb读临界区：仅插入内存屏障，无锁/原子操作开销
        // 语义：告知RCU机制，当前线程开始读取受保护的指针，宽限期回收需等待该线程退出临界区
        urcu_memb_read_lock();

        // 2. 安全读取RCU保护的全局指针：rcu_dereference来自urcu/pointer.h，插入内存屏障确保指针稳定
        // 读操作无锁、无原子操作，是RCU"读无锁"的核心体现
        Data *data = rcu_dereference(g_data);
        // 判空：防止指针未初始化时访问空指针
        if (data) {
            // 打印读线程ID、读取到的value和name，展示RCU读操作结果
            printf("Reader %ld: value=%d, name=%s\n", tid, data->value, data->name);
        }

        // 3. 退出RCU memb读临界区：仅插入内存屏障，标记读操作结束
        // 语义：告知RCU机制，当前线程已完成读操作，宽限期回收可开始计数
        urcu_memb_read_unlock();

        // 微秒级休眠（100微秒），模拟读操作的业务耗时，降低日志输出频率
        usleep(100);
    }

    // 触发清理函数（无限循环不会执行到此处，仅保证语法完整性）
    pthread_cleanup_pop(1);
    // 线程退出返回值（无限循环不会执行到此处，仅满足pthread线程函数签名要求）
    return NULL;
}

// 写线程清理函数：线程退出时注销RCU线程注册
// 参数arg：无实际意义，仅满足pthread_cleanup_push的函数签名要求
static void writer_cleanup(void *arg) {
    // 注销RCU写线程：写线程中使用了rcu_dereference（读操作），因此也需注销
    urcu_memb_unregister_thread();
}

// 写线程执行函数：RCU写操作核心逻辑，"写拷贝+原子替换+宽限期回收"，全局仅一个写线程（避免写竞争）
// 参数arg：无实际使用，仅满足pthread线程函数签名要求
void *writer_thread(void *arg) {
    // 初始化计数器，用于更新Data的value字段，模拟业务数据的递增修改
    int count = 0;

    // 【必须】注册RCU写线程：写线程中调用了rcu_dereference（读操作），因此也需注册
    urcu_memb_register_thread();
    // 注册线程清理函数：确保线程异常退出时注销RCU线程
    pthread_cleanup_push(writer_cleanup, NULL);

    // 无限循环模拟持续的写操作（实际场景可根据业务逻辑退出）
    while (1) {
        // 1. 第一步：拷贝旧数据并修改（RCU"写拷贝"核心）
        // 安全读取旧的全局指针：rcu_dereference确保读取到稳定的旧数据地址
        Data *old_data = rcu_dereference(g_data);
        // 分配新的Data内存：RCU不允许修改旧数据，必须创建新实例
        Data *new_data = malloc(sizeof(Data));
        // 如果旧数据存在，将旧数据内容拷贝到新实例（仅拷贝，不修改旧数据）
        if (old_data) {
            memcpy(new_data, old_data, sizeof(Data));
        }
        // 修改新数据的value：递增计数器，模拟业务数据更新（无锁、无竞争）
        new_data->value = count++;
        // 格式化新数据的name：根据value生成带标识的字符串（无锁、无竞争）
        snprintf(new_data->name, 32, "RCU-Data-%d", new_data->value);

        // 2. 第二步：原子替换全局指针（写无阻塞读）
        // rcu_assign_pointer：原子操作替换g_data指针，内存屏障确保新指针对所有读线程可见
        rcu_assign_pointer(g_data, new_data);

        // 3. 第三步：等待RCU memb宽限期结束
        // urcu_memb_synchronize_rcu()：阻塞等待所有已进入读临界区的线程退出
        // 宽限期结束后，旧数据不再被任何读线程引用，可安全释放
        urcu_memb_synchronize_rcu();

        // 如果旧数据存在，释放其内存（避免内存泄漏）
        if (old_data) {
            free(old_data);
            // 打印释放日志，确认旧数据已被安全回收
            printf("Writer: freed old data (value=%d)\n", old_data->value);
        }

        // 毫秒级休眠（1000微秒=1毫秒），模拟写操作的业务间隔，降低写频率
        usleep(1000);
    }

    // 触发清理函数（无限循环不会执行到此处，仅保证语法完整性）
    pthread_cleanup_pop(1);
    // 线程退出返回值（无限循环不会执行到此处，仅满足pthread线程函数签名要求）
    return NULL;
}

// 主函数：初始化RCU、创建读写线程、等待线程执行
int main() {
    // 初始化URCU memb模块：必须在创建线程前执行，初始化RCU核心机制
    urcu_memb_init();

    // 初始化全局RCU指针：分配第一个Data实例，作为读写线程的初始数据
    g_data = malloc(sizeof(Data));
    // 设置初始value为0
    g_data->value = 0;
    // 格式化初始name为"RCU-Data-0"
    snprintf(g_data->name, 32, "RCU-Data-0");

    // 定义5个读线程的ID数组：用于存储pthread_create返回的线程句柄
    pthread_t readers[5];
    // 循环创建5个读线程
    for (long i = 0; i < 5; i++) {
        // pthread_create参数说明：
        // &readers[i]：存储新线程的句柄；NULL：使用默认线程属性；
        // reader_thread：读线程执行函数；(void *)i：传入线程ID作为参数
        pthread_create(&readers[i], NULL, reader_thread, (void *)i);
    }

    // 定义写线程的ID：存储写线程句柄
    pthread_t writer;
    // 创建1个写线程：参数同读线程，无传入参数（arg为NULL）
    pthread_create(&writer, NULL, writer_thread, NULL);

    // 等待写线程退出：阻塞主线程，直到写线程结束（实际写线程是无限循环，不会退出）
    // 实际场景中可替换为信号处理、业务逻辑等，此处仅保证主线程不提前退出
    pthread_join(writer, NULL);

    // 注：urcu-memb.h未定义urcu_memb_cleanup，因此删除原错误的rcu_memb_cleanup调用

    // 主函数正常退出返回值
    return 0;
}
```

```bash
sudo apt install -y liburcu-dev
```

### 二、逐行语法解析

#### 1. 第一行：`data_ = static_cast<T*>(malloc(sizeof(T)));`

这行的核心是**C 风格内存分配 + C++ 类型安全转换**，拆解如下：

##### (1) `malloc(sizeof(T))`

- **malloc**：C 标准库函数，定义在 `<cstdlib>` 中，作用是**分配原始、未初始化的字节内存**，不调用任何构造函数。
  - 入参：`sizeof(T)` 计算类型 `T` 的字节大小（比如 `T=int` 时是 4，`T=std::string` 时是平台相关的大小，如 8）。
  - 返回值：`void*` 类型（指向分配内存的起始地址），如果分配失败返回 `nullptr`（C++11 后）/ `NULL`（旧标准）。
  - 关键特性：malloc 只分配内存，不对内存做任何初始化 —— 内存中的值是随机的 “脏数据”。

##### (2) `static_cast<T*>(...)`

- **static_cast**：C++ 的静态类型转换（编译期检查），是 C++ 替代 C 风格强制转换（`(T*)malloc(...)`）的安全方式。
  
  - 作用：将 `malloc` 返回的 `void*` 转换为 `T*` 类型（指向 `T` 类型对象的指针）。
  
  - 为什么不用 C 风格转换？
    
    C 风格转换（`(T*)`）会跳过编译期类型检查，而 `static_cast` 仅允许 “合理的” 类型转换（如 `void*` 转具体指针、数值类型转换等），更安全，也更易读。
  
  - 注意：`static_cast` 不做运行时类型检查（区别于 `dynamic_cast`），这里转换 `void*` 是其典型合法场景。

##### (3) 赋值给 `data_`

`data_` 作为 `T*` 类型的指针，接收转换后的内存地址，此时 `data_` 指向的是**原始内存**（未构造 `T` 对象）。

#### 2. 第二行：`if (data_) { ... }`

- 检查 `malloc` 是否成功：如果 `malloc` 分配失败，返回 `nullptr`（布尔值为 false），则跳过后续构造逻辑；分配成功则进入分支。
- 注意：在 C++ 中，指针可以隐式转换为布尔值 ——`nullptr` 为 false，非空指针为 true。

#### 3. 第三行：`new (data_) T();`

这是 C++ 的**定位 new 表达式（Placement New）**，是这段代码的核心，也是 C++ 手动管理对象生命周期的关键语法。

```cpp
// 分配初始数据内存（默认构造T类型对象）
data_ = static_cast<T*>(malloc(sizeof(T)));
if (data_) {
    // 调用T的默认构造函数（针对非POD类型）
    new (data_) T();
}
```

```cpp
#include <urcu/urcu-memb.h>
#include <urcu/pointer.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <pthread.h>

// 单写多读的RCU模板类
template<typename T>
class Ten_one_write_multiple_read
{
public:
    // 禁用拷贝构造和赋值运算符（避免RCU指针被非法拷贝）
    Ten_one_write_multiple_read(const Ten_one_write_multiple_read& wr) = delete;
    Ten_one_write_multiple_read& operator=(const Ten_one_write_multiple_read& wr) = delete;

    // 构造函数：初始化RCU并分配初始数据内存
    Ten_one_write_multiple_read()
        : data_(nullptr)
    {
        // 初始化URCU的memb模块（全局仅需初始化一次，此处简化为类内初始化）
        static bool rcu_inited = false;
        if (!rcu_inited) {
            urcu_memb_init();
            rcu_inited = true;
        }

        // 分配初始数据内存（默认构造T类型对象）
        data_ = static_cast<T*>(malloc(sizeof(T)));
        if (data_) {
            // 调用T的默认构造函数（针对非POD类型）
            new (data_) T();
        }

        // 注册当前线程（如果构造函数在读写线程中调用，需确保RCU线程注册）
        urcu_memb_register_thread();
    }

    // 析构函数：清理RCU资源和数据内存
    ~Ten_one_write_multiple_read()
    {
        // 等待RCU宽限期结束，确保所有读线程已退出临界区
        urcu_memb_synchronize_rcu();

        // 释放数据内存（调用T的析构函数）
        if (data_) {
            data_->~T();
            free(data_);
            data_ = nullptr;
        }

        // 注销RCU线程
        urcu_memb_unregister_thread();
    }

    // 写数据：RCU写拷贝+原子替换+宽限期回收
    void write_data(const T& new_data)
    {
        // 1. 读旧数据（RCU安全读取）
        T* old_data = rcu_dereference(data_);

        // 2. 分配新内存并拷贝数据（写拷贝核心）
        T* new_data_ptr = static_cast<T*>(malloc(sizeof(T)));
        if (!new_data_ptr) {
            perror("malloc failed in write_data");
            return;
        }

        // 拷贝新数据（调用T的拷贝构造/赋值）
        new (new_data_ptr) T(new_data);

        // 3. 原子替换全局指针（RCU写操作核心）
        rcu_assign_pointer(data_, new_data_ptr);

        // 4. 等待RCU宽限期结束，确保旧数据不再被读线程引用
        urcu_memb_synchronize_rcu();

        // 5. 释放旧数据内存
        if (old_data) {
            old_data->~T(); // 调用析构函数
            free(old_data);
        }
    }

    // 读数据：RCU无锁读（线程安全）
    T read_data()
    {
        // 1. 进入RCU读临界区（无锁，仅内存屏障）
        urcu_memb_read_lock();

        // 2. 安全读取RCU保护的指针
        T* data = rcu_dereference(data_);
        T ret;
        if (data) {
            ret = *data; // 拷贝数据（线程安全，旧数据不会被修改）
        }

        // 3. 退出RCU读临界区
        urcu_memb_read_unlock();

        return ret;
    }

private:
    T* data_; // RCU保护的核心数据指针
};

// ------------------------------ 测试代码 ------------------------------
// 测试用的数据结构（与原代码兼容）
struct Data {
    int value{0};
    char name[32]{};

    // 空构造函数（默认初始化）
    Data() = default;

    // 拷贝构造函数
    Data(const Data& other) {
        value = other.value;
        memcpy(name, other.name, sizeof(name));
    }

    // 赋值运算符
    Data& operator=(const Data& other) {
        if (this != &other) {
            value = other.value;
            memcpy(name, other.name, sizeof(name));
        }
        return *this;
    }

    // 析构函数（空，仅为兼容模板类的析构逻辑）
    ~Data() = default;
};

// 测试写线程函数（模拟单写）
void* writer_func(void* arg) {
    auto* rcu_obj = static_cast<Ten_one_write_multiple_read<Data>*>(arg);
    int count = 0;

    // 注册当前线程到RCU（写线程必须注册）
    urcu_memb_register_thread();

    while (1) {
        // 构造新数据
        Data new_data;
        new_data.value = count++;
        snprintf(new_data.name, 32, "RCU-Data-%d", new_data.value);

        // 写数据
        rcu_obj->write_data(new_data);

        // 打印日志
        printf("Writer: write value=%d, name=%s\n", new_data.value, new_data.name);

        // 模拟写间隔
        usleep(1000 * 1000); // 1秒
    }

    urcu_memb_unregister_thread();
    return nullptr;
}

// 测试读线程函数（模拟多读）
void* reader_func(void* arg) {
    auto* args = static_cast<std::pair<Ten_one_write_multiple_read<Data>*, long>*>(arg);
    auto* rcu_obj = args->first;
    long tid = args->second;

    // 注册当前线程到RCU（读线程必须注册）
    urcu_memb_register_thread();

    while (1) {
        // 读数据
        Data data = rcu_obj->read_data();

        // 打印日志
        printf("Reader %ld: read value=%d, name=%s\n", tid, data.value, data.name);

        // 模拟读间隔
        usleep(100 * 1000); // 100毫秒
    }

    urcu_memb_unregister_thread();
    return nullptr;
}

// 主函数：测试模板类
int main() {
    // 创建RCU对象
    Ten_one_write_multiple_read<Data> rcu_data;

    // 创建写线程
    pthread_t writer;
    pthread_create(&writer, nullptr, writer_func, &rcu_data);

    // 创建5个读线程
    pthread_t readers[5];
    std::pair<Ten_one_write_multiple_read<Data>*, long> reader_args[5];
    for (long i = 0; i < 5; ++i) {
        reader_args[i] = {&rcu_data, i};
        pthread_create(&readers[i], nullptr, reader_func, &reader_args[i]);
    }

    // 等待线程退出（实际为无限循环）
    pthread_join(writer, nullptr);
    for (long i = 0; i < 5; ++i) {
        pthread_join(readers[i], nullptr);
    }

    return 0;
}
```

#### 步骤 2：安装 liburcu 开发包

`<urcu/urcu-memb.h>` 属于 `liburcu-dev` 包（`-dev` 后缀表示开发包，包含头文件和静态库），执行安装：

```bash
sudo apt install -y liburcu-devbash
```
