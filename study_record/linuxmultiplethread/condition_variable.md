# condition_variable

## 1. 什么是 `std::condition_variable`

`std::condition_variable` 是 C++11 引入的同步原语，用于**线程间的通信**。它可以让一个线程等待某个条件成立，而其他线程可以在条件成立时通知它。

- **核心作用**：解决线程间的等待 - 通知问题。
- **依赖**：必须与 `std::mutex` 配合使用。
- **头文件**：`<condition_variable>`

---

## 2. 工作原理

`std::condition_variable` 的工作机制可以简单概括为：

1. **等待线程**：
   
   - 获取互斥锁 (`std::unique_lock`)。
   - 检查条件，如果条件不满足，则调用 `wait()` 方法。
   - `wait()` 会**原子地**释放锁并阻塞线程，直到被通知。
   - 当线程被唤醒时，它会重新获取锁，并再次检查条件（防止虚假唤醒）。

2. **通知线程**：
   
   - 获取互斥锁。
   - 修改共享变量（使条件成立）。
   - 调用 `notify_one()` 或 `notify_all()` 通知等待的线程。
   - 释放锁。

```cpp
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

std::mutex mtx;
std::condition_variable cv;
std::queue<int> q;

void producer() {
    for (int i = 0; i < 5; ++i) {
        std::unique_lock<std::mutex> lock(mtx);
        q.push(i);
        std::cout << "Produced: " << i << std::endl;
        lock.unlock(); // 可以提前解锁，让消费者更快获取锁
        cv.notify_one(); // 通知消费者
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void consumer() {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        // 等待条件：队列不为空
        cv.wait(lock, []() { return !q.empty(); });

        int data = q.front();
        q.pop();
        std::cout << "Consumed: " << data << std::endl;

        if (data == 4) break; // 结束标志
    }
}

int main() {
    std::thread t1(producer);
    std::thread t2(consumer);

    t1.join();
    t2.join();

    return 0;
}
```

```textile
Produced: 0
Consumed: 0
Produced: 1
Consumed: 1
Produced: 2
Consumed: 2
Produced: 3
Consumed: 3
Produced: 4
Consumed: 4
```

好的，我们来深入探讨 C++ 中的 Lambda 表达式。

Lambda 表达式是 C++11 引入的一项强大特性，它定义了一个**匿名函数**，可以捕获作用域中的变量，并且可以像对象一样被传递和使用。

### 核心概念

Lambda 表达式的核心是**创建一个可调用的代码单元**，而无需显式定义一个函数。它通常用于：

- **STL 算法**：作为 `std::sort`、`std::find_if` 等函数的谓词（判断条件）。
- **多线程编程**：作为 `std::thread` 的执行函数，或 `std::condition_variable` 的等待条件。
- **回调函数**：作为函数的参数，在特定事件发生时被调用。

### 语法结构

Lambda 表达式的语法非常灵活，基本结构如下：

```cpp
[capture-clause](parameters) mutable -> return-type { body }
```

各部分解释：

1. **捕获子句 (`capture-clause`)**
   
   - 定义了 Lambda 表达式可以访问外部作用域中的哪些变量。
   - 格式：`[]` 中可以包含变量名、引用符号 `&` 或 `this` 指针。
   - 示例：
     - `[]`：不捕获任何变量。
     - `[x]`：按值捕获变量 `x`（拷贝一份）。
     - `[&x]`：按引用捕获变量 `x`（直接引用原变量）。
     - `[x, &y]`：按值捕获 `x`，按引用捕获 `y`。
     - `[=]`：按值捕获所有外部变量。
     - `[&]`：按引用捕获所有外部变量。
     - `[this]`：在类成员函数中，捕获当前对象的指针。

2. **参数列表 (`parameters`)**
   
   - 与普通函数的参数列表类似。
   - 示例：`(int x, int y)`。
   - 如果没有参数，可以省略括号：`()`。

3. **`mutable` 关键字**
   
   - 允许 Lambda 表达式修改按值捕获的变量。
   - 因为按值捕获的变量默认是 `const` 的，`mutable` 可以解除这个限制。

4. **返回类型 (`-> return-type`)**
   
   - 显式指定 Lambda 表达式的返回类型。
   - 如果 Lambda 体中只有一个 `return` 语句，编译器可以自动推导返回类型，此时可以省略。

5. **函数体 (`body`)**
   
   - Lambda 表达式的执行代码，与普通函数体相同。

### 用法示例

#### 1. 基本用法

```cpp
#include <iostream>

int main() {
    // 定义一个简单的 Lambda 表达式
    auto add = [](int a, int b) { return a + b; };

    // 调用 Lambda 表达式
    std::cout << add(3, 5) << std::endl; // 输出：8

    return 0;
}
```

#### 2. 捕获变量

```cpp
#include <iostream>

int main() {
    int x = 10;
    int y = 20;

    // 按值捕获 x，按引用捕获 y
    auto lambda = [x, &y]() {
        // x = 100; // 错误：按值捕获的变量默认是 const 的
        y = 200; // 正确：按引用捕获的变量可以修改
        std::cout << "x: " << x << ", y: " << y << std::endl;
    };

    lambda(); // 输出：x: 10, y: 200
    std::cout << "After lambda, y: " << y << std::endl; // 输出：200

    return 0;
}
```

#### 3. `mutable` 关键字

```cpp
#include <iostream>

int main() {
    int x = 10;

    // 使用 mutable 允许修改按值捕获的变量
    auto lambda = [x]() mutable {
        x = 100;
        std::cout << "Inside lambda, x: " << x << std::endl;
    };

    lambda(); // 输出：Inside lambda, x: 100
    std::cout << "Outside lambda, x: " << x << std::endl; // 输出：10（原变量未改变）

    return 0;
}
```

#### 4. 作为 STL 算法的谓词

```cpp
#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<int> nums = {1, 2, 3, 4, 5};

    // 使用 Lambda 表达式作为 std::find_if 的谓词
    auto it = std::find_if(nums.begin(), nums.end(), [](int num) {
        return num > 3;
    });

    if (it != nums.end()) {
        std::cout << "First number greater than 3: " << *it << std::endl; // 输出：4
    }

    return 0;
}
```

#### 5. 在多线程中使用

```cpp
#include <iostream>
#include <thread>

int main() {
    int x = 0;

    // Lambda 表达式作为线程的执行函数
    std::thread t([&x]() {
        for (int i = 0; i < 100000; ++i) {
            ++x;
        }
    });

    t.join();

    std::cout << "x: " << x << std::endl; // 输出：100000

    return 0;
}
```

### 总结

Lambda 表达式的核心优势：

1. **简洁**：避免了定义独立函数或函数对象的麻烦。
2. **灵活**：可以捕获外部变量，适应不同的场景。
3. **高效**：编译器通常会将 Lambda 表达式内联，性能开销小。

它是 C++ 中非常重要的特性，尤其是在现代 C++ 开发中，被广泛用于简化代码和提高可读性。

编辑分享

详细介绍一下lambda表达式的捕获机制

除了condition_variable，还有哪些场景会用到lambda表达式？

如何在lambda表达式中修改按值捕获的变量？

我们来详细拆解 Lambda 表达式的**捕获机制**。

Lambda 表达式的捕获机制是其最强大也最复杂的特性之一。它允许 Lambda 函数访问其定义所在作用域中的变量，这使得 Lambda 能够像一个 “闭包（Closure）” 一样工作。

### 什么是 “捕获”？

当你在一个函数内部定义一个 Lambda 表达式时，这个 Lambda 可以 “捕获” 该函数作用域中的变量，以便在其函数体内部使用。捕获发生在 Lambda 表达式**定义**的时候，而不是在**调用**的时候。

捕获的本质是：Lambda 表达式对象在创建时，会将捕获的变量作为其内部状态的一部分。

### 捕获子句 (Capture Clause)

捕获机制是通过 Lambda 语法最前面的 **捕获子句**（即 `[]` 中的内容）来控制的。捕获子句决定了哪些变量被捕获，以及是以何种方式被捕获的。

下面是各种捕获方式的详细介绍：

#### 1. 值捕获 (Value Capture)

**语法**: `[var1, var2, ...]`

- **行为**：Lambda 会创建被捕获变量的**副本**。Lambda 内部访问的是这个副本，而不是原始变量。
- **生命周期**：副本的生命周期与 Lambda 对象的生命周期相同。
- **可修改性**：默认情况下，值捕获的变量在 Lambda 内部是 `const` 的，不能被修改。如果需要修改，必须使用 `mutable` 关键字。

```cpp
#include <iostream>

int main() {
    int x = 10;
    int y = 20;

    // 值捕获 x 和 y
    auto lambda = [x, y]() {
        // x = 100; // 错误：默认情况下，值捕获的变量是 const 的
        std::cout << "Inside lambda: x = " << x << ", y = " << y << std::endl;
    };

    x = 100; // 修改原始变量
    y = 200;

    lambda(); // 输出: Inside lambda: x = 10, y = 20
              // Lambda 捕获的是定义时的副本

    // 使用 mutable 关键字
    auto mutable_lambda = [x]() mutable {
        x = 1000; // 允许修改副本
        std::cout << "Inside mutable lambda: x = " << x << std::endl;
    };

    mutable_lambda(); // 输出: Inside mutable lambda: x = 1000
    std::cout << "Outside lambda: x = " << x << std::endl; // 输出: Outside lambda: x = 100

    return 0;
}
```

#### 2. 引用捕获 (Reference Capture)

**语法**: `[&var1, &var2, ...]`

- **行为**：Lambda 会捕获指向被捕获变量的**引用**。Lambda 内部访问和修改的都是原始变量。
- **生命周期**：Lambda 内部的引用必须在 Lambda 调用时仍然有效。如果原始变量的生命周期结束，Lambda 再去访问它会导致**未定义行为**（悬空引用）。
- **可修改性**：可以直接修改原始变量，无需 `mutable` 关键字。

**示例：**

```cpp
#include <iostream>

int main() {
    int x = 10;
    int y = 20;

    // 引用捕获 x 和 y
    auto lambda = [&x, &y]() {
        x = 100; // 修改原始变量
        y = 200;
        std::cout << "Inside lambda: x = " << x << ", y = " << y << std::endl;
    };

    lambda(); // 输出: Inside lambda: x = 100, y = 200

    std::cout << "Outside lambda: x = " << x << ", y = " << y << std::endl; // 输出: Outside lambda: x = 100, y = 200
                                                                           // 原始变量已被修改

    return 0;
}
```

**风险警示**：引用捕获非常强大，但也很危险。务必确保在 Lambda 可能被调用的整个期间，被引用的变量都还存在。

你的问题非常好，这涉及到 `std::call_once` 的工作机制和 Lambda 表达式捕获规则的结合。

我们来详细分析一下为什么 `std::call_once(flag, []() { instance = new Log(); });` 这种写法，如果 `instance` 是一个静态变量，Lambda 表达式**不需要捕获**它，并且**不能捕获**它。

### 核心结论

1. **静态变量不需要捕获**：静态变量（无论是全局静态还是局部静态）不属于任何函数的局部作用域。它们具有程序生命周期，在 Lambda 表达式内部可以直接访问，无需通过捕获子句。
2. **Lambda 不能捕获静态变量**：Lambda 表达式的捕获机制是为了捕获**局部作用域**中的变量（即函数内的非静态变量）。静态变量不在捕获机制的范围内，你不能也不需要将它们放在捕获列表 `[]` 中。

---

### 详细解释

#### 1. 静态变量的特性

当你在函数内部定义一个静态变量时，比如：

```cpp
void getInstance() {
    static Log* instance = nullptr;
    // ...
}
```

这个 `instance` 变量具有以下特性：

- **存储期**：静态存储期。它从程序开始执行时就被分配内存，直到程序结束时才被销毁。
- **作用域**：局部作用域。它只能在 `getInstance` 函数内部被直接访问。

虽然它的作用域是局部的，但它的**生命周期是全局的**。这意味着，即使 `getInstance` 函数执行完毕，`instance` 变量的值也会被保留下来，等待下一次调用。

#### 2. Lambda 表达式的捕获规则

Lambda 表达式的捕获子句 `[]` 用于捕获**定义它时所在的局部作用域**中的变量。

- 它可以捕获局部的**非静态**变量（通过值或引用）。
- 它**不能**捕获局部的**静态**变量，因为静态变量不属于局部作用域的 “自动存储期” 变量。
- 它也**不能**捕获全局变量或命名空间级别的变量，因为它们同样不属于局部作用域。

简单来说，捕获机制是为了让 Lambda 能够 “带走” 它所在函数的一些临时状态。而静态变量和全局变量是共享的、持久的状态，不需要被 “带走”。

#### 3. `std::call_once` 的使用场景

`std::call_once` 通常用于实现**线程安全的单例模式**。完整的代码通常是这样的：

```cpp
#include <mutex>

class Log {
    // ...
};

Log* getInstance() {
    // 1. 局部静态变量，用于存储唯一实例
    static Log* instance = nullptr;

    // 2. once_flag，用于确保初始化代码只执行一次
    static std::once_flag flag;

    // 3. 调用 std::call_once
    std::call_once(flag, []() {
        // Lambda 表达式的函数体
        instance = new Log(); // 这里为什么不需要捕获 instance？
    });

    return instance;
}
```

在这个例子中：

- `instance` 和 `flag` 都是 `getInstance` 函数内部的**局部静态变量**。
- Lambda 表达式 `[]() { instance = new Log(); }` 是在 `getInstance` 函数内部定义的。

**为什么不需要捕获 `instance`？**

因为当 Lambda 表达式被执行时（也就是 `std::call_once` 确保的唯一一次执行），它仍然在 `getInstance` 函数的调用栈上下文中。虽然 Lambda 表达式本身没有捕获 `instance`，但在它的函数体内部，编译器知道 `instance` 是当前作用域（`getInstance` 函数）内的一个局部静态变量。因此，它可以直接访问和修改 `instance`。

Lambda 表达式在这里只是一个 “代码块”，被 `std::call_once` 在合适的时机调用。这个代码块执行时，拥有访问 `getInstance` 函数局部静态变量的权限，就像你在 `getInstance` 函数里写了一个普通的 `if` 语句块一样。

### 总结

在你提供的代码 `std::call_once(flag, []() { instance = new Log(); });` 中：

- `instance` 几乎可以肯定是一个**局部静态变量**（这是单例模式的标准写法）。
- 因为它是静态的，所以 Lambda 表达式**不需要**也**不能**在捕获列表 `[]` 中捕获它。
- Lambda 函数体内部可以**直接访问** `instance`，因为当 `std::call_once` 执行这个 Lambda 时，它仍然在定义 `instance` 的那个函数（例如 `getInstance`）的作用域内。

简而言之：**静态变量不属于 Lambda 捕获的范畴，它们在作用域内可被直接访问。**

我们来详细探讨一下 C++ 中的队列（Queue）。

队列（Queue）是一种 ** 先进先出（First-In-First-Out, FIFO）** 的线性数据结构，它模拟了现实生活中 “排队” 的场景：最早进入队列的元素，会最早被取出。

### 一、队列的核心定义与特性

- **核心原则**：FIFO（先进先出），即 “入队顺序 = 出队顺序”。
- **关键操作**：
  - 入队（enqueue）：将元素添加到队列的**队尾**（rear）；
  - 出队（dequeue）：从队列的**队首**（front）移除元素；
  - 访问队首 / 队尾：仅能查看队首（即将出队）或队尾（最新入队）的元素，无法随机访问中间元素。
- **结构类比**：类似电影院排队检票，先到的人先检票（出队），后到的人排到队尾（入队）。

### 二、队列的核心作用

队列的本质是 **“有序管理数据的流动”**，解决 “按顺序处理数据” 的核心问题，具体作用体现在以下场景：

#### 1. 任务调度：按顺序执行任务

当多个任务需要 “先来后到” 执行时，队列可作为任务缓冲区：

- 示例：操作系统的进程调度（分时系统中，进程按到达顺序排队，CPU 轮流执行）、线程池的任务队列（任务提交后按顺序被线程处理）、打印机打印队列（多个文档按提交顺序打印）。
- 优势：保证任务执行的公平性（按提交顺序），避免任务抢占导致的混乱。

#### 2. 消息缓冲：解耦生产者与消费者

在 “生产者 - 消费者” 模型中，队列作为中间缓冲区，隔离生产和消费的速度差异：

- 示例：网络通信中的消息队列（服务器接收客户端请求后，将请求存入队列，业务线程从队列中取出请求处理，避免服务器因处理速度慢而丢失请求）、UI 事件队列（用户点击、键盘输入等事件按触发顺序存入队列，UI 线程依次处理，保证界面响应的顺序性）。
- 优势：解耦生产者（如客户端、用户操作）和消费者（如业务线程、UI 线程），避免因一方速度慢导致另一方阻塞。

#### 3. 广度优先搜索（BFS）：按层次遍历

BFS 是图论 / 树论中的经典算法，其核心是 “按层次访问节点”，队列恰好满足这一需求：

- 原理：将起始节点入队，然后循环执行 “出队一个节点 → 访问该节点 → 将其相邻节点入队”，确保先访问当前层次的节点，再访问下一层节点。
- 示例：二叉树的层序遍历（按从上到下、从左到右的顺序访问节点）、寻找图中两点的最短路径（无权图中，BFS 找到的路径是最短路径）。

#### 4. 模拟现实排队场景

队列的结构与现实中的 “排队” 完全契合，可直接用于模拟此类场景：

- 示例：银行排队叫号系统（顾客按到达顺序取号排队，窗口按顺序叫号）、售票系统（乘客按购票顺序排队，依次出票）。

#### 5. 数据限流与缓冲

当数据产生速度超过处理速度时，队列可作为 “缓冲池”，临时存储过量数据，避免数据丢失：

- 示例：日志收集系统（应用程序快速产生大量日志，日志队列缓冲这些日志，后台线程缓慢写入磁盘，防止日志丢失）、秒杀系统（大量用户同时下单，队列缓冲订单请求，后端系统按顺序处理，避免系统崩溃）。

### 三、C++ 中的队列实现

C++ 标准库提供了两种核心队列相关容器：

1. `std::queue`：普通队列（FIFO）；
2. `std::priority_queue`：优先队列（非 FIFO，按优先级出队）。

#### 1. `std::queue`（普通队列）

`std::queue` 是一种**容器适配器**（底层依赖其他容器实现），默认底层容器为 `std::deque`（双端队列），也可指定为 `std::list`（链表），但不能使用 `std::vector`（`vector` 不支持高效的队首删除操作）。

##### 核心操作（常用接口）

| 操作                 | 功能说明                  |
| ------------------ | --------------------- |
| `push(const T& x)` | 将元素 `x` 入队（添加到队尾）     |
| `pop()`            | 队首元素出队（无返回值，仅删除）      |
| `front()`          | 返回队首元素的引用（不删除，需先判断非空） |
| `back()`           | 返回队尾元素的引用（不删除，需先判断非空） |
| `empty()`          | 判断队列是否为空（空返回 `true`）  |
| `size()`           | 返回队列中元素的个数            |

```cpp
#include <iostream>
#include <queue>
#include <string>

using namespace std;

int main() {
    queue<string> bankQueue; // 银行排队队列（存储顾客姓名）

    // 顾客入队
    bankQueue.push("张三");
    bankQueue.push("李四");
    bankQueue.push("王五");

    cout << "排队人数：" << bankQueue.size() << endl;
    cout << "当前队首：" << bankQueue.front() << endl;
    cout << "当前队尾：" << bankQueue.back() << endl;

    // 顾客依次办理业务（出队）
    cout << "\n办理业务顺序：" << endl;
    while (!bankQueue.empty()) {
        cout << "正在办理：" << bankQueue.front() << endl;
        bankQueue.pop(); // 队首顾客离开
    }

    cout << "\n排队人数：" << bankQueue.size() << endl; // 0（队列空）
    return 0;
}
```

好的，我们来深入探讨 C++ 中的 `std::condition_variable`。

`std::condition_variable` 是 C++11 引入的同步原语，用于线程间通信：**让一个线程等待某个条件成立，直到其他线程通知它条件已满足**。它常与 `std::mutex` 配合使用，避免线程 “忙等”（反复检查条件浪费 CPU），是多线程同步的核心工具之一。

## 一、核心概念与设计目的

### 1. 解决的问题

多线程中，线程 A 可能需要等待线程 B 完成某个操作（比如生产数据、初始化资源）才能继续执行。如果没有 `condition_variable`，线程 A 只能循环检查条件（如 `while (flag == false)`），导致 CPU 空转（忙等）。

`condition_variable` 的作用是：让线程 A 在条件不满足时**阻塞挂起**（释放 CPU），直到线程 B 通知条件满足后再**唤醒**线程 A，从而提高效率。

### 2. 核心特性

- 依赖 `std::mutex`：条件判断和等待 / 通知的过程必须受互斥锁保护，确保原子性。
- 支持两种等待方式：无条件等待（`wait()`）和带超时的等待（`wait_for()`/`wait_until()`）。
- 支持两种通知方式：唤醒一个等待线程（`notify_one()`）或唤醒所有等待线程（`notify_all()`）。
- 可能产生 “虚假唤醒”（spurious wakeup）：线程可能在没有被通知的情况下被唤醒，因此必须用循环检查条件。

## 二、核心成员函数

`std::condition_variable` 定义在 `<condition_variable>` 头文件中，核心成员函数如下：

| 函数                                     | 功能说明                                                                        |
| -------------------------------------- | --------------------------------------------------------------------------- |
| `wait(unique_lock&)`                   | 阻塞当前线程，释放 `unique_lock` 持有的锁，直到被 `notify_one()`/`notify_all()` 唤醒。唤醒后重新获取锁。 |
| `wait(unique_lock&, Pred)`             | 带条件的等待：先检查条件 `Pred`，若为 `false` 则阻塞；被唤醒后再次检查 `Pred`，直到 `Pred` 为 `true`。      |
| `wait_for(unique_lock&, duration)`     | 带超时的等待：阻塞直到被唤醒或超时（返回 `std::cv_status::timeout`）。                            |
| `wait_until(unique_lock&, time_point)` | 带截止时间的等待：阻塞直到被唤醒或到达指定时间点。                                                   |
| `notify_one()`                         | 唤醒**一个**正在等待该条件变量的线程（若有多个等待线程，不确定唤醒哪个）。                                     |
| `notify_all()`                         | 唤醒**所有**正在等待该条件变量的线程。                                                       |

## 三、使用规则与流程

`std::condition_variable` 的使用必须遵循严格的流程，核心是 “**锁保护条件，循环检查条件**”：

### 1. 必备前提

- 必须搭配 `std::unique_lock<std::mutex>`（不能用 `std::lock_guard`，因为 `wait()` 需要临时释放锁）。
- 共享变量（条件的载体，如 `flag`、`queue.size()`）必须在互斥锁保护下读写。

`std::condition_variable` 的底层原理核心是**封装操作系统的同步原语**，结合互斥锁实现线程的 “条件等待 - 通知” 机制，避免 CPU 忙等。由于 C++ 标准仅定义接口，具体实现依赖操作系统（如 Linux、Windows）的底层同步 API，以下从通用逻辑和主流 OS 实现两方面拆解底层原理。

## 一、核心设计逻辑

`std::condition_variable` 的本质是**线程等待队列 + 操作系统阻塞 / 唤醒机制**，其核心目标是：

1. 让等待线程在条件不满足时**释放 CPU 并阻塞**（而非循环检查）；
2. 让通知线程在条件满足时**精准唤醒等待线程**；
3. 确保 “释放锁” 和 “阻塞线程” 的原子性（避免唤醒丢失）。

为实现这一点，它必须与 `std::mutex` 配合，因为：

- 条件的检查和修改需要**互斥保护**（避免多线程并发修改条件导致数据竞争）；
- 阻塞前必须**释放锁**（否则其他线程无法修改条件，导致死锁）；
- 唤醒后必须**重新获取锁**（确保后续操作在互斥环境下执行）。

## 二、底层依赖的操作系统原语

C++ 标准库的 `std::condition_variable` 是对操作系统底层同步原语的封装，不同 OS 的实现差异较大，但核心逻辑一致：

| 操作系统    | 底层同步原语                       | 关键 API                                                                                                              |
| ------- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| Linux   | POSIX 条件变量（`pthread_cond_t`） | `pthread_cond_init()`、`pthread_cond_wait()`、`pthread_cond_signal()`、`pthread_cond_broadcast()`                      |
| Windows | 条件变量（`CONDITION_VARIABLE`）   | `InitializeConditionVariable()`、`SleepConditionVariableCS()`、`WakeConditionVariable()`、`WakeAllConditionVariable()` |

注：早期 Linux 系统的 `pthread_cond_t` 可能存在性能问题（如使用信号量实现），但现代 glibc 已优化为更高效的内核级实现。

## 三、核心操作的底层流程拆解

以 Linux 平台的 `pthread_cond_t` 实现为例，详细拆解 `wait()`、`notify_one()`、`notify_all()` 的底层执行步骤：

### 1. `wait(unique_lock& lock, Pred pred)` 的底层流程

`wait()` 是条件变量最核心的操作，其底层逻辑可概括为：**“检查条件→不满足则释放锁并阻塞→被唤醒后重新获取锁并再次检查条件”**。

具体步骤：

1. **检查条件（`pred()`）**：
   
   - 此时 `unique_lock` 已持有互斥锁（`mtx` 已上锁），线程安全地检查条件（如 `data_queue.empty()`）。
   - 若 `pred()` 为 `true`，直接返回（无需阻塞）。

2. **释放锁并阻塞（核心原子操作）**：
   
   - 若 `pred()` 为 `false`，调用底层 `pthread_cond_wait(&cond, &mtx)`：
     - **原子操作**：先释放 `mtx`（让其他线程可修改条件），再将当前线程加入条件变量的**等待队列**，最后让线程进入**阻塞状态**（释放 CPU 资源，由 OS 调度器管理）。
     - 为何必须原子？若 “释放锁” 和 “阻塞” 非原子，可能出现：线程释放锁后、阻塞前，其他线程已修改条件并发出通知，但此时当前线程尚未进入等待队列，导致**唤醒丢失**。

3. **被唤醒后重新获取锁**：
   
   - 当其他线程调用 `notify_one()`/`notify_all()` 时，OS 会从等待队列中选择一个 / 所有线程，将其状态从 “阻塞” 改为 “就绪”。
   - 就绪线程会尝试重新获取 `mtx`（此时 `unique_lock` 会调用 `mtx.lock()`）：
     - 若锁未被占用，直接获取成功；
     - 若锁已被其他线程占用（如多个线程被 `notify_all()` 唤醒），则线程会阻塞在**锁的等待队列**中，直到获取锁。

4. **再次检查条件（避免虚假唤醒）**：
   
   - 重新获取锁后，再次调用 `pred()` 检查条件：
     - 若 `pred()` 为 `true`，返回；
     - 若为 `false`（可能是虚假唤醒，或其他线程已修改条件），重复步骤 2（再次释放锁并阻塞）。
   
   这就是为什么必须用循环检查条件（`cv.wait(lock, pred)` 内部已封装循环）。

### 2. `notify_one()` 的底层流程

`notify_one()` 的作用是**唤醒等待队列中的一个线程**，底层步骤：

1. 调用底层 `pthread_cond_signal(&cond)`；
2. OS 从条件变量的等待队列中，选择一个线程（通常是队列头部，即 “先进先出”）；
3. 将该线程的状态从 “阻塞” 改为 “就绪”，并将其从等待队列中移除；
4. 就绪线程会尝试重新获取互斥锁（如步骤 1.3 所述），获取成功后继续执行。

注：`notify_one()` 本身**不持有互斥锁**，也不修改共享资源，仅负责唤醒线程。

### 3. `notify_all()` 的底层流程

`notify_all()` 的作用是**唤醒等待队列中的所有线程**，底层步骤：

1. 调用底层 `pthread_cond_broadcast(&cond)`；
2. OS 将条件变量等待队列中的**所有线程**状态改为 “就绪”，并清空等待队列；
3. 所有就绪线程会同时尝试获取互斥锁（锁的竞争）；
4. 最终只有一个线程能成功获取锁（其他线程会阻塞在锁的等待队列中），获取锁的线程会检查条件，满足则执行，否则再次阻塞。

注：`notify_all()` 会导致 “惊群效应”（多个线程被唤醒后竞争锁），可能浪费 CPU 资源，因此仅在需要所有线程响应条件时使用。

## 四、关键细节的底层解释

### 1. 虚假唤醒（Spurious Wakeup）的底层原因

虚假唤醒是指线程在未被 `notify_*()` 通知的情况下，`wait()` 函数返回。其底层原因来自 OS 或硬件：

- **Linux**：`pthread_cond_wait()` 可能因信号（如 `SIGINT`）、内核调度策略变化或硬件中断而提前返回；
- **Windows**：`SleepConditionVariableCS()` 可能因系统事件（如线程优先级调整）而唤醒。

C++ 标准允许虚假唤醒，因此必须通过循环检查条件来应对。

### 2. 为什么必须用 `unique_lock` 而不是 `lock_guard`？

`wait()` 函数需要在阻塞前**手动释放锁**，阻塞后**重新获取锁**，而 `lock_guard` 仅支持 “构造时上锁、析构时解锁”，不支持手动 `unlock()`/`lock()`；`unique_lock` 则提供 `unlock()` 和 `lock()` 成员函数，满足 `wait()` 的需求。

底层来看，`unique_lock` 内部维护一个 “是否持有锁” 的状态：

- 调用 `wait()` 时，`unique_lock` 会先调用 `mtx.unlock()`（释放锁），并将状态设为 “未持有”；
- 被唤醒后，`unique_lock` 会调用 `mtx.lock()`（重新获取锁），并将状态设为 “持有”。

### 3. 等待队列的底层实现

条件变量的等待队列是 OS 管理的内核级队列（或用户级队列，取决于实现），用于存储阻塞的线程：

- 线程调用 `wait()` 时，会将自己的线程控制块（TCB，Thread Control Block）加入队列；
- 线程被唤醒时，会从队列中移除 TCB，并将其状态改为就绪；
- 队列通常采用 “先进先出”（FIFO）策略，确保线程等待的公平性（部分 OS 支持优先级调度，但 C++ 标准不保证）。

## 五、`std::condition_variable_any` 的底层差异

`std::condition_variable_any` 与 `std::condition_variable` 的核心区别是**支持任意 BasicLockable 锁**（如 `lock_guard`、`shared_lock`），其底层实现差异：

- `condition_variable` 针对 `unique_lock` 优化，直接调用 `unique_lock` 的 `unlock()`/`lock()` 方法；
- `condition_variable_any` 采用模板编程，通过 `lock.unlock()` 和 `lock.lock()` 通用接口操作锁，因此支持任意满足 `BasicLockable` 要求的锁（只要提供 `lock()` 和 `unlock()` 方法）。

代价：`condition_variable_any` 的灵活性提升，但模板实例化和通用接口调用会带来**轻微的性能开销**（通常可忽略，除非高频调用）。

## 六、底层性能考量

1. **避免忙等**：条件变量的核心优势是阻塞线程时释放 CPU，避免 “循环检查条件” 导致的 CPU 空转；
2. **唤醒效率**：`notify_one()` 仅唤醒一个线程，效率高于 `notify_all()`（避免惊群效应）；
3. **锁竞争**：唤醒后线程需竞争互斥锁，若锁竞争激烈，可能导致线程阻塞在锁上，影响性能（需优化锁的粒度）；
4. **内核态切换**：`wait()` 和 `notify_*()` 会涉及内核态切换（线程阻塞 / 唤醒需 OS 调度），频繁调用可能产生性能开销（需合理设计条件通知的频率）。

## 七、总结

`std::condition_variable` 的底层原理可概括为：

- **封装 OS 同步原语**：Linux 用 `pthread_cond_t`，Windows 用 `CONDITION_VARIABLE`；
- **核心流程**：等待线程 “检查条件→释放锁→阻塞→唤醒→重新获取锁→再次检查条件”，通知线程 “唤醒等待队列中的线程”；
- **关键保障**：`wait()` 中 “释放锁” 和 “阻塞” 的原子性，避免唤醒丢失；循环检查条件，应对虚假唤醒；
- **锁配合**：依赖 `unique_lock` 的手动 `unlock()`/`lock()` 能力，实现锁与条件变量的协同。

理解底层原理有助于更好地使用条件变量，避免死锁、唤醒丢失等问题，尤其在高并发场景下优化性能。

编辑分享

虚假唤醒的处理方法有哪些？

如何避免虚假唤醒？

condition_variable_any和condition_variable的性能测试

### 完整流程（生产者 - 消费者示例）

```cpp
std::queue<int> _task;
std::condition_variable _cv;
std::mutex mtx;

void Producer()
{
    for (int i = 0; i < 10; i++)
    {
        std::unique_lock<std::mutex> lock(mtx);
        _task.push(i);
        _cv.notify_one();
        std::cout << __TIME__ << " Producer: " << i << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

bool GetCondition()
{
    static int _tmp = 0;
    _tmp++;
    std::cout << "_tmp: " << _tmp << " !_task.empty(): " << !_task.empty() <<std::endl;
    return !_task.empty();
}


void Consumer()
{
    while (1)
    {
        std::unique_lock<std::mutex> lock(mtx);
        //_cv.wait(lock, []() {return !_task.empty(); });
        _cv.wait(lock, GetCondition);
        int tmp = _task.front();
        _task.pop();
        std::cout << __TIME__ << " Consumer: " << tmp << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}




int main()
{
    std::thread t1(Producer);
    std::thread t2(Consumer);

    t1.join();
    t2.join();
    return 0;
}
```
