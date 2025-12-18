# call_once

在 C++ 中，**内置类型隐式转换为自定义类型**时，调用的是自定义类型的**转换构造函数**（Conversion Constructor）。

### 一、转换构造函数的定义

转换构造函数是一种特殊的构造函数，满足以下条件：

1. 它是一个**单参数构造函数**（参数可以是内置类型，也可以是其他自定义类型）。
2. 这个参数**不是**该类的**拷贝构造函数**或**移动构造函数**的参数（即参数类型不是 `const ClassName&` 或 `ClassName&&`）。

```cpp
class MyClass {
public:
    // 转换构造函数：允许从 int 隐式转换到 MyClass
    MyClass(int value) {
        // ...
    }

    // 拷贝构造函数：不是转换构造函数
    MyClass(const MyClass& other) {
        // ...
    }
};
```

### 二、隐式转换的触发场景

当编译器需要将一个内置类型的值赋给自定义类型对象，或者传递给期望自定义类型参数的函数时，会自动调用转换构造函数完成隐式转换。

```cpp
void func(MyClass obj) {
    // ...
}

int main() {
    // 场景 1：赋值时的隐式转换
    MyClass obj = 123; // 等价于 MyClass obj = MyClass(123);

    // 场景 2：函数参数传递时的隐式转换
    func(456); // 等价于 func(MyClass(456));

    return 0;
}
```

在上面的代码中，`123` 和 `456` 是内置类型 `int`，编译器会自动调用 `MyClass(int)` 将它们转换为 `MyClass` 对象。

### 三、禁止隐式转换：`explicit` 关键字

如果你不希望编译器进行这种隐式转换，可以在转换构造函数前加上 `explicit` 关键字。

```cpp
class MyClass {
public:
    // 禁止隐式转换
    explicit MyClass(int value) {
        // ...
    }
};

int main() {
    // 编译错误：不能隐式转换
    MyClass obj = 123; 

    // 编译错误：不能隐式转换
    func(456);         

    // 正确：显式转换
    MyClass obj2(123);      // 直接初始化
    MyClass obj3 = MyClass(123); // 显式创建临时对象
    func(MyClass(456));     // 显式传递临时对象
    func(static_cast<MyClass>(789)); // 显式类型转换

    return 0;
}
```

`explicit` 可以防止意外的隐式转换，让代码的意图更加清晰，是 C++11 及以后推荐的最佳实践之一（除非你明确需要隐式转换）。

### 总结

- **转换构造函数**用于将其他类型（通常是内置类型）隐式转换为当前自定义类型。
- 它是一个**单参数**的构造函数。
- 使用 `explicit` 关键字可以**禁止**这种隐式转换，只能通过显式方式进行转换。

在 C++11 中，`<mutex>` 头文件提供了 `std::call_once` 和 `std::once_flag`，用于**多线程环境下保证某个函数只被执行一次**。

---

## 1. 核心概念

**`std::call_once`**

```cpp
template<class Callable, class... Args>
void call_once(std::once_flag& flag, Callable&& f, Args&&... args);
```

- 作用：
  
  保证 `f(args...)` 只被执行一次，即使被多个线程同时调用。

**`std::once_flag`**

- 用于标记是否已经执行过函数。
- 必须是**全局变量**或**静态变量**（生命周期足够长）。
- 不可复制、不可移动，只能默认构造。

---

## 2. 使用示例

```cpp
#include <iostream>
#include <thread>
#include <mutex>

std::once_flag flag;

void init() {
    std::cout << "Initializing..." << std::endl;
    // 模拟初始化操作
}

void worker() {
    std::call_once(flag, init); // 多个线程调用，但init只执行一次
    std::cout << "Worker running..." << std::endl;
}

int main() {
    std::thread t1(worker);
    std::thread t2(worker);
    std::thread t3(worker);

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
```

## 3. 特点

1. **线程安全**：
   
   - 即使多个线程同时调用 `std::call_once`，也只有一个线程会执行 `init()`。
   - 其他线程会阻塞，直到 `init()` 执行完成。

2. **异常处理**：
   
   - 如果 `init()` 抛出异常，`std::call_once` 会认为该函数未执行成功。
   - 其他线程会继续尝试调用 `init()`，直到成功为止。

```cpp
void init() {
    static int count = 0;
    count++;
    if (count == 1) {
        throw std::runtime_error("First call failed");
    }
    std::cout << "Initializing successfully..." << std::endl;
}
```

1. 第一次调用失败，第二次会成功。

2. **性能**：
   
   - 比使用 `std::mutex` 手动实现更高效。
   - 内部使用原子操作，避免了不必要的锁竞争。

---

## 4. 常见用途

- **单例模式**：
  
  懒汉式单例的线程安全初始化。

```cpp
class Singleton {
public:
    static Singleton& getInstance() {
        std::call_once(flag, []() {
            instance.reset(new Singleton());
        });
        return *instance;
    }

private:
    Singleton() {}
    static std::unique_ptr<Singleton> instance;
    static std::once_flag flag;
};

std::unique_ptr<Singleton> Singleton::instance;
std::once_flag Singleton::flag;
```

- **资源初始化**：
  
  如加载配置文件、初始化数据库连接等。

- **避免重复初始化**：
  
  确保某个操作只执行一次，无论多少线程调用。

---

## 5. 注意事项

1. **`std::once_flag` 的生命周期**：
   
   - 必须保证 `flag` 的生命周期长于所有调用 `std::call_once` 的线程。
   - 通常定义为全局变量或类的静态成员。

2. **不可复制**：
   
   - `std::once_flag` 没有拷贝构造函数，不能作为函数参数传递（除非通过引用）。

3. **与 `std::mutex` 的区别**：
   
   - `std::mutex` 用于保护一段代码不被多个线程同时执行。
   - `std::call_once` 专注于保证函数只执行一次，更适合初始化场景。

---

## 6. C++11 之前的实现方式

在 C++11 之前，可以用 `pthread_once`（POSIX）或 `InitializeCriticalSection`（Windows）实现类似功能，但 `std::call_once` 是跨平台的标准解决方案。

---

**总结**：

`std::call_once` 是多线程环境下保证函数只执行一次的高效、线程安全的工具，常用于初始化和单例模式等场景。

```cpp
class Log
{
public:
    Log(const Log& log) = delete;
    Log& operator=(const Log& log) = delete;
    static Log& GetInstance()
    {
        static Log* log = nullptr;
        //if (!log)
        //{
        //    log = new Log;
        //}
        std::call_once(flag, initInstance, log);
        return *log;
    }

    void PrintLog(std::string msg)
    {
        std::cout << __TIME__ << ' ' << msg << std::endl;
    }

private:
    Log() {}
    static void initInstance(Log* log)
    {
        log = new Log;
        std::cout << "init" << std::endl;
    }
    static std::once_flag flag;
};

std::once_flag Log::flag;

void func()
{
    for (int i = 0; i < 3; i++)
    {
        Log::GetInstance().PrintLog("maple");
    }

}


int main()
{

    std::thread t1(func);
    std::thread t2(func);

    t1.join();
    t2.join();
    return 0;
}
```

```cpp
class Log
{
public:
    Log(const Log& log) = delete;
    Log& operator=(const Log& log) = delete;
    static Log& GetInstance()
    {
        std::call_once(flag, []() {log = new Log; std::cout << "init" << std::endl;});
        return *log;
    }
    void PrintLog(std::string msg)
    {
        std::cout << __TIME__ << ' ' << msg << std::endl;
    }
private:
    Log() {}
    static std::once_flag flag;
    static Log* log;
};
Log* Log::log = nullptr;
std::once_flag Log::flag;



class Log
{
public:
    Log(const Log& log) = delete;
    Log& operator=(const Log& log) = delete;
    static Log& GetInstance()
    {
        static Log* log = nullptr;
        std::call_once(flag, []() {log = new Log; std::cout << "init" << std::endl;});
        return *log;
    }
    void PrintLog(std::string msg)
    {
        std::cout << __TIME__ << ' ' << msg << std::endl;
    }
private:
    Log() {}
    static std::once_flag flag;
    //static Log* log;
};
//Log* Log::log = nullptr;
std::once_flag Log::flag;

void func()
{
    for (int i = 0; i < 3; i++)
    {
        Log::GetInstance().PrintLog("maple");
    }

}


int main()
{

    std::thread t1(func);
    std::thread t2(func);

    t1.join();
    t2.join();
    return 0;
}
```
