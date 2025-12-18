# worker

类中使用std::vector<std::thread> _workers;

#### 如果是非静态成员函数：

```cpp
_workers.emplace_back(&lidar::worker_task1, this, _nh);
```

这里 `this` 是必需的，它会作为成员函数的第一个参数传递。

### 同理

```cpp
ros::Subscriber tf_sub = nh.subscribe("/tf", 10, &lidar::tfCallback, this);
```

这个错误的核心原因是：**不能直接将非静态成员函数（non-static member function）作为线程函数传递**，因为非静态成员函数需要依赖类的实例（`this`指针）才能调用，而线程创建时默认无法关联实例。

### 具体分析

你的代码中尝试在`lidar`类的构造函数里创建线程，传入了非静态成员函数`worker_task1`：

cpp

运行

```cpp
_workers.emplace_back(lidar::worker_task1, this, _nh);
```

非静态成员函数`worker_task1`的完整调用需要绑定到一个`lidar`类的实例（即`this`指针），但直接传递函数名时，编译器无法自动关联实例，因此报错。

### 解决方法

需要通过`std::bind`将非静态成员函数与当前实例（`this`）绑定，再传递给线程。具体步骤如下：

- **包含必要的头文件**

- 在代码中添加`#include <functional>`，用于`std::bind`的使用。

- **修改线程创建代码**

使用`std::bind`绑定成员函数与当前实例，将线程创建代码修改为：

cpp

运行

1. ```cpp
   #include <functional>  // 记得添加这个头文件
   
   // 在构造函数中
   _workers.emplace_back(std::bind(&lidar::worker_task1, this, _nh));
   ```
   
   - `&lidar::worker_task1`：获取成员函数的地址。
   - `this`：指定当前`lidar`实例（绑定实例）。
   - `_nh`：传递给`worker_task1`的参数（与函数定义中的`ros::NodeHandle nh`匹配）。

### 补充说明

如果`worker_task1`需要修改类的非静态成员变量，这种绑定方式完全可行（因为`this`指针已经关联了当前实例）。

如果使用 C++11 及以上标准，`std::bind`是处理成员函数线程绑定的标准方式，兼容性更好。

修改后重新编译，该错误即可解决。
