# Point-LIO

`ros::Time().fromSec()` 是 ROS (Robot Operating System) 中的一个函数，​**​用于将一个以秒（double 类型）表示的时间戳转换为 ROS 的时间对象（`ros::Time`）​**​

将 ​**​浮点数秒​**​（例如 `1650000000.123`）转换为等价的 `ros::Time` 对象

`ros::Time` 对象包含两个整数成员：`sec` (秒) 和 `nsec` (纳秒)

## 比较ros::Time的用法

> `ros::Time::fromSec(double t)`   将浮点数秒 → `ros::Time` 对象
> `ros::Time::toSec()`   将 `ros::Time` 对象 → 浮点数秒
> 
> `ros::Time::now()`   获取当前时间戳的 `ros::Time` 对象

```cpp
std::vector<double> extrinT(3, 0.0);
```

> **构造函数参数​**​
> 
> - `(3, 0.0)`：调用 vector 的 ​**​fill constructor​**​（填充构造函数）
> - ​**​第一个参数 `3`​**​：指定向量初始包含的元素数量
> - ​**​第二个参数 `0.0`​**​：指定每个元素的初始值

```cpp
const M3D Eye3d(M3D::Identity());
```

Eigen 库创建并初始化了一个​**​3x3单位矩阵**

```cpp
MTK_BUILD_MANIFOLD(state_input,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((vect3, gravity))
);
```

**​​C++ 中的宏定义（Macro）:  定义一个结构化状态变量（state_input），将多个组件封装在一个流形（Manifold）空间中**

```cpp
MTK_BUILD_MANIFOLD( // 宏名称
    state_input,     // 生成的结构体名称
    ((vect3, pos))   // 成员1：类型+名称
    ((SO3, rot))     // 成员2：类型+名称
    ...              // 其他成员
);
```
