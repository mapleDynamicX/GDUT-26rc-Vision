# Matrix(矩阵)

## 基本旋转矩阵 (绕坐标轴)

​**​1.绕 X 轴旋转角度 θ：​**

![](/home/maple/笔记/images/2025-07-10-16-24-26-RX.png)

**2.绕 Y 轴旋转角度 θ**

<img title="" src="file:///home/maple/笔记/images/2025-07-10-16-25-00-RY.png" alt="" width="327">

​**3.​绕 Z 轴旋转角度 θ：​**

<img src="file:///home/maple/笔记/images/2025-07-10-16-25-47-RZ.png" title="" alt="" width="324">

## ​齐次变换矩阵结构

![](/home/maple/笔记/images/2025-07-10-16-51-30-T.png)

若需将点从坐标系 A 到坐标系 C：

```
P_c = T_{c←a} * P_a
```

> - ​**​链式变换推导​**​：
>   
>   - 已知：
>     - `T_{b←a}`：从 A 到 B 的变换
>     - `T_{c←b}`：从 B 到 C 的变换
>   - 则 ​**​T_{c←a} = T_{c←b} * T_{b←a}​**​

```cpp
#include <Eigen/Geometry>
// 定义变换矩阵 (A→B 和 B→C)
Eigen::Affine3d T_b_a; // A→B 变换 (注意变量命名方向)
Eigen::Affine3d T_c_b; // B→C 变换
// 设置变换参数 (示例)
T_b_a = Eigen::Translation3d(2, 0, 0) * Eigen::Matrix3d::Identity();
T_c_b = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
// 合成总变换 A→C
Eigen::Affine3d T_c_a = T_c_b * T_b_a;
// 定义点 (A坐标系)
Eigen::Vector3d P_a(1, 0, 0);
// 转换为齐次坐标
Eigen::Vector4d P_a_homog = (Eigen::Vector4d() << P_a, 1).finished();
// 坐标转换
Eigen::Vector4d P_c_homog = T_c_a.matrix() * P_a_homog;
Eigen::Vector3d P_c = P_c_homog.head<3>();
std::cout << "C系坐标: " << P_c.transpose(); // 输出: 0 3 0
```

## **旋转顺序​**​：通常按Z-Y-X顺序（内旋）或X-Y-Z顺序（外旋）组合，PCL默认使用​**​Z-Y-X顺序​**​

​**​单位​**​：角度需转换为​**​弧度制​**​（Eigen库要求）

### **示例：**

```cpp
include <pcl/common/transforms.h>
#include <Eigen/Geometry>
Eigen::Matrix3f createRotationMatrix(float rx, float ry, float rz) {
    // 转换为弧度
    rx = rx * M_PI / 180.0f; // Roll (绕X轴)
    ry = ry * M_PI / 180.0f; // Pitch (绕Y轴)
    rz = rz * M_PI / 180.0f; // Yaw (绕Z轴)
    // 创建绕各轴的旋转矩阵
    Eigen::Matrix3f R_x;
    R_x << 1, 0, 0,
           0, cos(rx), -sin(rx),
           0, sin(rx), cos(rx);
    Eigen::Matrix3f R_y;
    R_y << cos(ry), 0, sin(ry),
           0, 1, 0,
           -sin(ry), 0, cos(ry);
    Eigen::Matrix3f R_z;
    R_z << cos(rz), -sin(rz), 0,
           sin(rz), cos(rz), 0,
           0, 0, 1;
    // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
    return R_z * R_y * R_x;
}
方法2：使用Eigen的AngleAxis（更简洁）
Eigen::Matrix3f createRotationMatrix(float rx, float ry, float rz) {
    Eigen::AngleAxisf roll(rx * M_PI/180, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch(ry * M_PI/180, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw(rz * M_PI/180, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = yaw * pitch * roll; // Z-Y-X顺序
    return q.toRotationMatrix();
}
```

### 创建平移向量

```cpp
Eigen::Vector3f createTranslationVector(float tx, float ty, float tz) {
    Eigen::Vector3f translation(tx, ty, tz);
    return translation;
}
```

### 合成变化矩阵

```cpp
// 分离和组合现有旋转矩阵与平移向量
Eigen::Matrix4f combineRotationAndTranslation(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation) 
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}
```

### Eigen::Matrix3f 和 Eigen::Vector3f 详解

#### **Eigen::Matrix3f**

3x3 单精度浮点矩阵，内存布局为​**​列优先​**​（Column-major）

```
地址布局：  
0: m00  3: m01  6: m02  
1: m10  4: m11  7: m12  
2: m20  5: m21  8: m22  
```

#### **​Eigen::Vector3f​**

3维单精度浮点列向量（等价于`Matrix<float, 3, 1>`）

```
地址布局：  
0: x  
1: y  
2: z 
```

### **数据访问方法**

```cpp
// 矩阵访问 (行,列)
Eigen::Matrix3f m;
m(0,0) = 1.0f;  // 设置第0行0列元素
float val = m(2,1); // 读取第2行1列元素
// 向量访问 [索引]
Eigen::Vector3f v;
v(0) = 2.0f;    // 等效于v.x()
v[1] = 3.0f;    // 数组语法 (等效于v.y())
float z = v.z(); // 通过成员函数访问

//通过指针访问原始数据​​
float* matrix_data = m.data();
matrix_data[0] = 1.0f; // m(0,0)
// 向量数据指针（连续内存）
float* vec_data = v.data();
vec_data[2] = 5.0f; // v.z()

//初始化语法​​
// 逗号初始化（按行填充）
Eigen::Matrix3f m;
m << 1.0f, 2.0f, 3.0f,
     4.0f, 5.0f, 6.0f,
     7.0f, 8.0f, 9.0f;
// 向量初始化
Eigen::Vector3f v1(1.0f, 2.0f, 3.0f); // 构造函数
Eigen::Vector3f v2; v2 << 4.0f, 5.0f, 6.0f;  // 逗号初始化
// 特殊初始化
m = Eigen::Matrix3f::Identity(); // 单位矩阵
v = Eigen::Vector3f::Zero();     // 零向量
```

## 矩阵和向量相乘

```cpp
#include <iostream>
#include <Eigen/Dense>  // 包含 Eigen 核心功能

int main() {
    // --------------------------
    // 步骤 1：声明并初始化 Matrix2d 和 Vector2d
    // --------------------------
    Eigen::Matrix2d A;  // 2x2 矩阵（未初始化，元素随机）
    A << 1, 2,          // 第一行
         3, 4;          // 第二行

    Eigen::Vector2d v;  // 2x1 列向量（未初始化）
    v << 5, 6;          // 初始化为 [5; 6]（列向量）

    // --------------------------
    // 步骤 2：矩阵与向量相乘（A × v）
    // 结果类型为 Eigen::Vector2d（2x1 列向量）
    // --------------------------
    Eigen::Vector2d result = A * v;

    // --------------------------
    // 步骤 3：输出结果
    // --------------------------
    std::cout << "矩阵 A：" << std::endl<< A << std::endl << std::endl;
    std::cout << "向量 v：" << std::endl<< v << std::endl << std::endl;
    std::cout << "A × v 的结果：" << std::endl << result << std::endl;

    return 0;
}
```

## 矩阵求转置和求逆

```cpp
#include <iostream>
#include <Eigen/Dense>  // 包含 Eigen 核心功能

int main() {
    // --------------------------
    // 步骤 1：定义并初始化原矩阵 Matrix2d
    // --------------------------
    Eigen::Matrix2d A;
    A << 1, 2,          // 第一行 [1, 2]
         3, 4;          // 第二行 [3, 4]

    // --------------------------
    // 步骤 2：计算转置矩阵（A^T）
    // --------------------------
    Eigen::Matrix2d At = A.transpose();  // 调用 transpose() 获取转置

    // --------------------------
    // 步骤 3：输出原矩阵和转置矩阵
    // --------------------------
    std::cout << "原矩阵 A：" << std::endl;
    std::cout<< A << std::endl << std::endl;

    std::cout << "转置矩阵 A^T：" << std::endl;
    std::cout << At << std::endl;

    return 0;
}

//求t的逆矩阵
    Eigen::Matrix4f inverse_transform = T.inverse();
```

## 点云不动公式推导

![](/home/maple/笔记/images/2025-07-13-17-53-41-点云.png)

## 打印矩阵

```cpp
for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
        std::cout << matrix(i, j) << "\t";
    }
    std::cout << std::endl;
}
```

## 打印向量

```cpp
#include <iostream>
#include <Eigen/Dense>
int main() {
    Eigen::Vector3f v(0.1f, 0.2f, 0.3f);
    for (int i = 0; i < v.size(); ++i) {
        std::cout << "Element " << i << ": " << v[i] << "\n";
    }
    return 0;
}
```

## 点云从当前坐标变换成世界坐标

```cpp
#include<ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // 包含变换函数
#include <Eigen/Geometry>
#include<iostream>

Eigen::Matrix3f createRotationMatrix(float rx, float ry, float rz) {
    // 转换为弧度
    rx = rx * M_PI / 180.0f; // Roll (绕X轴)
    ry = ry * M_PI / 180.0f; // Pitch (绕Y轴)
    rz = rz * M_PI / 180.0f; // Yaw (绕Z轴)
    // 创建绕各轴的旋转矩阵
    Eigen::Matrix3f R_x;
    R_x << 1, 0, 0,
           0, cos(rx), -sin(rx),
           0, sin(rx), cos(rx);
    Eigen::Matrix3f R_y;
    R_y << cos(ry), 0, sin(ry),
           0, 1, 0,
           -sin(ry), 0, cos(ry);
    Eigen::Matrix3f R_z;
    R_z << cos(rz), -sin(rz), 0,
           sin(rz), cos(rz), 0,
           0, 0, 1;
    // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
    return R_z * R_y * R_x;
}
Eigen::Vector3f createTranslationVector(float tx, float ty, float tz) {
    Eigen::Vector3f translation(tx, ty, tz);
    return translation;
}
// 分离和组合现有旋转矩阵与平移向量
Eigen::Matrix4f combineRotationAndTranslation(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}
void test1()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data
    cloud->width  = 1;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    cloud->points[0].x = 2.0f;
    cloud->points[0].y = 0;
    cloud->points[0].z = 1;
    //旋转
    Eigen::Matrix3f rot = createRotationMatrix(0, 0, 30);
    //平移
    Eigen::Vector3f tra = createTranslationVector(1, 1, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    //先旋转后平移
    Eigen::Matrix4f T = combineRotationAndTranslation(rot, tra);
    pcl::transformPointCloud(*cloud, *output, T);
    for(int i = 0; i < output->points.size(); i++)
    {
        std::cout<<"x: "<<output->points[i].x<<" y: "<<output->points[i].y<<" z: "<<output->points[i].z<<std::endl;
    }
}
```

> 旋转
> 
> 世界坐标系向当前坐标系的旋转（逆时针为正）
> 
> 平移
> 
> 当前坐标相对于是世界坐标系的坐标

### 内参矩阵的结构（通常表示为 K）

内参矩阵是一个 ​**​3×3​**​ 的上三角矩阵，具有以下标准形式：

```
内参矩阵 K = [
  [fx,    s,  cx],
  [    0,  fy, cy],
  [    0,     0,    1.0]
]
```

#### 各参数的含义：

- ​**​焦距参数​**​：
  
  - fx​：以像素为单位表示的相机在x轴方向的等效焦距。
  
  - fy​：以像素为单位表示的相机在y轴方向的等效焦距。
    
    *（当图像传感器像素为正方形时，通常 fx​≈fy​）*

- ​**​主点坐标​**​：
  
  - cx​：相机光心（主点）在图像中的x坐标（像素单位）。
  
  - cy​：相机光心（主点）在图像中的y坐标（像素单位）。
    
    *（理想情况下位于图像中心）*

- ​**​轴倾斜因子​**​ s（Skew）：
  
  描述图像坐标轴之间的夹角倾斜程度（单位为像素）。
  
  *（现代相机多为矩形像素，通常 s≈0，可忽略）*
