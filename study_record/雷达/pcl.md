# pcl点云配准

## 工作流程

![](/home/maple/笔记/images/2025-07-08-15-04-13-pcl配准.png)

## RANSAC粗配准

```
目标​​：估算初始变换矩阵（6DOF刚体变换）
​​处理流程​​：
    ​​特征匹配​​：
        在特征空间中通过KD-Tree搜索匹配点对
        输出：一组候选匹配点对（含大量噪声）
    ​​RANSAC迭代​​：
        随机采样3组点对
        估算变换矩阵（SVD求解最小二乘解）
    ​​变换评估​​：
        应用当前变换到源关键点
        计算目标点云中对应点的距离
        统计内点数量（距离<阈值）
    ​​最优选择​​：
        保留内点最多的变换矩阵
        使用内点重新估计精确变换
```

## 应用初始变化

**目的​**​：将源点云初步对齐到目标点云附近

## ICP精配准​

```
 目标​​：优化变换矩阵达到亚毫米级精度
    迭代过程：
    ​流程控制​：最大迭代次数(100) + 变换收敛阈值(1e-6)
        ​​最近点搜索​​：
            KD-Tree查找源点在目标点云中的最近邻
            剔除异常点（距离 > 3倍点云平均密度）
        ​​误差最小化​​： R,tmin​i=1∑n​∣∣qi​−(Rpi​+t)∣∣2
            使用SVD分解求解最优R（旋转）和t（平移）
        ​​变换更新​​： Ticp​←[R0​t1​]⋅Ticp​
            累积变换矩阵
        ​​收敛判断​​：
            δ = 当前迭代的均方误差变化量
            终止条件：δ < ε 或 达到最大迭代次数

 ​结果输出​​： 最终变换​​： Tfinal​=Ticp​⋅Trough
```

## NDT点云配准

NDT是一种用于​**​对齐（配准）​**​ 两个点云（Point Cloud）的算法。它的核心思想是​**​概率模型匹配​**​，与传统ICP（Iterative Closest Point）的点对点匹配思路不同

> 1. ​**​预处理参考点云：​**​
>    
>    - ​**​设定网格大小：​**​ 这是关键参数！尺寸太小，效率低且对噪声敏感；尺寸太大，丢失细节，模型粗糙。需要根据点云密度和应用场景选择。
>    - ​**​划分网格：​**​ 根据设定尺寸，将参考点云的空间划分成规则网格。
>    - ​**​计算网格属性：​**​ 对每个包含至少一定数量（如3个）点的非空网格，计算其均值和协方差矩阵，并存储。
> 
> 2. ​**​初始化变换参数：​**​
>    
>    - 提供一个初始的猜测变换（R0, t0）。这个初始值越接近真实值，效果越好。可以是零（假设点云大致重合）、手动旋转平移、或者来自其他粗配准方法（如基于特征）的粗略估计。​**​NDT的鲁棒性意味着它可以容忍比ICP更大的初始偏差。​**​
> 
> 3. ​**​迭代优化：​**​
>    
>    - 对当前变换后的源点云每个点 `x_i_transformed`:  
>      a) 根据其位置找到对应的​**​参考网格单元​**​。  
>      b) 如果该网格是​**​非空且有效的​**​（点数量大于阈值，协方差条件良好），则计算 `p(x_i_transformed)`。
>    - 将所有点的 `p(x_i_transformed)` 累加（或平均）得到当前得分 `S`。
>    - 计算得分函数 `S` 对变换参数（旋转的欧拉角或四元数、平移分量）的​**​导数​**​（梯度/Jacobian/Hessian，具体取决于使用的优化器）。
>    - 优化器根据函数值和导数信息，计算一个更新量 `Δp` 来改进变换参数 `p = (R, t)`。
>    - `p_new = p_old + Δp` （或在优化器内部表示的参数空间上进行更新）。
>    - 判断是否收敛（函数值变化小、参数变化小、达到最大迭代次数）？如果否，转到步骤3开头。
> 
> 4. ​**​输出最优变换：​**​
>    
>    - 将优化得到的最优变换（R*, t*) 作为最终配准结果输出。

## NDT和RANSAC对比

![](/home/maple/笔记/images/2025-07-08-19-25-41-NR1.png)

![](/home/maple/笔记/images/2025-07-08-19-25-46-NR2.png)

**变换矩阵的含义（通常情况）：​**​ 经过 RANSAC (粗配准) 和 ICP (精配准) 后得到的变换矩阵 ​**​T​**​ ​**​通常表示“当前点云坐标系”相对于“目标点云坐标系”的变换​**​

**这个目标点云的坐标系原点(0,0,0)和坐标轴方向，就是你所定义的“世界坐标系”的具体位置和方向**

**变换矩阵 T**

```
[ R     t ]
[ 0 0 0 1 ]
```

`R`: 3x3 的​**​旋转矩阵​**​。描述了当前坐标系相对于世界坐标系的旋转。

`t`: 3x1 的​**​平移向量​**​。描述了当前坐标系​**​原点​**​在世界坐标系中的坐标位置 `(x, y, z)`

### PCL中的SAC-IA算法详解（Sample Consensus - Initial Alignment）

SAC-IA（Sample Consensus Initial Alignment）是点云库（PCL）中用于​**​点云粗配准（Coarse Registration）​**​ 的核心算法

> - **​初始位姿估计​**​：解决两个点云间存在较大旋转/平移时（>30°），ICP等精细配准算法无法收敛的问题
> - ​**​特征驱动配准​**​：基于局部几何特征（如法线、曲率）进行鲁棒匹配
> - ​**​点云预处理​**​：为ICP等精细配准提供良好的初始变换矩阵
> - 典型应用领域：三维重建、SLAM、物体识别、医学图像对齐

## ICP执行流程

![](/home/maple/笔记/images/2025-07-10-17-41-07-ICP.png)

## 将点云从A变化到B

```cpp
pcl::transformPointCloud(*cloud, *transformed_cloud, transform2);
```

这是对点云作变化而非坐标系, transform2是cloud坐标系到transformed坐标系的变化矩阵

## 删除点云前n个点

在 PCL 中删除点云的前 n 个点可以通过直接操作点云数据来实现，因为 PCL 的点云类本质上是一个继承自 `std::vector`的容器

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 确保这是你的点类型定义
// using PointCloudTT = pcl::PointCloud<pcl::PointXYZ>;

void removeFirstNPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n) {
    if (!cloud || n <= 0) return;  // 检查无效输入

    // 确保n不超过点云大小
    if (n > cloud->size()) n = cloud->size();

    // 方法1：使用erase - 最高效 (O(M) 复杂度，M=剩余点数量)
    cloud->points.erase(cloud->points.begin(), cloud->points.begin() + n);

    // 方法2：创建新点云拷贝剩余点 (内存占用更高)
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    // temp->points.assign(cloud->points.begin() + n, cloud->points.end());
    // cloud = temp;

    // 更新点云元数据
    cloud->width = cloud->size();  // 更新宽度
    cloud->height = 1;             // 设置成无序点云
}
```

## 滤波

在 PCL (Point Cloud Library) 中使用 C++ 过滤离群点（Outliers）和噪点（Noise）主要有两种常用方法：​**​统计滤波​**​和​**​半径滤波**

### 方法 1：统计滤波 (Statistical Outlier Removal)

通过分析点云局部邻域的分布特征，移除不符合统计规律的离群点。

#### 步骤：

1. ​**​计算平均距离​**​：对每个点，计算其到 `k`个最近邻点的平均距离。

2. ​**​统计分析​**​：假设所有点平均距离服从高斯分布，计算全局均值 μ和标准差 σ。

3. ​**​阈值过滤​**​：移除距离大于 μ+α⋅σ的点。

```cpp
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr filterStatistical(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, 
    int mean_k, 
    float stddev_mult
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(mean_k);             // 邻域点数 (建议值: 50)
    sor.setStddevMulThresh(stddev_mult); // 阈值倍数 (建议值: 1.0-3.0)
    sor.filter(*filtered_cloud);

    return filtered_cloud;
}
```

### 方法 2：半径滤波 (Radius Outlier Removal)

移除局部邻域密度低于阈值的孤立点。

#### 步骤：

1. ​**​邻域分析​**​：对每个点，统计其半径 r内的邻点数 n。

2. ​**​阈值过滤​**​：移除邻点数 n<Nmin​的点。

```cpp
#include <pcl/filters/radius_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr filterRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, 
    float radius, 
    int min_neighbors
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(input_cloud);
    ror.setRadiusSearch(radius);       // 搜索半径 (单位: 米)
    ror.setMinNeighborsInRadius(min_neighbors); // 最小邻点数
    ror.filter(*filtered_cloud);

    return filtered_cloud;
}
```

## sensor_msgs::LaserScan转换为sensor_msgs::PointCloud2

根据ROS的官方文档，**sensor_msgs/LaserScan**的定义大致如下：

```cpp
#include <sensor_msgs/LaserScan.h>
namespace sensor_msgs {
  struct LaserScan {
    std_msgs::Header header;          // 头信息（时间戳、坐标系）
    float32 angle_min;                // 扫描起始角度（弧度，rad）
    float32 angle_max;                // 扫描结束角度（弧度，rad）
    float32 angle_increment;          // 相邻点的角度间隔（rad）
    float32 time_increment;           // 相邻点的时间间隔（秒，s）
    float32 scan_time;                // 整个扫描周期的时间（秒，s）
    float32 range_min;                // 有效距离最小值（米，m）
    float32 range_max;                // 有效距离最大值（米，m）
    float32[] ranges;                 // 距离测量值数组（单位：m）
    float32[] intensities;            // 强度测量值数组（可选，单位：无量纲）
  };
}
```

**sensor_msgs::PointCloud2**

```cpp
#include <sensor_msgs/PointCloud2.h>
namespace sensor_msgs {
  struct PointCloud2 {
    std_msgs::Header header;               // 头信息（时间戳、坐标系）
    uint32_t height;                       // 点云高度（有序点云时为行数，无序时为 1）
    uint32_t width;                        // 点云宽度（每行的点数，无序时为总点数）
    bool is_bigendian;                     // 数据字节序（大端或小端）
    uint32_t point_step;                   // 单个点的字节大小（所有字段总长度）
    uint32_t row_step;                     // 一行的字节大小（width × point_step）
    bool is_dense;                         // 是否为密集点云（无无效点时为 true）
    sensor_msgs::PointField[] fields;      // 点的字段描述数组（定义每个属性的结构）
    uint8_t[] data;                        // 点云数据的二进制字节数组（按行存储）
  };
}

struct PointField {
  std::string name;       // 字段名（如 "x", "y", "intensity", "rgb"）
  uint32_t offset;        // 字段在单个点中的字节偏移量（相对于点起始位置）
  uint8_t datatype;       // 字段数据类型（枚举值，如 FLOAT32=7, UINT8=1）
  uint32_t count;         // 字段的分量数（如 RGB 颜色通常 count=3，表示 R/G/B 三个分量）
};
```

### **一、PointCloud 与 PointCloud2 的核心区别​**​

| ​**​特性​**​        | ​**​sensor_msgs::PointCloud​**​                                                              | ​**​sensor_msgs::PointCloud2​**​                                                              |
| ----------------- | -------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| ​**​消息结构​**​      | 固定结构，点的字段（如 `x,y,z,r,g,b`）需预先定义，扩展性差。<br>每个点的所有字段必须连续存储，无法动态增减字段。                            | 动态结构，通过 `fields`字段描述点的属性（如 `name`, `offset`, `datatype`, `count`），支持灵活添加/删除字段（如 RGB、强度、时间戳等）。 |
| ​**​存储方式​**​      | 点以 `points`数组形式存储，每个点是 `geometry_msgs::Point32`或自定义类型（如 `PointXYZRGB`）。<br>内存占用较高（因重复存储字段名）。 | 点以二进制块（`data`字段）存储，采用紧凑的二进制格式（类似 `pcl::PCLPointCloud2`），内存效率更高。                               |
| ​**​性能​**​        | 数据解析效率低（需遍历数组并逐个解析字段）。                                                                       | 支持零拷贝（Zero-Copy）读取，直接映射到内存，适合高性能计算（如 PCL 库处理）。                                                |
| ​**​与 PCL 兼容性​**​ | 需手动转换（如 `pcl::fromROSMsg`），兼容性较差。                                                            | 直接与 PCL 的 `pcl::PCLPointCloud2`兼容（通过 `pcl_conversions`转换），是 ROS 与 PCL 集成的推荐格式。                |
| ​**​适用场景​**​      | 简单点云（仅 `x,y,z`），早期 ROS 项目或遗留代码。                                                              | 复杂点云（含 RGB、强度、时间戳等）、大规模点云或需要高性能处理的场景（如 SLAM、目标检测）。                                            |
