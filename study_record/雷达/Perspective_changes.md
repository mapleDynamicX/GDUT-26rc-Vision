# Perspective changes

## icp测试

### 测试代码：

```cpp
#include<ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // 包含变换函数
#include <pcl/registration/icp_nl.h>
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
void test2()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
    //设置旋转和平移
    Eigen::Matrix3f rot = createRotationMatrix(0, 0, 10);
    Eigen::Vector3f tra = createTranslationVector(1, 1, 0);
    //创建1000个测试点云
    map->width  = 1000;
    map->height = 1;
    map->points.resize (map->width * map->height);
    for (std::size_t i = 0; i < map->points.size(); ++i)
    {
        map->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f) * 10;
        map->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f) * 10;
        map->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f) * 10;
    }
    cloud->width  = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    //得到map->cloud变化矩阵
    Eigen::Matrix4f T = combineRotationAndTranslation(rot, tra);
    //求t的逆矩阵
    Eigen::Matrix4f inverse_transform = T.inverse();
    pcl::transformPointCloud(*map, *cloud, T);
    //输出前三个点观察变化
    std::cout<<"map->points[i]"<<std::endl;
    for(int i = 0; i < 3; i++)
    {
        std::cout<<"x: "<<map->points[i].x<<" y: "<<map->points[i].y<<" z: "<<map->points[i].z<<std::endl;
    }
    std::cout<<"cloud->points[i]"<<std::endl;
    for(int i = 0; i < 3; i++)
    {
        std::cout<<"x: "<<cloud->points[i].x<<" y: "<<cloud->points[i].y<<" z: "<<cloud->points[i].z<<std::endl;
    }
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp_nl;
    // 设置最大对应点距离阈值（超出此距离的点对不参与配准计算）
    icp_nl.setMaxCorrespondenceDistance(1);
    // 设置最大迭代次数（限制优化过程执行次数）
    icp_nl.setMaximumIterations(10);
    // 设置变换收敛阈值（当连续两次变换矩阵的变化小于此值时停止迭代）
    icp_nl.setTransformationEpsilon(1e-5);
    // 设置源点云（待配准的扫描点云）
    icp_nl.setInputSource(cloud);
    // 设置目标点云（参考地图点云）
    icp_nl.setInputTarget(map);
    // 准备配准后的输出点云容器
   pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    // 执行配准计算：使用初始变换估计开始优化过程
    /*
    pcl::IterativeClosestPointNonLinear::align 是点云库（PCL）中用于执行非线性迭代最近点（ICP）配准算法的函数。
    它的目标是将源点云（source cloud）配准到目标点云（target cloud），从而找到两个点云之间的最优变换矩阵（通常是刚性变换：旋转+平移）。
    */
   // 使用传入的initial矩阵作为初始变换估计
    icp_nl.align(*final);
    Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
    // 检查配准是否收敛（达到收敛条件）
    if (!icp_nl.hasConverged())
    {
      ROS_ERROR("ndt_nl not Converged");

    }
    else
    {
        // 获取最终配准变换矩阵（4x4刚体变换矩阵）
        transformation2 = icp_nl.getFinalTransformation();
        // 计算配准质量得分（点对距离平方和均值，越小匹配越好）
        double score = icp_nl.getFitnessScore();
        std::cout<<"icp score"<< score <<std::endl;
        //  print4x4Matrix(transformation);
    }
    std::cout<<"final->points[i]"<<std::endl;
    for(int i = 0; i < 3; i++)
    {
        std::cout<<"x: "<<final->points[i].x<<" y: "<<final->points[i].y<<" z: "<<final->points[i].z<<std::endl;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *output, transformation2);
    std::cout<<"output->points[i]"<<std::endl;
    for(int i = 0; i < 3; i++)
    {
        std::cout<<"x: "<<output->points[i].x<<" y: "<<output->points[i].y<<" z: "<<output->points[i].z<<std::endl;
    }
    std::cout<<"T: "<<T<<std::endl;
    std::cout<<"transformation2: "<<transformation2<<std::endl;
    std::cout<<"inverse_transform: "<<inverse_transform<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    test2();
    return 0;
}
```

### 输出结果

```bash
map->points[i]
x: 3.52222 y: -1.51883 z: -1.06395
x: -3.97406 y: -4.73106 z: 2.92602
x: -7.31898 y: 6.67105 z: 4.41304
cloud->points[i]
x: 4.73245 y: 0.11587 z: -1.06395
x: -2.09215 y: -4.34927 z: 2.92602
x: -7.36621 y: 6.29877 z: 4.41304
icp score1.48821e-10
final->points[i]
x: 3.52222 y: -1.51884 z: -1.06395
x: -3.97406 y: -4.73107 z: 2.92603
x: -7.31899 y: 6.67104 z: 4.41303
output->points[i]
x: 3.52222 y: -1.51884 z: -1.06395
x: -3.97406 y: -4.73107 z: 2.92603
x: -7.31899 y: 6.67104 z: 4.41304
T:  0.984808 -0.173648         0         1
 0.173648  0.984808         0         1
        0         0         1         0
        0         0         0         1
transformation2:     0.984808     0.173648  1.62632e-07     -1.15846
   -0.173648     0.984808    6.966e-07    -0.811169
-3.90573e-08 -7.12753e-07            1  2.17045e-06
           0            0            0            1
inverse_transform:  0.984808  0.173648         0  -1.15846
-0.173648  0.984808        -0  -0.81116
        0        -0         1        -0
       -0         0        -0         1
```

### 结论

> icp配准结果实际上是求当前点云通过坐标变化转化为世界点云时
> 
> 当前的点云与真实值间存在的误差而不能直接得出当前点云相对世界的位置

#### 1. ​**​特征提取与匹配​**​

- ​**​算法选择​**​：ORB（实时性好）或SIFT（精度高）

- ​**​关键步骤​**​：
  
  1. 提取当前帧的2D特征点
  
  2. 在点云中找到对应3D坐标
  
  3. 与上一帧特征点进行匹配（FLANN/KNN）

#### 2. ​**​位姿估计（视觉里程计VO）​**​

- ​**​求解方法​**​：Perspective-n-Point (PnP) + RANSAC
  
  - 输入：3D-2D点对（上一帧的3D点 + 当前帧的2D点）
  
  - 输出：相机旋转矩阵R和平移向量t

- ​**​优化​**​：使用非线性优化（如Levenberg-Marquardt）细化位姿

## 1. 按索引访问（推荐）

Eigen 重载了括号运算符 `()`

```cpp
#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Matrix3f R_x;
    R_x << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;

    // 访问第 i 行第 j 列的元素（索引从 0 开始）
    float val = R_x(1, 2); // 第二行第三列
    std::cout << val << std::endl; // 输出 0

    // 修改元素
    R_x(0, 0) = 2.0f;
    std::cout << R_x << std::endl;
}


#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Vector3f translation(1.0f, 2.0f, 3.0f);

    float x = translation(0); // x 分量
    float y = translation(1); // y 分量
    float z = translation(2); // z 分量

    std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;

    // 修改元素
    translation(0) = 10.0f;
    std::cout << translation << std::endl;
}

```
