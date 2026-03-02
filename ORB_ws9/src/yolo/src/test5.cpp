#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <limits>
// 新增：std::cout需要的头文件
#include <iostream>
// 新增：控制输出精度需要的头文件
#include <iomanip>

// OBB有向包围盒定义
struct OBB {
  Eigen::Vector3d center;       // 中心坐标
  Eigen::Vector3d axes[3];      // 3个正交单位轴（x/y/z）
  Eigen::Vector3d extents;      // 3个轴的半长度（half-extents）
};

// 射线定义
struct Ray {
  Eigen::Vector3d origin;       // 起点
  Eigen::Vector3d direction;    // 方向（单位向量）
};

/**
 * @brief Slab算法：射线与OBB相交检测
 * @param ray 输入射线
 * @param obb 输入OBB
 * @param t_out 输出相交点的t参数（P = origin + t*direction）
 * @param intersection 输出相交点坐标
 * @return 是否相交
 */
bool rayIntersectsOBB_Slab(const Ray& ray, const OBB& obb, double& t_out, Eigen::Vector3d& intersection) {
  double t_min = 0.0;                     // 射线有效起始t（向前）
  double t_max = std::numeric_limits<double>::infinity();
  Eigen::Vector3d origin_local = ray.origin - obb.center;  // 转到OBB局部坐标系

  // 遍历3个轴计算Slab区间
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector3d& axis = obb.axes[i];
    double ext = obb.extents[i];
    double dir_dot = ray.direction.dot(axis);
    double orig_dot = origin_local.dot(axis);

    // 射线与当前轴平行：检查起点是否在Slab内
    if (std::fabs(dir_dot) < 1e-8) {
      if (std::fabs(orig_dot) > ext) return false;
      continue;
    }

    // 计算射线与两个Slab面的t值
    double t1 = (ext - orig_dot) / dir_dot;
    double t2 = (-ext - orig_dot) / dir_dot;
    if (t1 > t2) std::swap(t1, t2);  // 保证t1 <= t2

    // 更新全局t区间（取交集）
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if (t_min > t_max) return false;  // 区间无效，提前退出
  }

  // 相交：返回最近交点（t_min）
  t_out = t_min;
  intersection = ray.origin + t_out * ray.direction;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ray_obb_intersection_node");
  ros::NodeHandle nh;

  // 构造测试OBB（1x1x1立方体，中心在(1,0,0)）
  OBB test_obb;
  test_obb.center = Eigen::Vector3d(1.0, 0.0, 0.0);
  test_obb.axes[0] = Eigen::Vector3d(1,0,0);  // x轴
  test_obb.axes[1] = Eigen::Vector3d(0,1,0);  // y轴
  test_obb.axes[2] = Eigen::Vector3d(0,0,1);  // z轴
  test_obb.extents = Eigen::Vector3d(0.5, 0.5, 0.5);

  // 构造测试射线（起点(0,0,0)，方向(1,0,0)）
  Ray test_ray;
  test_ray.origin = Eigen::Vector3d(0.0, 0.0, 0.0);
  test_ray.direction = Eigen::Vector3d(1.0, 0.0, 0.0).normalized();

  // 执行相交检测
  double t;
  Eigen::Vector3d intersection;
  bool is_hit = rayIntersectsOBB_Slab(test_ray, test_obb, t, intersection);

  // 输出结果（转换为OpenCV格式）
  if (is_hit) {
    // 替换ROS_INFO为std::cout，用setprecision控制小数位数
    std::cout << "✅ 射线与OBB相交" << std::endl;
    std::cout << "t参数: " << std::fixed << std::setprecision(2) << t 
              << " | 相交点: (" << intersection.x() << ", " << intersection.y() << ", " << intersection.z() << ")" << std::endl;
    cv::Mat cv_point = (cv::Mat_<double>(3,1) << intersection.x(), intersection.y(), intersection.z());
    std::cout << "OpenCV格式: [" << cv_point.at<double>(0) << ", " 
              << cv_point.at<double>(1) << ", " << cv_point.at<double>(2) << "]" << std::endl;
  } else {
    std::cout << "❌ 射线与OBB不相交" << std::endl;
  }

  ros::spinOnce();
  return 0;
}
