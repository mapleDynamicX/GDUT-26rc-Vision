#ifndef __PRE_PCL_H_
#define __PRE_PCL_H_
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <unordered_set>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>  // 质心计算
#include <opencv2/core/types.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include "./../method_math.h"

// 统计离群滤波 参数
#define MeanK  40                   // 每个点要找的「最近邻点数」， 约小约快， 容易误删正常点，越大计算慢，过度平滑
#define StddevMulThresh 1.0f        // 距离超过「平均值 + 2 倍标准差」的点，全部删掉,数值越小 = 删得越狠, 数值越大 = 删得越少

// 体素下采样 参数
#define leaf_size 0.01f            // 体素边长

// 平面拟合 参数
#define DistanceThreshold 0.02f     // 点到 拟合平面的距离上限(m),
#define MaxIterations 500          // 迭代次数，保证找到最优平面

namespace Ten
{
namespace Plane_FitLocator
{

struct Plane_Info
{
    Eigen::Vector3d plane_center = Eigen::Vector3d();
    RPY plane_euler;
    XYZ normal_;
};

class Ten_pre_pcl
{
public:

    Ten_pre_pcl()
    {
        plane_coeffs.reset(new pcl::ModelCoefficients);
    }

    /**
     * @brief 点云滤波器
     * @param input_pclclouds 输入点云
     * @param out_pclclouds 输出的点云
    */
    bool cloud_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pclclouds,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_pclclouds
    )
    {
        voxel_Downsample(input_pclclouds,out_pclclouds);
        statistical_filter(out_pclclouds,out_pclclouds);
        if(out_pclclouds->size() <= 50)
        {
            std::cout << "out_pclclouds->size(): " <<out_pclclouds->size() <<std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief 提取点云中【最大平面】
     * @param input_cloud 输入点云
     * @param output_cloud 输出点云（拟合的最大平面的点云） 
     * @return 拟合成功返回true
     */
    bool Plane_fitter(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud
    )
    {
        // 1 输出检查
        if (input_cloud->empty() || input_cloud->size() < 50)
        {
            std::cout << "Plane_fitter: input_cloud->empty() || input_cloud->size() < 50" << std::endl;
            return false;
        }
        // 2. RANSAC 分割【最大平面】
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        if (!ransac_Plane_Segment(input_cloud, plane_inliers))
        {
            std::cerr << "未检测到平面！" << std::endl;
            return false;
        }
        // 3. 提取最大平面的点云
        extract_Plane_Cloud(input_cloud, plane_inliers, output_cloud);

        //std::cout << "✅ 最大平面点数：" << output_cloud->size() << "/" << input_cloud->size() << std::endl;
        return true;
    }

    /**
     * @brief 根据输入的最大平面点云，写入该平面的质心(中心点)和单位法向量
     * @param input_cloud 输入点云（必须先滤波！）
     * @return 拟合成功返回Plane_Info
     */
    Plane_Info computeCenterAndNormal(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud
    )
    {
        // 1. 计算【最大平面的质心】（中心点）
        Eigen::Vector4f centroid_float;
        pcl::compute3DCentroid(*input_cloud, centroid_float);
        plane_info.plane_center.x() = centroid_float[0];
        plane_info.plane_center.y() = centroid_float[1];
        plane_info.plane_center.z() = centroid_float[2];

        //  2. 计算【单位法向量】并归一化 
        float a = plane_coeffs->values[0];
        float b = plane_coeffs->values[1];
        float c = plane_coeffs->values[2];
        float norm = sqrt(a*a + b*b + c*c);
        float nx = a / norm;
        float ny = b / norm;
        float nz = c / norm;

        // 强制法向量朝向相机（Z轴正方向）
        if (nz > 0)
        {
            nx = -nx;
            ny = -ny;
            nz = -nz;
        }
        plane_info.normal_._x = nx;
        plane_info.normal_._y = ny;
        plane_info.normal_._z = nz;
        // 3. 笛卡尔法向量 → 转为 RPY 欧拉角（适配你的结构体）
        // vectorToRPY(nx, ny, nz, plane_info.plane_normal);
        //createPlaneCoordinate(nx, ny, nz, plane_info.plane_euler);
        correctPlaneYawByM4(plane_info, input_cloud);
        return plane_info;
    }



    /**
     * @brief 矩形平面点云专用：使用四阶矩解算平面内旋转角（正方形/矩形专用）
     * @param plane_info     输入/输出：已包含法向量 normal_ + 质心 plane_center
     * @param cloud          输入：矩形平面点云
     * @param voxel_size     平面内2D体素下采样网格大小（默认 0.03）
     */
    bool correctPlaneYawByM4(
        Plane_Info& plane_info,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float voxel_size = 0.05f)
    {
        // ====================== 1. 输入校验 ======================
        if (cloud == nullptr || cloud->empty() || cloud->size() < 10) {
            std::cerr << "点云无效，跳过yaw修正" << std::endl;
            return false;
        }

        // ====================== 2. 从 Plane_Info 取法向量 + 质心 ======================
        Eigen::Vector3d z_axis(plane_info.normal_._x,
                            plane_info.normal_._y,
                            plane_info.normal_._z);
        Eigen::Vector3d centroid = plane_info.plane_center;

        if (z_axis.norm() < 1e-6) {
            std::cerr << "平面法向量无效" << std::endl;
            return false;
        }
        z_axis.normalize();

        // ====================== 3. 构建平面局部坐标系 ======================
        Eigen::Vector3d arbitrary_vec(1, 0, 0);
        if (fabs(z_axis.dot(arbitrary_vec)) > 0.9) {
            arbitrary_vec = Eigen::Vector3d(0, 1, 0);
        }

        Eigen::Vector3d local_x = z_axis.cross(arbitrary_vec).normalized();
        Eigen::Vector3d local_y = z_axis.cross(local_x).normalized();

        // ====================== 4. 3D 点 → 投影到平面 → 2D 点 ======================
        std::vector<Eigen::Vector2d> points2D;
        points2D.reserve(cloud->size());

        for (const auto& p : cloud->points) {
            Eigen::Vector3d p3d(p.x, p.y, p.z);
            Eigen::Vector3d rel = p3d - centroid;

            double x2d = rel.dot(local_x);
            double y2d = rel.dot(local_y);
            points2D.emplace_back(x2d, y2d);
        }

        // ====================== 5. 平面内2D体素下采样 ======================
        std::map<std::pair<int, int>, std::vector<Eigen::Vector2d>> voxelGrid;
        for (const auto& p : points2D) {
            int ix = static_cast<int>(floor(p.x() / voxel_size));
            int iy = static_cast<int>(floor(p.y() / voxel_size));
            voxelGrid[{ix, iy}].push_back(p);
        }

        std::vector<Eigen::Vector2d> downPoints;
        for (const auto& pair : voxelGrid) {
            const auto& pts = pair.second;
            Eigen::Vector2d sum = Eigen::Vector2d::Zero();
            for (const auto& p : pts) sum += p;
            sum /= pts.size();
            downPoints.push_back(sum);
        }

        if (downPoints.size() < 5) {
            std::cerr << "下采样后点数量过少" << std::endl;
            return false;
        }

        std::cout << "downPoints.size(): " << downPoints.size() << std::endl;
        // ====================== 6. 四阶矩解算旋转角 ======================
        auto computeRotationM4 = [](const std::vector<Eigen::Vector2d>& pts) -> double {
            // 去中心化
            Eigen::Vector2d mean = Eigen::Vector2d::Zero();
            for (const auto& p : pts) mean += p;
            mean /= pts.size();

            std::vector<Eigen::Vector2d> centered;
            for (const auto& p : pts) centered.push_back(p - mean);

            // 高精度遍历 0~45°
            double minM4 = 1e12;
            double bestDeg = 0.0;

            for (double deg = 0; deg <= 45; deg += 0.001) {
                double rad = deg * M_PI / 180.0;
                double c = cos(rad);
                double s = sin(rad);

                double sum = 0.0;
                for (const auto& p : centered) {
                    double proj = p.x() * c + p.y() * s;
                    sum += proj * proj * proj * proj;
                }
                double m4 = sum / centered.size();

                if (m4 < minM4) {
                    minM4 = m4;
                    bestDeg = deg;
                }
            }
            return bestDeg;
        };

        double yaw_deg = computeRotationM4(downPoints);
        double yaw_rad = yaw_deg * M_PI / 180.0;

        // ====================== 7. 生成四阶矩得到的两个正交方向 ======================
        double c = cos(yaw_rad);
        double s = sin(yaw_rad);
        Eigen::Vector3d dir1 = local_x * c + local_y * s;
        Eigen::Vector3d dir2 = local_x * (-s) + local_y * c;

        // ====================== 🔥 8. X 轴约束：与相机 -Y 轴夹角 < 90° ======================
        const Eigen::Vector3d cam_neg_Y(0, -1, 0);
        Eigen::Vector3d x_axis;

        double dot1 = dir1.dot(cam_neg_Y);
        double dot2 = dir2.dot(cam_neg_Y);

        if (dot1 > 0 && dot2 <= 0) {
            x_axis = dir1;
        } else if (dot2 > 0 && dot1 <= 0) {
            x_axis = dir2;
        } else {
            x_axis = (dot1 > dot2) ? dir1 : dir2;
            if (x_axis.dot(cam_neg_Y) < 0) {
                x_axis = -x_axis;
            }
        }

        // ====================== 9. 正交化并构建最终坐标系 ======================
        x_axis = (x_axis - z_axis.dot(x_axis) * z_axis).normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

        // ====================== 10. 旋转矩阵 → RPY ======================
        Eigen::Matrix3d R;
        R.col(0) = x_axis;
        R.col(1) = y_axis;
        R.col(2) = z_axis;

        RPY& rpy = plane_info.plane_euler;
        rpy._roll = atan2(R(2, 1), R(2, 2));
        rpy._pitch = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
        rpy._yaw = atan2(R(1, 0), R(0, 0));

        // std::cout << "===== 四阶矩解算平面内旋转角 =====" << std::endl;
        // std::cout << "解算角度: " << yaw_deg << " °" << std::endl;

        return true;
    }




    // 取到 plane_info, 内部含有中心点和法向量
    Plane_Info get_plane_info(bool is_pre)
    {
        return plane_info;
    }

private:
    Plane_Info plane_info; 
    pcl::ModelCoefficients::Ptr plane_coeffs; // 平面方程系数


    // 体素下采样
    void voxel_Downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
    {
        if (input_cloud->empty())
            return;

        // 创建体素网格滤波器
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(input_cloud);
        // 设置体素大小（核心参数）
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        // 执行滤波
        vg.filter(*output_cloud);
    }


    // 统计离群滤波
    void statistical_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pclclouds,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_pclclouds
    )
    {
        // 统计离群滤波
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_pclclouds);                   // 输入点云
        sor.setMeanK(MeanK);                                  // 近邻点数（和你原参数一致）
        sor.setStddevMulThresh(StddevMulThresh);              // 标准差系数（和你原参数一致）
        
        // 执行滤波
        sor.filter(*out_pclclouds); 
    }


    // RANSAC拟合平面，仅计算平面内点索引和方程系数
    bool ransac_Plane_Segment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointIndices::Ptr& plane_inliers
    )
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(DistanceThreshold);
        seg.setMaxIterations(MaxIterations);

        seg.setInputCloud(input_cloud);
        seg.segment(*plane_inliers, *plane_coeffs); // 计算索引+系数

        return !plane_inliers->indices.empty();
    }

    // 根据点云索引，提取对应点云
    void extract_Plane_Cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const pcl::PointIndices::Ptr& plane_inliers,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud
    )
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(plane_inliers);
        extract.filter(*output_cloud);
    }

    // 构建【标准平面坐标系】Z=法向量(垂直平面)，X/Y=平面内(贴合平面)
    void createPlaneCoordinate(float nx, float ny, float nz, RPY& rpy)
    {
        // 1. 平面法向量 = Z轴 (垂直平面)
        Eigen::Vector3d n(nx, ny, nz);
        n.normalize();

        // 2. 构造平面内的X轴（正交于Z轴，绝对贴合平面）
        Eigen::Vector3d up(0, 1, 0);  // 参考方向
        if (fabs(n.dot(up)) > 0.99) up = Eigen::Vector3d(1, 0, 0); // 避免共线
        Eigen::Vector3d x = up.cross(n).normalized();

        // 3. 构造平面内的Y轴（正交于X/Z，贴合平面）
        Eigen::Vector3d y = n.cross(x).normalized();

        // 4. 旋转矩阵 → 转RPY（TF坐标系：X/Y在平面，Z垂直平面）
        Eigen::Matrix3d rot_mat;
        rot_mat.col(0) = x;
        rot_mat.col(1) = y;
        rot_mat.col(2) = n;

        // 矩阵转欧拉角(roll,pitch,yaw) → 赋值给你的RPY结构体
        rpy._roll  = atan2(rot_mat(2,1), rot_mat(2,2));
        rpy._pitch = atan2(-rot_mat(2,0), sqrt(rot_mat(2,1)*rot_mat(2,1) + rot_mat(2,2)*rot_mat(2,2)));
        rpy._yaw   = atan2(rot_mat(1,0), rot_mat(0,0));
    }

};      // class Ten_pre_pcl
}       // namespace Plane_FitLocator
}       // namespace Ten

#endif 
