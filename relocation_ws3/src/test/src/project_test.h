#ifndef __PROJECT_TEST_H_
#define __PROJECT_TEST_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <cmath>
#include "./parameter/parameter.h"

namespace Ten
{
    //重定位测试
    // extern int _normal_points_;
    // extern int _fpfh_points_;
    // extern double _fpfh_match_distance_;
    // extern double _th1_;
    // extern double _th2_;
    // extern double _th3_;

    
    // 点云类型定义
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;
    using NormalT = pcl::Normal;
    using NormalCloudT = pcl::PointCloud<NormalT>;
    using FPFHSignatureT = pcl::FPFHSignature33;
    using FPFHCloudT = pcl::PointCloud<FPFHSignatureT>;
    
    // ===================== 子函数1：体素下采样 =====================
    void voxelDownSample(const PointCloudT::Ptr& input_cloud, 
                         PointCloudT::Ptr& output_cloud, 
                         float leaf_size)
    {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*output_cloud);
        std::cout << "[下采样] 原始点数: " << input_cloud->size() 
                  << " | 下采样后点数: " << output_cloud->size() << std::endl;
    }
    
    // ===================== 子函数2：计算点云法向量 =====================
    void computeNormals(const PointCloudT::Ptr& cloud, 
                        NormalCloudT::Ptr& normals)
    {
        pcl::NormalEstimation<PointT, NormalT> ne;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(_normal_points_);  // 固定近邻数，适配PointXYZI
        ne.compute(*normals);
    }
    
    // ===================== 子函数3：提取FPFH特征 =====================
    void extractFPFHFeatures(const PointCloudT::Ptr& cloud, 
                             FPFHCloudT::Ptr& fpfh_features)
    {
        // 1. 先计算法向量
        NormalCloudT::Ptr normals(new NormalCloudT);
        computeNormals(cloud, normals);
    
        // 2. 计算FPFH特征
        pcl::FPFHEstimation<PointT, NormalT, FPFHSignatureT> fpfh;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(tree);
        fpfh.setKSearch(_fpfh_points_);
        fpfh.compute(*fpfh_features);
        std::cout << "[FPFH] 特征提取完成，特征数: " << fpfh_features->size() << std::endl;
    }
    
    // ===================== 子函数4：FPFH特征匹配，生成对应点对 =====================
    void matchFeatures(const PointCloudT::Ptr& src_cloud,   // 局部点云
                       const PointCloudT::Ptr& tgt_cloud,   // 全局点云
                       const FPFHCloudT::Ptr& src_fpfh, 
                       const FPFHCloudT::Ptr& tgt_fpfh, 
                       pcl::Correspondences& correspondences)
    {
        pcl::registration::CorrespondenceEstimation<FPFHSignatureT, FPFHSignatureT> ce;
        ce.setInputSource(src_fpfh);
        ce.setInputTarget(tgt_fpfh);
        ce.determineCorrespondences(correspondences, _fpfh_match_distance_); // 特征匹配阈值
        std::cout << "[匹配] 生成对应点对数量: " << correspondences.size() << std::endl;
    }
    
    // ===================== 核心主函数：你要求的完整功能 =====================
    /**
     * @brief 点云配对+距离统计函数
     * @param global_cloud 全局点云 (输入)
     * @param local_cloud 局部点云 (输入)
     * @param leaf_size 下采样体素大小 (输入)
     * @param transform_matrix 4x4变换矩阵 (输入)
     * @param th1 阈值1 (输入)
     * @param th2 阈值2 (输入)
     * @param th3 阈值3 (输入)
     */
    void calculateMatchPointDistanceStats(
        const PointCloudT::Ptr& global_cloud,
        const PointCloudT::Ptr& local_cloud,
        float leaf_size,
        const Eigen::Matrix4d& transform_matrix,
        double th1 = _th1_,
        double th2 = _th2_,
        double th3 = _th3_
    )
    {
        // ========== 0. 输入合法性校验 ==========
        if (global_cloud->empty() || local_cloud->empty())
        {
            std::cerr << "[错误] 全局/局部点云为空！" << std::endl;
            return;
        }
        if (th1 >= th2 || th2 >= th3)
        {
            std::cerr << "[错误] 阈值必须满足 th1 < th2 < th3！" << std::endl;
            return;
        }
    
        // ========== 1. 点云下采样 ==========
        PointCloudT::Ptr global_down(new PointCloudT);
        PointCloudT::Ptr local_down(new PointCloudT);
        voxelDownSample(global_cloud, global_down, leaf_size);
        voxelDownSample(local_cloud, local_down, leaf_size);
    
        // ========== 2. 提取FPFH特征 ==========
        FPFHCloudT::Ptr global_fpfh(new FPFHCloudT);
        FPFHCloudT::Ptr local_fpfh(new FPFHCloudT);
        extractFPFHFeatures(global_down, global_fpfh);
        extractFPFHFeatures(local_down, local_fpfh);
    
        // ========== 3. 特征匹配，生成点对 ==========
        pcl::Correspondences correspondences;
        // 注意：src=local，tgt=global（和你的配准逻辑一致）
        matchFeatures(local_down, global_down, local_fpfh, global_fpfh, correspondences);
    
        if (correspondences.empty())
        {
            std::cerr << "[错误] 未匹配到任何点对！" << std::endl;
            return;
        }
    
        // ========== 4. 遍历匹配点对，计算距离+统计 ==========
        int count_less_th1 = 0;    // 距离 < th1
        int count_between_th1_th2 = 0; // th1 ≤ 距离 < th2
        int count_between_th2_th3 = 0; // th2 ≤ 距离 < th3
        int count_greater_th3 = 0; // 距离 > th3
    
        for (const auto& corr : correspondences)
        {
            // 1. 获取匹配的全局点（目标点）
            const PointT& global_point = global_down->points[corr.index_match];
            // 2. 获取匹配的局部点（源点）
            const PointT& local_point = local_down->points[corr.index_query];
    
            // 3. 全局点 → 齐次坐标 → 应用变换矩阵
            Eigen::Vector4d global_point_homo(
                global_point.x,
                global_point.y,
                global_point.z,
                1.0
            );
            Eigen::Vector4d transformed_global_homo = transform_matrix * global_point_homo;
    
            // 4. 提取变换后的3D坐标
            PointT transformed_global;
            transformed_global.x = transformed_global_homo(0);
            transformed_global.y = transformed_global_homo(1);
            transformed_global.z = transformed_global_homo(2);
    
            // 5. 计算欧式距离（变换后的全局点 ↔ 局部匹配点）
            double distance = std::sqrt(
                std::pow(transformed_global.x - local_point.x, 2) +
                std::pow(transformed_global.y - local_point.y, 2) +
                std::pow(transformed_global.z - local_point.z, 2)
            );
    
            // 6. 按阈值统计
            if (distance < th1)
                count_less_th1++;
            else if (distance >= th1 && distance < th2)
                count_between_th1_th2++;
            else if (distance >= th2 && distance < th3)
                count_between_th2_th3++;
            else
                count_greater_th3++;
        }
    
        // ========== 5. 打印统计结果 ==========
        std::cout << "\n==================== 匹配点距离统计结果 ====================" << std::endl;
        std::cout << "总匹配点对数: " << correspondences.size() << std::endl;
        std::cout << "距离 < " << th1 << "m : " << count_less_th1 << " 个" << std::endl;
        std::cout << th1 << "m ≤ 距离 < " << th2 << "m : " << count_between_th1_th2 << " 个" << std::endl;
        std::cout << th2 << "m ≤ 距离 < " << th3 << "m : " << count_between_th2_th3 << " 个" << std::endl;
        std::cout << "距离 > " << th3 << "m : " << count_greater_th3 << " 个" << std::endl;
        std::cout << "===========================================================\n" << std::endl;
    }
    
    // ===================== 测试示例（可直接运行） =====================
    int main()
    {
        // 1. 创建测试点云（也可以替换为你自己的PCD读取）
        PointCloudT::Ptr global_cloud(new PointCloudT);
        PointCloudT::Ptr local_cloud(new PointCloudT);
        // 这里省略点云填充，你可以用 pcl::io::loadPCDFile 读取真实点云
    
        // 2. 测试参数
        float leaf_size = 0.3f;                // 下采样大小
        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity(); // 单位变换矩阵
        double th1 = 0.2, th2 = 0.5, th3 = 1.0; // 三个阈值
    
        // 3. 调用核心函数
        calculateMatchPointDistanceStats(global_cloud, local_cloud, leaf_size, trans, th1, th2, th3);
    
        return 0;
    }
       



}




#endif
