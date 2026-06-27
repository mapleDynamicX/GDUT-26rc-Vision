#ifndef __MAPPING_H_
#define __MAPPING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <iostream>
#include <cmath>
#include "./../parameter/parameter.h"
#include "./../lidar.h"
#include <ros/callback_queue.h>

namespace Ten
{
class mapping
{
public:
    /**
     * @brief 构造函数：初始化后自动完成点云订阅与发布器注册
     * @param subscribe_topic 输入点云话题名
     * @param publish_topic   输出累积下采样点云话题名
     * @param voxel_size      基础体素下采样分辨率，单位：米
     * @param max_points      累积点云最大点数上限，超限自动压缩
     */
    mapping(const std::string& subscribe_topic = "/fast_lio2/cloud_registered",
            const std::string& publish_topic = "/relocation/mapping",
            float voxel_size = _voxeldownsample_threshold_,
            size_t max_points = 100000)
    : accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
    , voxel_leaf_size_(voxel_size)
    , max_accumulated_points_(max_points)
    {
        // 绑定私有回调队列，订阅者完全隔离于全局队列
        nh_.setCallbackQueue(&callback_queue_);

        // 订阅点云话题
        cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(subscribe_topic, 10, &mapping::pointCloudCallback, this);

        // 注册发布者
        accumulated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(publish_topic, 10);

        std::cout << "[mapping] 初始化完成" << std::endl
                  << "  订阅话题: " << subscribe_topic << std::endl
                  << "  发布话题: " << publish_topic << std::endl
                  << "  基础体素分辨率: " << voxel_size << "m" << std::endl
                  << "  最大点数上限: " << max_points << std::endl;
    }

    ~mapping() = default;

    /**
     * @brief 处理一次私有回调队列，需在线程循环中调用
     * @param timeout 队列等待超时时间，单位秒
     */
    void callback(double timeout = 0.03)
    {
        callback_queue_.callAvailable(ros::WallDuration(timeout));
    }

    /**
     * @brief 清空当前累积的点云地图
     */
    void clearMap()
    {
        accumulated_cloud_->clear();
        std::cout << "[mapping] 累积点云已清空" << std::endl;
    }

private:
    /**
     * @brief 点云回调核心逻辑：格式转换 → 累加 → 下采样 → 超限压缩 → 发布
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // 1. ROS消息转 PCL PointXYZI 格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        if (current_cloud->empty())
        {
            //std::cerr << "[mapping] 收到空点云，跳过处理" << std::endl;
            return;
        }

        // 2. 当前帧累加到历史累积点云
        *accumulated_cloud_ += *current_cloud;

        // 3. 基础体素下采样：保证日常建图密度均匀
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(accumulated_cloud_);
        voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*downsampled_cloud);
        accumulated_cloud_ = downsampled_cloud;

        // ========== 新增：点数超限压缩逻辑（带滞回区间，避免频繁触发） ==========
        const float trigger_ratio = 1.1f;  // 超过上限10%才触发压缩
        const float target_ratio  = 0.9f;  // 压缩到上限的90%，留缓冲带
        const float max_voxel_scale = 3.0f; // 体素最大放大倍数，避免过度损失细节

        size_t current_points = accumulated_cloud_->size();
        if (current_points > max_accumulated_points_ * trigger_ratio)
        {
            size_t target_points = static_cast<size_t>(max_accumulated_points_ * target_ratio);
            pcl::PointCloud<pcl::PointXYZI>::Ptr compressed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

            // 阶段1：优先通过放大体素均匀降采样（最大限度保留地图几何结构）
            // 体素边长与点数近似成三次反比关系，一次计算目标体素，避免迭代
            float point_ratio = static_cast<float>(current_points) / static_cast<float>(target_points);
            float voxel_scale = std::pow(point_ratio, 1.0f / 3.0f);
            float target_voxel = voxel_leaf_size_ * voxel_scale;
            float max_allowed_voxel = voxel_leaf_size_ * max_voxel_scale;
            target_voxel = std::min(target_voxel, max_allowed_voxel);

            pcl::VoxelGrid<pcl::PointXYZI> compress_voxel;
            compress_voxel.setInputCloud(accumulated_cloud_);
            compress_voxel.setLeafSize(target_voxel, target_voxel, target_voxel);
            compress_voxel.filter(*compressed_cloud);

            // 阶段2：体素达上限后仍超限，用均匀随机降采样兜底（保证全图点分布均匀）
            if (compressed_cloud->size() > target_points)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::RandomSample<pcl::PointXYZI> random_sampler;
                random_sampler.setInputCloud(compressed_cloud);
                random_sampler.setSample(static_cast<unsigned int>(target_points));
                random_sampler.filter(*sampled_cloud);
                compressed_cloud = sampled_cloud;
            }

            accumulated_cloud_ = compressed_cloud;
            // std::cout << "[mapping] 触发点数上限压缩，体素调整为: " << target_voxel 
            //           << "m，压缩后点数: " << accumulated_cloud_->size() << std::endl;
        }

        // 4. 转为ROS消息并发布，保留原时间戳与坐标系
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*accumulated_cloud_, output_msg);
        output_msg.header = cloud_msg->header;
        _Map_GET2_.push(output_msg);
        accumulated_pub_.publish(output_msg);

        //std::cout << "[mapping] 当前累积点数: " << accumulated_cloud_->size() << std::endl;
    }

    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;  // 私有回调队列，与全局完全隔离
    ros::Subscriber cloud_sub_;          // 类内局部订阅者
    ros::Publisher accumulated_pub_;     // 累积点云发布者

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_; // 累积点云地图
    float voxel_leaf_size_;               // 基础体素下采样分辨率
    size_t max_accumulated_points_;       // 累积点云最大点数上限
};

} // namespace Ten

#endif // __MAPPING_H_
