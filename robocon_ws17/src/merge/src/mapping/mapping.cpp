#include "mapping.h"
#include <pcl/common/transforms.h>
#include <cmath>

namespace Ten
{

// ==================== 构造函数：初始化TSDF建图系统 ====================
// 功能：加载参数、初始化话题订阅/发布、打印初始化信息
TSDFMapping::TSDFMapping() : nh_("/map")
{
    // 体素分辨率：三维建图的最小立方体单元边长（单位：米）
    // 数值越小，地图精度越高，但计算量和内存消耗越大
    voxel_resolution_ = 0.05f;

    // TSDF截断距离：符号距离函数的有效截断范围（单位：米）
    // 超出该距离的体素强制标记为自由空间，通常设为 3倍体素分辨率（TSDF经典配置）
    truncation_distance_ = 5 * voxel_resolution_;

    // 体素最大权重：单个体素的观测权重上限
    // 防止权重无限累积，保证新观测数据能正常更新地图
    max_weight_ = 10.0f;

    // 体素最小权重：有效体素的权重阈值
    // 地图清理时，权重低于该值的无效体素会被直接删除，过滤噪声和无效观测
    min_weight_ = 2.0f;

    // 权重衰减因子：动态环境适配参数（取值范围0~1）
    // 物体消失时，旧观测的权重会按该系数衰减，让地图逐渐遗忘旧物体
    decay_factor_ = 0.7f;

    // 环境变化检测阈值：判断场景是否发生突变的临界值（单位：米）
    // TSDF值变化超过该阈值，视为环境动态改变，直接重置体素权重和数值
    change_threshold_ = voxel_resolution_;

    // 激光雷达最大有效观测距离（单位：米）
    // 超过该距离的激光点视为无效，不参与TSDF地图融合
    max_range_ = 50.0f;

    // 激光雷达最小有效观测距离（单位：米）
    // 低于该距离的点为雷达自身噪声/近场干扰，直接过滤不处理
    min_range_ = 0.3f;

    // 订阅配准后的点云话题，队列长度10，绑定回调函数
    cloud_sub_ = nh_.subscribe("/fast_lio2/cloud_registered", 10, 
                              &TSDFMapping::cloudCallback, this);
    // 订阅里程计话题，队列长度100，绑定回调函数
    odom_sub_ = nh_.subscribe("/fast_lio2/odom", 10, 
                             &TSDFMapping::odomCallback, this);

    // 发布TSDF表面点云，队列长度1
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/tsdf_map/surface", 3);

    // 打印初始化成功日志
    ROS_INFO("TSDF Mapping initialized successfully!");
    // 打印核心参数
    ROS_INFO("Parameters: voxel_resolution=%.2fm, truncation_distance=%.2fm", 
             voxel_resolution_, truncation_distance_);
}

void TSDFMapping::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    // 更新最新位姿
    latest_pose_.setIdentity();
    latest_pose_.translation() = Eigen::Vector3d(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z
    );
    latest_pose_.linear() = Eigen::Quaterniond(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z
    ).toRotationMatrix();
    
    has_pose_.store(true);
}

void TSDFMapping::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 等待第一个位姿
    if (!has_pose_.load())
    {
        ROS_WARN_THROTTLE(1.0, "Waiting for first odometry message...");
        return;
    }

    // 转换点云格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 获取当前位姿（线程安全）
    Eigen::Isometry3d current_pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose = latest_pose_;
    }

    // 核心：TSDF融合
    integratePointCloud(cloud, current_pose);

    // 定期清理地图（每10帧清理一次）
    static int frame_count = 0;
    if (++frame_count % 10 == 0)
    {
        cleanMap();
    }

    // 发布调试地图
    publishDebugMap();
}

void TSDFMapping::integratePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                     const Eigen::Isometry3d& pose)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    Eigen::Vector3d origin = pose.translation();

    for (const auto& point : cloud->points)
    {
        Eigen::Vector3d world_point(point.x, point.y, point.z);
        Eigen::Vector3d direction = world_point - origin;
        double distance = direction.norm();

        // 距离滤波
        if (distance < min_range_ || distance > max_range_)
        {
            continue;
        }

        // 光线投射：获取所有可见体素和表面体素
        std::vector<VoxelKey> visible_voxels;
        VoxelKey surface_voxel;
        rayCast(origin, world_point, visible_voxels, surface_voxel);

        // 1. 更新自由空间体素（光线穿过的体素）
        for (const auto& voxel_key : visible_voxels)
        {
            auto it = tsdf_map_.find(voxel_key);
            if (it == tsdf_map_.end())
            {
                // 新体素：初始化为空
                tsdf_map_[voxel_key] = TSDFVoxel(truncation_distance_, 1.0f);
            }
            else
            {
                TSDFVoxel& voxel = it->second;
                float obs_tsdf = truncation_distance_; // 自由空间观测值为正最大值

                // 检测变化
                float diff = std::abs(obs_tsdf - voxel.tsdf);
                if (diff > change_threshold_ && voxel.weight > min_weight_)
                {
                    // 物体消失：衰减权重并向观测值调整
                    voxel.tsdf = obs_tsdf * (1 - decay_factor_) + voxel.tsdf * decay_factor_;
                    voxel.weight *= decay_factor_;
                }
                else
                {
                    // 不变区域：标准加权平均
                    float weight_obs = 1.0f;
                    float total_weight = voxel.weight + weight_obs;
                    voxel.tsdf = (voxel.tsdf * voxel.weight + obs_tsdf * weight_obs) / total_weight;
                    voxel.weight = std::min(total_weight, max_weight_);
                }
            }
        }

        // 2. 更新表面体素（光线终点）
        auto it = tsdf_map_.find(surface_voxel);
        if (it == tsdf_map_.end())
        {
            // 新表面体素
            tsdf_map_[surface_voxel] = TSDFVoxel(0.0f, 1.0f);
        }
        else
        {
            TSDFVoxel& voxel = it->second;
            float obs_tsdf = 0.0f; // 表面观测值为0

            // 检测变化
            float diff = std::abs(obs_tsdf - voxel.tsdf);
            if (diff > change_threshold_ && voxel.weight > min_weight_)
            {
                // 物体出现：用新观测覆盖
                voxel.tsdf = obs_tsdf;
                voxel.weight = 1.0f;
            }
            else
            {
                // 不变区域：标准加权平均
                float weight_obs = 1.0f;
                float total_weight = voxel.weight + weight_obs;
                voxel.tsdf = (voxel.tsdf * voxel.weight + obs_tsdf * weight_obs) / total_weight;
                voxel.weight = std::min(total_weight, max_weight_);
            }
        }
    }
}

void TSDFMapping::rayCast(const Eigen::Vector3d& origin, const Eigen::Vector3d& point,
                         std::vector<VoxelKey>& visible_voxels, VoxelKey& surface_voxel)
{
    // Amanatides-Woo快速体素遍历算法
    Eigen::Vector3d direction = (point - origin).normalized();
    
    // 起始体素
    VoxelKey current_voxel = worldToVoxel(origin);
    surface_voxel = worldToVoxel(point);
    
    // 步长方向
    int step_x = (direction.x() > 0) ? 1 : -1;
    int step_y = (direction.y() > 0) ? 1 : -1;
    int step_z = (direction.z() > 0) ? 1 : -1;
    
    // 到下一个体素边界的距离
    double t_max_x, t_max_y, t_max_z;
    double t_delta_x, t_delta_y, t_delta_z;
    
    // 计算t_max和t_delta
    if (std::abs(direction.x()) < 1e-6)
    {
        t_max_x = 1e10;
        t_delta_x = 1e10;
    }
    else
    {
        double voxel_boundary_x = (current_voxel.x + (step_x > 0 ? 1 : 0)) * voxel_resolution_;
        t_max_x = (voxel_boundary_x - origin.x()) / direction.x();
        t_delta_x = voxel_resolution_ / std::abs(direction.x());
    }
    
    if (std::abs(direction.y()) < 1e-6)
    {
        t_max_y = 1e10;
        t_delta_y = 1e10;
    }
    else
    {
        double voxel_boundary_y = (current_voxel.y + (step_y > 0 ? 1 : 0)) * voxel_resolution_;
        t_max_y = (voxel_boundary_y - origin.y()) / direction.y();
        t_delta_y = voxel_resolution_ / std::abs(direction.y());
    }
    
    if (std::abs(direction.z()) < 1e-6)
    {
        t_max_z = 1e10;
        t_delta_z = 1e10;
    }
    else
    {
        double voxel_boundary_z = (current_voxel.z + (step_z > 0 ? 1 : 0)) * voxel_resolution_;
        t_max_z = (voxel_boundary_z - origin.z()) / direction.z();
        t_delta_z = voxel_resolution_ / std::abs(direction.z());
    }
    
    // 遍历体素直到到达表面体素
    while (current_voxel != surface_voxel)
    {
        visible_voxels.push_back(current_voxel);
        
        // 移动到下一个体素
        if (t_max_x < t_max_y && t_max_x < t_max_z)
        {
            current_voxel.x += step_x;
            t_max_x += t_delta_x;
        }
        else if (t_max_y < t_max_z)
        {
            current_voxel.y += step_y;
            t_max_y += t_delta_y;
        }
        else
        {
            current_voxel.z += step_z;
            t_max_z += t_delta_z;
        }
    }
}

VoxelKey TSDFMapping::worldToVoxel(const Eigen::Vector3d& world_point) const
{
    return VoxelKey(
        static_cast<int>(std::floor(world_point.x() / voxel_resolution_)),
        static_cast<int>(std::floor(world_point.y() / voxel_resolution_)),
        static_cast<int>(std::floor(world_point.z() / voxel_resolution_))
    );
}

Eigen::Vector3d TSDFMapping::voxelToWorld(const VoxelKey& voxel_key) const
{
    return Eigen::Vector3d(
        (voxel_key.x + 0.5) * voxel_resolution_,
        (voxel_key.y + 0.5) * voxel_resolution_,
        (voxel_key.z + 0.5) * voxel_resolution_
    );
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TSDFMapping::extractSurfaceCloud() 
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    for (const auto& pair : tsdf_map_)
    {
        const TSDFVoxel& voxel = pair.second;
        const VoxelKey& key = pair.first;
        
        // 只保留表面附近且权重足够的体素
        if (std::abs(voxel.tsdf) < voxel_resolution_ && voxel.weight >= min_weight_)
        {
            Eigen::Vector3d world_point = voxelToWorld(key);
            pcl::PointXYZ point;
            point.x = world_point.x();
            point.y = world_point.y();
            point.z = world_point.z();
            surface_cloud->push_back(point);
        }
    }
    
    surface_cloud->width = surface_cloud->size();
    surface_cloud->height = 1;
    surface_cloud->is_dense = true;
    
    return surface_cloud;
}

void TSDFMapping::cleanMap()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    auto it = tsdf_map_.begin();
    while (it != tsdf_map_.end())
    {
        if (it->second.weight < min_weight_)
        {
            it = tsdf_map_.erase(it);
        }
        else
        {
            // 全局权重衰减：处理缓慢变化的环境
            it->second.weight *= 0.99f;
            ++it;
        }
    }
}

void TSDFMapping::publishDebugMap()
{
    auto surface_cloud = extractSurfaceCloud();
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*surface_cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_init";
    cloud_msg.header.stamp = ros::Time::now();
    
    map_pub_.publish(cloud_msg);
}

} // namespace Ten
