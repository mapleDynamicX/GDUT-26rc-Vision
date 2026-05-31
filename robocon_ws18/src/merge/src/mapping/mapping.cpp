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
    voxel_resolution_ = 0.10f;

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

    // ==================== 新增：XYZ三维范围滤波参数 ====================
    // 默认范围：x: -20~20m, y: -20~20m, z: -5~5m
    // 你可以根据实际场景调整这些值
    min_x_ = -2.0f;
    max_x_ = 15.0f;
    min_y_ = -5.0f;
    max_y_ = 10.0f;
    min_z_ = -1.0f;
    max_z_ = 2.0f;
    // ================================================================

    // ==================== 新增：滑动窗口与下采样参数 ====================
    // 修改：从2帧改为8帧，平衡动态过滤效果和性能
    // 如果你确实需要更快的地图更新，可以降到5帧，但不建议低于3帧
    sliding_window_size_ = 8;
    downsample_resolution_ = 0.1f;  // 与体素分辨率保持一致
    dynamic_threshold_ = 0.1f;  // 动态点检测阈值（TSDF值差异大于此值视为动态）

    // 初始化全局地图
    global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // ================================================================

    // ==================== 新增：初始化原始累积点云 ====================
    raw_accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    raw_accumulated_downsample_res_ = 0.3f; // 写死0.3m，你后面可以自己调整
    // ================================================================

    // 订阅配准后的点云话题，队列长度10，绑定回调函数
    cloud_sub_ = nh_.subscribe("/fast_lio2/cloud_registered", 10, 
                              &TSDFMapping::cloudCallback, this);
    // 订阅里程计话题，队列长度100，绑定回调函数
    odom_sub_ = nh_.subscribe("/fast_lio2/odom", 10, 
                             &TSDFMapping::odomCallback, this);

    // 发布完整TSDF地图点云
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/tsdf_map/full", 3);

    // 打印初始化成功日志
    ROS_INFO("TSDF Mapping initialized successfully!");
    // 打印核心参数
    ROS_INFO("Parameters: voxel_resolution=%.2fm, truncation_distance=%.2fm", 
             voxel_resolution_, truncation_distance_);
    ROS_INFO("XYZ filter range: x[%.1f, %.1f], y[%.1f, %.1f], z[%.1f, %.1f]",
             min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    ROS_INFO("Sliding window size: %d frames, downsample resolution: %.2fm", 
             sliding_window_size_, downsample_resolution_);
    ROS_INFO("Raw accumulated cloud downsample resolution: %.2fm",
             raw_accumulated_downsample_res_);
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

    // ==================== 新增：累积原始点云 ====================
    {
        std::lock_guard<std::mutex> lock(raw_accumulated_mutex_);
        *raw_accumulated_cloud_ += *cloud;
    }
    // ================================================================

    // 获取当前位姿（线程安全）
    Eigen::Isometry3d current_pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose = latest_pose_;
    }

    // 核心：TSDF融合
    integratePointCloud(cloud, current_pose);

    // ==================== 新增：滑动窗口处理逻辑 ====================
    // 基于TSDF过滤当前帧的动态点
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = filterDynamicPoints(cloud);
    
    // 将过滤后的点云加入滑动窗口
    sliding_window_.push_back(filtered_cloud);
    
    // 当滑动窗口满时，合并到全局地图
    if (sliding_window_.size() >= sliding_window_size_)
    {
        mergeSlidingWindowToGlobalMap();
        sliding_window_.clear();  // 清空滑动窗口，准备下一轮
    }
    // ================================================================

    // 修改：每20帧执行一次地图清理和累积点云下采样
    // 原来每2帧一次会导致严重的性能问题
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 20 == 0)
    {
        cleanMap();
        downsampleRawAccumulatedCloud();
    }

    // 发布完整地图
    publishFullMap();
}

void TSDFMapping::integratePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                     const Eigen::Isometry3d& pose)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    Eigen::Vector3d origin = pose.translation();

    for (const auto& point : cloud->points)
    {
        Eigen::Vector3d world_point(point.x, point.y, point.z);

        // ==================== 修改：XYZ三维范围滤波 ====================
        // 替换原来的距离滤波，只处理在指定立方体内的点
        if (world_point.x() < min_x_ || world_point.x() > max_x_ ||
            world_point.y() < min_y_ || world_point.y() > max_y_ ||
            world_point.z() < min_z_ || world_point.z() > max_z_)
        {
            continue;
        }
        // ================================================================

        Eigen::Vector3d direction = world_point - origin;

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

// ==================== 新增：基于TSDF的动态点云过滤 ====================
pcl::PointCloud<pcl::PointXYZI>::Ptr TSDFMapping::filterDynamicPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    for (const auto& point : cloud->points)
    {
        Eigen::Vector3d world_point(point.x, point.y, point.z);
        VoxelKey voxel_key = worldToVoxel(world_point);
        
        auto it = tsdf_map_.find(voxel_key);
        if (it == tsdf_map_.end())
        {
            // 新体素：保留该点（可能是新出现的静态物体）
            filtered_cloud->push_back(point);
            continue;
        }
        
        const TSDFVoxel& voxel = it->second;
        
        // 过滤逻辑：
        // 1. 如果体素权重足够高（已被多次观测为静态）
        // 2. 且当前点的TSDF值与体素TSDF值差异超过阈值
        // 则认为是动态点，过滤掉
        if (voxel.weight >= min_weight_ && 
            std::abs(voxel.tsdf) > dynamic_threshold_)
        {
            // 自由空间中的点：动态物体，过滤
            continue;
        }
        
        // 保留静态点和不确定的点
        filtered_cloud->push_back(point);
    }
    
    filtered_cloud->width = filtered_cloud->size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    ROS_DEBUG_THROTTLE(1.0, "Dynamic filtering: %d -> %d points", 
                      (int)cloud->size(), (int)filtered_cloud->size());
    
    return filtered_cloud;
}

// ==================== 新增：合并滑动窗口到全局地图 ====================
void TSDFMapping::mergeSlidingWindowToGlobalMap()
{
    std::lock_guard<std::mutex> lock(global_map_mutex_);
    
    ROS_INFO("Merging sliding window (%d frames) to global map...", 
             (int)sliding_window_.size());
    
    // 1. 基于TSDF剔除全局地图中的自由空间点
    removeFreeSpacePointsFromGlobalMap();
    
    // 2. 叠加滑动窗口中的所有点云
    for (const auto& cloud : sliding_window_)
    {
        *global_map_ += *cloud;
    }
    
    // 3. 对全局地图进行体素下采样
    downsampleGlobalMap();
    
    ROS_INFO("Global map updated: %d points", (int)global_map_->size());
}

// ==================== 新增：剔除全局地图中的自由空间点 ====================
void TSDFMapping::removeFreeSpacePointsFromGlobalMap()
{
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cleaned_map(new pcl::PointCloud<pcl::PointXYZI>());
    
    for (const auto& point : global_map_->points)
    {
        Eigen::Vector3d world_point(point.x, point.y, point.z);
        VoxelKey voxel_key = worldToVoxel(world_point);
        
        auto it = tsdf_map_.find(voxel_key);
        if (it == tsdf_map_.end())
        {
            // 未被观测过的体素：保留
            cleaned_map->push_back(point);
            continue;
        }
        
        const TSDFVoxel& voxel = it->second;
        
        // 只保留表面附近的点（TSDF值接近0）
        // 自由空间点（TSDF值接近truncation_distance_）被剔除
        if (std::abs(voxel.tsdf) < truncation_distance_ * 0.5f && 
            voxel.weight >= min_weight_)
        {
            cleaned_map->push_back(point);
        }
    }
    
    global_map_.swap(cleaned_map);
    
    ROS_DEBUG("Removed free space points: %d -> %d", 
              (int)cleaned_map->size() + (int)(global_map_->size() - cleaned_map->size()), 
              (int)global_map_->size());
}

// ==================== 新增：全局地图体素下采样 ====================
void TSDFMapping::downsampleGlobalMap()
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(global_map_);
    voxel_grid.setLeafSize(downsample_resolution_, 
                          downsample_resolution_, 
                          downsample_resolution_);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid.filter(*downsampled_map);
    
    global_map_.swap(downsampled_map);
}

// ==================== 修改：发布完整地图 ====================
void TSDFMapping::publishFullMap()
{
    std::lock_guard<std::mutex> lock(global_map_mutex_);
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*global_map_, cloud_msg);
    cloud_msg.header.frame_id = "camera_init";
    cloud_msg.header.stamp = ros::Time::now();
    
    map_pub_.publish(cloud_msg);
}

// ==================== 修改：原始累积点云下采样（均匀采样版本） ====================
// 功能：先进行体素下采样，然后如果点数超过限制，在整个点云中均匀采样
void TSDFMapping::downsampleRawAccumulatedCloud()
{
    std::lock_guard<std::mutex> lock(raw_accumulated_mutex_);
    
    if (raw_accumulated_cloud_->empty())
    {
        return;
    }
    
    // 第一步：体素下采样（保持你原来的0.3m分辨率）
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(raw_accumulated_cloud_);
    voxel_grid.setLeafSize(raw_accumulated_downsample_res_,
                          raw_accumulated_downsample_res_,
                          raw_accumulated_downsample_res_);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid.filter(*downsampled_cloud);
    
    // 第二步：如果点数仍然超过限制，进行均匀采样
    const int MAX_ACCUMULATED_POINTS = 100000; // 最大50万个点
    if (downsampled_cloud->size() > MAX_ACCUMULATED_POINTS)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr uniform_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        
        // 计算采样步长：确保最终点数不超过MAX_ACCUMULATED_POINTS
        // 使用浮点数步长可以更精确地控制最终点数
        double step = static_cast<double>(downsampled_cloud->size()) / MAX_ACCUMULATED_POINTS;
        
        // 均匀采样：从第0个点开始，每隔step个点取一个
        for (double i = 0.0; i < downsampled_cloud->size(); i += step)
        {
            int idx = static_cast<int>(std::floor(i));
            uniform_cloud->push_back((*downsampled_cloud)[idx]);
        }
        
        raw_accumulated_cloud_.swap(uniform_cloud);
        
        ROS_INFO("Raw accumulated cloud uniformly sampled: %d -> %d points",
                 (int)downsampled_cloud->size(), (int)raw_accumulated_cloud_->size());
    }
    else
    {
        // 点数未超过限制，直接使用体素下采样结果
        raw_accumulated_cloud_.swap(downsampled_cloud);
        
        ROS_DEBUG("Raw accumulated cloud downsampled: %d points", (int)raw_accumulated_cloud_->size());
    }
}

} // namespace Ten
