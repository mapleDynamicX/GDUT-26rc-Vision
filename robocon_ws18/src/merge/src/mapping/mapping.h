#ifndef __MAPPING_H_
#define __MAPPING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <deque>

namespace Ten
{

// 体素键值：用于哈希表索引
struct VoxelKey
{
    int x, y, z;

    VoxelKey() : x(0), y(0), z(0) {}
    VoxelKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}

    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    // 添加这行：重载!=运算符
    bool operator!=(const VoxelKey& other) const
    {
        return !(*this == other);
    }
};


struct VoxelKeyHash
{
    size_t operator()(const VoxelKey& key) const
    {
        // 使用boost库的组合哈希算法，冲突概率极低
        size_t seed = 0;
        seed ^= std::hash<int>()(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};


// TSDF体素结构
struct TSDFVoxel
{
    float tsdf;    // 截断符号距离值 [-truncation_distance, truncation_distance]
    float weight;  // 权重值 [0, max_weight]

    TSDFVoxel() : tsdf(0.0f), weight(0.0f) {}
    TSDFVoxel(float tsdf_, float weight_) : tsdf(tsdf_), weight(weight_) {}
};

class TSDFMapping
{
public:
    // 构造函数
    TSDFMapping();

    // 析构函数
    ~TSDFMapping() = default;

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher map_pub_;

    // 最新位姿（线程安全）
    Eigen::Isometry3d latest_pose_;
    std::mutex pose_mutex_;
    std::atomic<bool> has_pose_{false};

    // TSDF地图
    std::unordered_map<VoxelKey, TSDFVoxel, VoxelKeyHash> tsdf_map_;
    std::mutex map_mutex_;

    // ==================== 新增：滑动窗口与全局地图 ====================
    // 全局点云地图（保存所有静态物体）
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
    std::mutex global_map_mutex_;

    // 滑动窗口：保存最新N帧点云（用于动态物体过滤和增量更新）
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> sliding_window_;
    int sliding_window_size_;  // 滑动窗口大小（默认10帧）

    // 下采样参数
    float downsample_resolution_;  // 全局地图下采样分辨率
    // ================================================================

    // ==================== 新增：独立累积点云 ====================
    // 累积所有订阅到的原始点云（与TSDF建图功能完全独立）
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_accumulated_cloud_;
    std::mutex raw_accumulated_mutex_;
    float raw_accumulated_downsample_res_; // 原始累积点云下采样分辨率
    // ================================================================

    // 配置参数
    float voxel_resolution_;    // 体素分辨率 (m)
    float truncation_distance_; // 截断距离 (m)
    float max_weight_;          // 最大权重
    float min_weight_;          // 最小权重（低于此值的体素会被删除）
    float decay_factor_;        // 衰减因子（用于物体消失时的权重衰减）
    float change_threshold_;    // 变化检测阈值 (m)
    float dynamic_threshold_;   // 动态点检测阈值（TSDF值差异）

    // ==================== 新增：XYZ三维范围滤波参数 ====================
    // 只有在这个立方体内的点才会被处理
    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;
    float min_z_;
    float max_z_;
    // ================================================================

    // 发布完整地图
    void publishFullMap();
    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

    // 核心TSDF融合函数
    void integratePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                            const Eigen::Isometry3d& pose);

    // 光线投射算法（Amanatides-Woo）
    void rayCast(const Eigen::Vector3d& origin, const Eigen::Vector3d& point,
                std::vector<VoxelKey>& visible_voxels, VoxelKey& surface_voxel);

    // 世界坐标转体素坐标
    VoxelKey worldToVoxel(const Eigen::Vector3d& world_point) const;

    // 体素坐标转世界坐标
    Eigen::Vector3d voxelToWorld(const VoxelKey& voxel_key) const;

    // 从TSDF地图提取表面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractSurfaceCloud() ;

    // 地图清理：移除低权重体素
    void cleanMap();

    // ==================== 新增：滑动窗口与动态过滤函数 ====================
    // 基于TSDF过滤动态点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterDynamicPoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    // 合并滑动窗口点云到全局地图
    void mergeSlidingWindowToGlobalMap();

    // 基于TSDF剔除全局地图中的自由空间点
    void removeFreeSpacePointsFromGlobalMap();

    // 对全局地图进行体素下采样
    void downsampleGlobalMap();
    // ================================================================

    // ==================== 新增：原始累积点云下采样 ====================
    void downsampleRawAccumulatedCloud();
    // ================================================================
};

} // namespace Ten

#endif // __MAPPING_H_
