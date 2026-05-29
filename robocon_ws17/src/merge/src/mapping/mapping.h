#ifndef __MAPPING_H_
#define __MAPPING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <mutex>
#include <vector>

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

    // 配置参数
    float voxel_resolution_;    // 体素分辨率 (m)
    float truncation_distance_; // 截断距离 (m)
    float max_weight_;          // 最大权重
    float min_weight_;          // 最小权重（低于此值的体素会被删除）
    float decay_factor_;        // 衰减因子（用于物体消失时的权重衰减）
    float change_threshold_;    // 变化检测阈值 (m)
    float max_range_;           // 最大处理距离 (m)
    float min_range_;           // 最小处理距离 (m)

    // 发布调试地图
    void publishDebugMap();
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
};

} // namespace Ten

#endif // __MAPPING_H_
