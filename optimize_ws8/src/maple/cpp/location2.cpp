#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<iostream>
#include<thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>  // 体素滤波类
#include <pcl/common/transforms.h> // 包含变换函数
#include <pcl/segmentation/sac_segmentation.h>  // RANSAC平面分割
#include <pcl/filters/extract_indices.h>  // 提取地面/非地面点
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>          // ICP配准类
//std::mutex mtx;
//std::lock_guard<std::mutex> lock(mtx);

// struct
// {
//     std::mutex mtx_lidar;
// }global;


struct Pointlidar
{
    double _x = 0;
    double _y = 0;
    double _z = 0;
    double _roll = 0;
    double _pitch = 0;
    double _yaw = 0;
};

class lidar
{
public:
    lidar()
    :_nh("/"),
    _cloud_box(new pcl::PointCloud<pcl::PointXYZI>),
    _cloud_current(new pcl::PointCloud<pcl::PointXYZI>),
    _cloud_boxs(new pcl::PointCloud<pcl::PointXYZI>)
    {
        _livox_sub = _nh.subscribe("/livox/lidar", 10, &lidar::livoxCallback, this);
        _current_pub = _nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_current", 10);
        _map_pub = _nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_map", 10);
        _box_pub = _nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_box", 10);
        //_workers.emplace_back(lidar::worker_task1, this, _nh);
        _workers.emplace_back(std::bind(&lidar::worker_task1, this, _nh));
        _cloud_box->header.frame_id = "lidar";
        _cloud_boxs->header.frame_id = "lidar";
        //_cloud_box->points.resize(20000);
        //_cloud_boxs->points.resize(20000);
    }
    void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
    {
        ROS_INFO("Received a frame with %u points", msg->point_num);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->header.frame_id = "lidar"; // 设置坐标系
        cloud->width = msg->point_num;   // 点云宽度
        cloud->height = 1;              // 点云高度（1 表示无序点云）
        cloud->is_dense = false;
        cloud->points.resize(msg->point_num);
        //获取数据
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        double x = 0;
        double y = 0;
        double z = 0;
        {
            std::lock_guard<std::mutex> lock(_mtx_lidar);
            x = _point._x;
            y = _point._y;
            z = _point._z;
            roll = _point._roll;
            pitch = _point._pitch;
            yaw = _point._yaw;
        }
        // 遍历所有点
        for (int i = 0; i < msg->point_num; i++)
        {
            auto point = msg->points[i];
            cloud->points[i].x = point.x - 0.28*cos(yaw) + 0.28;
            cloud->points[i].y = point.y - 0.28*sin(yaw);
            cloud->points[i].z = point.z;
            cloud->points[i].intensity = (float)point.reflectivity; // 强度值 0~255
        }
        std::cout<< "遍历所有点" << std::endl;

        // 存储保留点的索引
        std::vector<int> indices;
        // 调用函数移除 NaN 点
        pcl::removeNaNFromPointCloud(*cloud, *_cloud_current, indices);
        std::cout<< "调用函数移除 NaN 点" << std::endl;

        //当前到世界
        //设置旋转和平移
        //{
            //std::lock_guard<std::mutex> lock(_mtx_lidar);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_T(new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Matrix3f rot = createRotationMatrix(-roll, -pitch, -yaw);
        Eigen::Vector3f tra = -(rot*createTranslationVector(x, y, z));
        //Eigen::Vector3f tra = createTranslationVector(0,0,0);
        //创建1000个测试点云
        //得到map->cloud变化矩阵
        Eigen::Matrix4f T = combineRotationAndTranslation(rot, tra);
        //求t的逆矩阵
        Eigen::Matrix4f inverse_transform = T.inverse();
        pcl::transformPointCloud(*_cloud_current, *cloud_T,  inverse_transform);
        *_cloud_current = *cloud_T;
        //}
        std::cout<< "当前到世界" << std::endl;

        registering();

        if(_cloud_map.size() >= 10)
        {
            _cloud_map.erase(_cloud_map.begin());
        }
        _cloud_map.push_back(_cloud_current);
        pubcloud();
    }

    // 回调函数：处理接收到的 TF 消息
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
    {
        for (const auto& transform : msg->transforms)
        {
            if(transform.child_frame_id != "body")
            {
                continue;
            }
            // 获取四元数
            geometry_msgs::Quaternion quat = transform.transform.rotation;

            // 使用 tf2 库将四元数转换为欧拉角
            tf2::Quaternion tf_quat;
            tf_quat.setX(quat.x);
            tf_quat.setY(quat.y);
            tf_quat.setZ(quat.z);
            tf_quat.setW(quat.w);

            // 转换为旋转矩阵，然后提取欧拉角
            tf2::Matrix3x3 mat(tf_quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);  
            double yaw_deg = yaw * 180.0 / M_PI;
            std::lock_guard<std::mutex> lock(_mtx_lidar);
            _point._x  = transform.transform.translation.x - 0.28*cos(yaw) + 0.28;
            _point._y = transform.transform.translation.y - 0.28*sin(yaw);
            _point._z = transform.transform.translation.z;
            _point._roll = roll;
            _point._pitch = pitch;
            _point._yaw = yaw;
            std::cout<<"x: "<<_point._x<<" y: "<<_point._y<<" yaw_deg: "<<yaw_deg<<std::endl;
        }
    }


private:
// RANSAC平面检测：分离地面与非地面
void ransacGroundFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    // 1. 初始化RANSAC分割器
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  // 地面点索引
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());  // 平面方程系数

    seg.setOptimizeCoefficients(true);  // 优化平面系数
    seg.setModelType(pcl::SACMODEL_PLANE);  // 平面模型
    seg.setMethodType(pcl::SAC_RANSAC);     // RANSAC算法
    seg.setMaxIterations(30);  // 最大迭代次数
    seg.setDistanceThreshold(0.05);  // 距离阈值（小于此值为地面）

    // 2. 执行分割，得到地面点索引
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("No ground plane found.");
        *output = *input;  // 若未检测到地面，全部视为非地面
        return;
    }

    // 3. 提取地面点和非地面点
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);

    // // 提取地面点
    // extract.setNegative(false);  // 保留inliers（地面）
    // extract.filter(*ground);

    //提取非地面点
    extract.setNegative(true);  // 保留非inliers（非地面）
    extract.filter(*output);
}

void statisticalFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(input);
    sor.setMeanK(10);  // 考虑的邻域点数
    sor.setStddevMulThresh(2.0);  // 标准差倍数阈值
    sor.filter(*output);
}

void filter()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered0(new pcl::PointCloud<pcl::PointXYZI>); 
    float x_min = 2.9, x_max = 7.7;
    float y_min = -0.3, y_max = 3.3;
    float z_min = -0.67, z_max = -0.3; //-0.7

    for (auto &p : _cloud_current->points)
    {
        if (p.x >= x_min && p.x <= x_max &&
            p.y >= y_min && p.y <= y_max &&
            p.z >= z_min && p.z <= z_max) 
        {
            //_cloud_box->points.push_back(p);
            cloud_filtered0->points.push_back(p);
        }
    }
    //降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);  // 滤波后点云
    // 4. 创建体素滤波对象（模板参数为PointXYZI，支持强度信息处理
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_filtered0);  // 设置输入点云
    // 设置体素大小（x, y, z方向，单位：米，根据场景调整）
    sor.setLeafSize(0.003f, 0.003f, 0.003f);  
    // 5. 执行滤波（会保留强度信息，每个体素的强度通常为内部点的平均值）
    sor.filter(*cloud_filtered);
    *_cloud_box = *cloud_filtered;
    std::cout<< "降采样" << std::endl;

    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
    //ransacGroundFilter(cloud_filtered, cloud_filtered2);
    //statisticalFilter(cloud_filtered2, _cloud_box);
    //点云大小小于等于20000
    if(_cloud_box->points.size() > 10000)
    {
        _cloud_box->resize(10000);
    }
}

double icpRegistration() {
    if(_cloud_boxs->points.size() <= 30 || _cloud_box->points.size() <= 30)
    {
        return 0;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr ouputicp(new pcl::PointCloud<pcl::PointXYZI>);
    // 初始化ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    // 设置输入点云
    icp.setInputSource(_cloud_box);    // 源点云（待配准）
    icp.setInputTarget(_cloud_boxs);    // 目标点云（基准）

    // 配置ICP参数
    icp.setMaxCorrespondenceDistance(0.5);  // 最大对应点距离（超过此值视为无效匹配，根据点云尺度调整）
    icp.setMaximumIterations(30);     // 最大迭代次数
    icp.setTransformationEpsilon(1e-5);  // 收敛阈值（变换矩阵更新量小于此值则停止）
    icp.setEuclideanFitnessEpsilon(1e-6);   // 均方根误差阈值（小于此值则停止）

    // 执行配准
    icp.align(*ouputicp);
    *_cloud_box = *ouputicp;

    // 获取变换矩阵
    //Eigen::Matrix4f transformation = icp.getFinalTransformation();

    // 返回配准误差（均方根误差）
    return icp.getFitnessScore();
}

void registering()
{
    filter();
    double error = icpRegistration();
    std::cout<< "icpRegistration() error = "<< error <<std::endl;
    *_cloud_boxs += *_cloud_box;
    //降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);  // 滤波后点云
    // 4. 创建体素滤波对象（模板参数为PointXYZI，支持强度信息处理
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(_cloud_boxs);  // 设置输入点云
    // 设置体素大小（x, y, z方向，单位：米，根据场景调整）
    sor.setLeafSize(0.005f, 0.005f, 0.005f);  
    // 5. 执行滤波（会保留强度信息，每个体素的强度通常为内部点的平均值）
    sor.filter(*cloud_filtered);
    *_cloud_boxs = *cloud_filtered;
    std::cout<< "降采样2" << std::endl;
    if(_cloud_boxs->points.size() > 7000)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
        *cloud_filtered2 = *_cloud_boxs;
        //ransacGroundFilter(_cloud_boxs, cloud_filtered2);
        statisticalFilter(cloud_filtered2, _cloud_boxs);
        if(_cloud_boxs->points.size() > 20000)
        _cloud_boxs->resize(20000);
    }

    
}



Eigen::Matrix3f createRotationMatrix(float rx, float ry, float rz) {
    // 转换为弧度
    // rx = rx * M_PI / 180.0f; // Roll (绕X轴)
    // ry = ry * M_PI / 180.0f; // Pitch (绕Y轴)
    // rz = rz * M_PI / 180.0f; // Yaw (绕Z轴)
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
void worker_task1(ros::NodeHandle nh)
{
    ros::Rate sl(1000);
    ros::Subscriber tf_sub = nh.subscribe("/tf", 2, &lidar::tfCallback, this);
    while (ros::ok())
    {
        /* code */
        ros::spinOnce();
        sl.sleep();
    }
    
}
void pubcloud()
{
    sensor_msgs::PointCloud2 current_msg;
    sensor_msgs::PointCloud2 map_msg;
    sensor_msgs::PointCloud2 box_msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_map->header.frame_id = "lidar";
    for(int i = 0; i < _cloud_map.size(); i++)
    {
        *cloud_map += *(_cloud_map[i]); 
    }
    pcl::toROSMsg(*_cloud_current, current_msg);
    pcl::toROSMsg(*cloud_map, map_msg);
    pcl::toROSMsg(*_cloud_boxs, box_msg);
    current_msg.header.stamp = ros::Time::now();
    map_msg.header.stamp = ros::Time::now();

    _current_pub.publish(current_msg);
    _map_pub.publish(map_msg);
    _box_pub.publish(box_msg);
}

ros::NodeHandle _nh;
ros::Subscriber _livox_sub;
ros::Publisher _map_pub;
ros::Publisher _box_pub;
ros::Publisher _current_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_current;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _cloud_map;
pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_box;
pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_boxs;
std::vector<std::thread> _workers;
std::mutex _mtx_lidar;
Pointlidar _point;

};


// // 回调函数
// void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg)
// {
//     ROS_INFO("Received a frame with %u points", msg->point_num);

//     // 遍历所有点
//     for (const auto& point : msg->points)
//     {
//         // // 计算绝对时间戳（ns）
//         // uint64_t absolute_time = msg->timebase + point.offset_time;

//         // // 打印部分信息（示例）
//         // ROS_DEBUG_STREAM_THROTTLE(1, "Point: x=" << point.x
//         //                          << ", y=" << point.y
//         //                          << ", z=" << point.z
//         //                          << ", reflectivity=" << static_cast<int>(point.reflectivity)
//         //                          << ", line=" << static_cast<int>(point.line)
//         //                          << ", time=" << absolute_time);
//     }
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "location_node");

    // ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("/livox/lidar", 1000, livoxCallback);
    // ros::spin();
    lidar lidar_1;
    ros::Rate sleep(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        sleep.sleep();
    }

    return 0;
}
