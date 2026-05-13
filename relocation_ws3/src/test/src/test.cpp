#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include<ros/ros.h>
#include "relocation.h"
#include "project_test.h"

// // 类型别名简化代码
// using PointT = pcl::PointXYZINormal;
// using PointCloudT = pcl::PointCloud<PointT>;
// using NormalCloudT = pcl::PointCloud<pcl::Normal>;
// using FPFHCloudT = pcl::PointCloud<pcl::FPFHSignature33>;

// /**
//  * @brief 读取PCD点云文件
//  * @param file_path PCD文件路径
//  * @param cloud 输出点云
//  * @return 读取成功返回true
//  */
// bool readPCD(const std::string& file_path, PointCloudT::Ptr& cloud) {
//     if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1) {
//         PCL_ERROR("无法读取PCD文件: %s\n", file_path.c_str());
//         return false;
//     }
//     std::cout << "读取点云 " << file_path << "，点数量: " << cloud->size() << std::endl;
//     return true;
// }

// /**
//  * @brief 点云下采样（体素滤波），减少计算量
//  * @param input_cloud 输入点云
//  * @param output_cloud 输出下采样点云
//  * @param leaf_size 体素大小（单位：m）
//  */
// void voxelDownSample(const PointCloudT::Ptr& input_cloud, 
//                      PointCloudT::Ptr& output_cloud, 
//                      float leaf_size = 0.05) {
//     pcl::VoxelGrid<PointT> vg;
//     vg.setInputCloud(input_cloud);
//     vg.setLeafSize(leaf_size, leaf_size, leaf_size);
//     vg.filter(*output_cloud);
//     std::cout << "下采样后点数量: " << output_cloud->size() << std::endl;
// }

// /**
//  * @brief 提取FPFH特征（用于生成点云对应关系）
//  * @param cloud 输入点云
//  * @param fpfh_features 输出FPFH特征
//  */
// void extractFPFHFeatures(const PointCloudT::Ptr& cloud, FPFHCloudT::Ptr& fpfh_features) {
//     // 1. 计算法向量
//     NormalCloudT::Ptr normals(new NormalCloudT);
//     pcl::NormalEstimation<PointT, pcl::Normal> ne;
//     pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//     ne.setInputCloud(cloud);
//     ne.setSearchMethod(tree);
//     ne.setKSearch(20); // 近邻数，可根据点云密度调整
//     ne.compute(*normals);

//     // 2. 计算FPFH特征
//     pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
//     fpfh.setInputCloud(cloud);
//     fpfh.setInputNormals(normals);
//     fpfh.setSearchMethod(tree);
//     fpfh.setKSearch(50); // 特征计算近邻数
//     fpfh.compute(*fpfh_features);
//     std::cout << "FPFH特征提取完成，特征维度: " << fpfh_features->size() << std::endl;
// }

// /**
//  * @brief 特征匹配生成点对（全局→局部）
//  * @param src_cloud 源点云（局部点云）
//  * @param tgt_cloud 目标点云（全局点云）
//  * @param src_fpfh 源点云FPFH特征
//  * @param tgt_fpfh 目标点云FPFH特征
//  * @param correspondences 输出对应点对（src_idx → tgt_idx）
//  */
// void matchFeatures(const PointCloudT::Ptr& src_cloud,
//                    const PointCloudT::Ptr& tgt_cloud,
//                    const FPFHCloudT::Ptr& src_fpfh,
//                    const FPFHCloudT::Ptr& tgt_fpfh,
//                    pcl::Correspondences& correspondences) {
//     pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> ce;
//     ce.setInputSource(src_fpfh);
//     ce.setInputTarget(tgt_fpfh);
//     ce.determineCorrespondences(correspondences, 0.5); // 距离阈值，可调整
//     std::cout << "特征匹配完成，生成对应点对数量: " << correspondences.size() << std::endl;
// }


// /**
//     @brief 创建旋转矩阵
//     @param rx: roll (弧度)
//     @param ry: pitch (弧度)
//     @param rz: yaw (弧度)
//     @return Eigen::Matrix3f: 3x3的旋转矩阵
// */
// Eigen::Matrix3f createRotationMatrix(float rx, float ry, float rz) {
// // 弧度
// // 创建绕各轴的旋转矩阵
// Eigen::Matrix3f R_x;
// R_x << 1, 0, 0,
//     0, cos(rx), -sin(rx),
//     0, sin(rx), cos(rx);
// Eigen::Matrix3f R_y;
// R_y << cos(ry), 0, sin(ry),
//     0, 1, 0,
//     -sin(ry), 0, cos(ry);
// Eigen::Matrix3f R_z;
// R_z << cos(rz), -sin(rz), 0,
//     sin(rz), cos(rz), 0,
//     0, 0, 1;
// // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
// return R_z * R_y * R_x;
// }
// /**
//     @brief 创建平移矩阵
//     @param tx: x (米)
//     @param ty: y (米)
//     @param tz: z (米)
//     @return Eigen::Vector3f: 1x3平移矩阵
// */
// Eigen::Vector3f createTranslationVector(float tx, float ty, float tz) {
//     Eigen::Vector3f translation(tx, ty, tz);
//     return translation;
// }
// /**
//     @brief 创建旋转矩阵
//     @param rotation: 旋转矩阵
//     @param translation: 平移矩阵
//     @return Eigen::Matrix4f: 4x4的RT矩阵
// */
// //组合现有旋转矩阵与平移向量
// Eigen::Matrix4f combineRotationAndTranslation(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation) {
//     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//     transform.block<3, 3>(0, 0) = rotation;
//     transform.block<3, 1>(0, 3) = translation;
//     return transform;
// }



// int main(int argc, char** argv) {
//     // ===================== 1. 输入参数检查 =====================
//     // if (argc != 3) {
//     //     std::cerr << "用法: " << argv[0] << " <全局点云PCD路径> <局部点云PCD路径>" << std::endl;
//     //     return -1;
//     // }

//     ros::init(argc, argv, "test_node");

//     std::string global_pcd_path = "/home/maple/study2/mapping/map.pcd";
//     std::string local_pcd_path = "/home/maple/study2/mapping/map.pcd";

//     // ===================== 2. 读取点云 =====================
//     PointCloudT::Ptr global_cloud(new PointCloudT);
//     PointCloudT::Ptr local_cloud(new PointCloudT);
//     if (!readPCD(global_pcd_path, global_cloud) || !readPCD(local_pcd_path, local_cloud)) {
//         return -1;
//     }

//     Eigen::Matrix3f rot = createRotationMatrix(20 * M_PI / 180.0, -50 * M_PI / 180.0, 40 * M_PI / 180.0);
//     Eigen::Vector3f tra = createTranslationVector(4, 8, 3);
//     //得到map->cloud变化矩阵
//     Eigen::Matrix4f T = combineRotationAndTranslation(rot, tra);
//     //求t的逆矩阵
//     Eigen::Matrix4f inverse_transform = T.inverse();
//     pcl::transformPointCloud(*global_cloud, *local_cloud, T);
//     std::cout<<"------------------- T -------------------"<< std::endl;
//     std::cout<< T << std::endl;
//     std::cout<<"------------------- inverse_transform -------------------"<< std::endl;
//     std::cout<< inverse_transform << std::endl;

//     // ===================== 3. 点云预处理（下采样） =====================
//     PointCloudT::Ptr global_cloud_downsampled(new PointCloudT);
//     PointCloudT::Ptr local_cloud_downsampled(new PointCloudT);
//     voxelDownSample(global_cloud, global_cloud_downsampled, 0.1); // 体素0.05m，可根据点云密度调整
//     voxelDownSample(local_cloud, local_cloud_downsampled, 0.1);

//     // ===================== 4. 提取FPFH特征 =====================
//     FPFHCloudT::Ptr global_fpfh(new FPFHCloudT);
//     FPFHCloudT::Ptr local_fpfh(new FPFHCloudT);
//     extractFPFHFeatures(global_cloud_downsampled, global_fpfh);
//     extractFPFHFeatures(local_cloud_downsampled, local_fpfh);

//     // ===================== 5. 特征匹配生成对应点对 =====================
//     pcl::Correspondences correspondences;
//     matchFeatures(local_cloud_downsampled, global_cloud_downsampled, local_fpfh, global_fpfh, correspondences);

//     //转换对应点对为TEASER++所需格式（Eigen矩阵）
//     Eigen::Matrix<double, 3, Eigen::Dynamic> src_points(3, correspondences.size());
//     Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_points(3, correspondences.size());
//     for (size_t i = 0; i < correspondences.size(); ++i) {
//         const auto& corr = correspondences[i];
//         // 源点（局部点云）
//         src_points(0, i) = local_cloud_downsampled->points[corr.index_query].x;
//         src_points(1, i) = local_cloud_downsampled->points[corr.index_query].y;
//         src_points(2, i) = local_cloud_downsampled->points[corr.index_query].z;
//         // 目标点（全局点云）
//         tgt_points(0, i) = global_cloud_downsampled->points[corr.index_match].x;
//         tgt_points(1, i) = global_cloud_downsampled->points[corr.index_match].y;
//         tgt_points(2, i) = global_cloud_downsampled->points[corr.index_match].z;
//     }

//     // Eigen::Matrix<double, 3, Eigen::Dynamic> src_points(3, global_cloud_downsampled->points.size());
//     // Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_points(3, local_cloud_downsampled->points.size());
//     // for (size_t i = 0; i < global_cloud_downsampled->points.size(); ++i) {
//     //     // 源点（局部点云）
//     //     src_points(0, i) = global_cloud_downsampled->points[i].x;
//     //     src_points(1, i) = global_cloud_downsampled->points[i].y;
//     //     src_points(2, i) = global_cloud_downsampled->points[i].z;
//     // }
//     // for (size_t i = 0; i < local_cloud_downsampled->points.size(); ++i) {
//     //     tgt_points(0, i) = local_cloud_downsampled->points[i].x;
//     //     tgt_points(1, i) = local_cloud_downsampled->points[i].y;
//     //     tgt_points(2, i) = local_cloud_downsampled->points[i].z;
//     // }



//     // ===================== 6. TEASER++ 配准配置 =====================
//     // teaser::RobustRegistrationSolver::Params params;
//     // params.noise_bound = 0.05;          // 噪声边界（与下采样体素大小匹配）
//     // params.cbar2 = 1.0;                // 正则化参数
//     // params.estimate_scaling = false;   // 刚体变换，不估计尺度
//     // params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
//     // params.rotation_gnc_factor = 1.4;
//     // params.rotation_max_iterations = 100;
//     // params.rotation_cost_threshold = 1e-6;


//   // 7. 配置TEASER++配准求解器参数
//     teaser::RobustRegistrationSolver::Params params;
//     params.noise_bound = 0.1;          // 噪声边界（与添加的噪声一致）
//     params.cbar2 = 1;                          // GNC-TLS的正则化参数
//     params.estimate_scaling = false;           // 不估计尺度（SE(3)变换无尺度）
//     params.rotation_max_iterations = 100;      // 旋转估计最大迭代次数
//     params.rotation_gnc_factor = 1.4;          // GNC-TLS的衰减因子
//     params.rotation_estimation_algorithm =     // 旋转估计算法选择GNC-TLS
//         teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
//     params.rotation_cost_threshold = 0.005;    // 旋转估计的成本阈值

//     // 初始化TEASER++求解器
//     teaser::RobustRegistrationSolver solver(params);
//     solver.solve(src_points, tgt_points);

//     // ===================== 7. 获取配准结果 =====================
//     auto solution = solver.getSolution();
//     Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
//     transform_matrix.block<3, 3>(0, 0) = solution.rotation; // 旋转矩阵
//     transform_matrix.block<3, 1>(0, 3) = solution.translation; // 平移向量

//     std::cout << "\n===================== TEASER++ 配准结果 =====================" << std::endl;
//     std::cout << "旋转矩阵:\n" << solution.rotation << std::endl;
//     std::cout << "平移向量 (x,y,z): " << solution.translation.transpose() << std::endl;
//     std::cout << "变换矩阵:\n" << transform_matrix << std::endl;

//     // ===================== 8. 应用变换到局部点云 =====================
//     // PointCloudT::Ptr local_cloud_registered(new PointCloudT);
//     // pcl::transformPointCloud(*local_cloud, *local_cloud_registered, transform_matrix);

//     // ===================== 9. 保存配准后的点云 =====================
//     // std::string output_path = "local_cloud_registered.pcd";
//     // pcl::io::savePCDFileASCII(output_path, *local_cloud_registered);
//     // std::cout << "\n配准后的局部点云已保存至: " << output_path << std::endl;

//     return 0;
// }


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "test_node");
//     Ten::parameter::loadyaml ly;
//     std::string global_pcd_path = "/home/maple/study3/maple/map/test_map1/3.pcd";
//     std::string local_pcd_path = "/home/maple/study3/maple/map/test_map1/4.pcd";
//     Ten::Ten_relocation<pcl::PointXYZI> rel(global_pcd_path);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>); 
//     pcl::io::loadPCDFile<pcl::PointXYZI>(global_pcd_path, *global_cloud);
//     pcl::io::loadPCDFile<pcl::PointXYZI>(local_pcd_path, *local_cloud);

//     // Ten::XYZ xyz;
//     // xyz._x = 1 ;
//     // xyz._y = 2 ;
//     // xyz._z = 3 ;

//     // Ten::RPY rpy;
//     // rpy._roll = 0.52;
//     // rpy._pitch = 0.52;
//     // rpy._yaw = 0.52;

//     // Eigen::Matrix4d T = Ten::worldtocurrent(xyz, rpy);
//     // pcl::transformPointCloud(*global_cloud, *local_cloud, T);

//     Ten::XYZRPY xyzrpy = rel.get_transformation(local_cloud);

//     std::cout << "---------------------------" << std::endl; 
//     std::cout << "x: " << xyzrpy._xyz._x << std::endl;
//     std::cout << "y: " << xyzrpy._xyz._y << std::endl;
//     std::cout << "z: " << xyzrpy._xyz._z << std::endl;
//     std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
//     std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
//     std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;
//     return 0;
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    Ten::parameter::loadyaml ly;
    std::string global_pcd_path = "/home/maple/study3/maple/map/test_map1/3.pcd";
    std::string local_pcd_path = "/home/maple/study3/maple/map/test_map1/4.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::io::loadPCDFile<pcl::PointXYZI>(global_pcd_path, *global_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZI>(local_pcd_path, *local_cloud);


    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = -0.178874;
    xyzrpy_error._xyz._y = 0.932056;
    xyzrpy_error._xyz._z = -0.00693875;
    xyzrpy_error._rpy._roll = -0.00217228;
    xyzrpy_error._rpy._pitch = -0.00353182;
    xyzrpy_error._rpy._yaw = -0.597487;

    Ten::calculateMatchPointDistanceStats(global_cloud, local_cloud, 0.6, Ten::XYZRPYtotransform_matrix(xyzrpy_error));


    return 0;
}