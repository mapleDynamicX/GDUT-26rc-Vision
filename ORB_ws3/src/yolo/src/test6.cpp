#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>  // 关键修复：添加Eigen-OpenCV互转头文件
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>


// ===================== 数据结构定义 =====================
// 相机参数（内参+外参）
struct CameraParams {
    cv::Mat K;          // 内参矩阵 (3x3) [fx,0,cx; 0,fy,cy; 0,0,1]
    cv::Mat dist_coeffs;// 畸变系数 (1x5/1x8)，无畸变则为空
    cv::Mat R;          // 外参旋转矩阵 (3x3)：世界→相机
    cv::Mat T;          // 外参平移向量 (3x1)：世界→相机
    int img_width;      // 图像宽度
    int img_height;     // 图像高度
};

// 长方体（正方体）定义：8个世界坐标角点 + OBB包围盒（用于遮挡检测）
struct Cube {
    std::vector<Eigen::Vector3d> world_corners; // 8个角点（世界坐标系）
    Eigen::Vector3d center;                     // 中心（OBB用）
    Eigen::Vector3d axes[3];                    // OBB正交轴（世界坐标系）
    Eigen::Vector3d extents;                    // OBB半长度（世界坐标系）
};

// 射线定义（世界坐标系）
struct Ray {
    Eigen::Vector3d origin;   // 起点（相机光心，世界坐标系）
    Eigen::Vector3d direction;// 方向（单位向量，世界坐标系）
};

// ===================== 辅助函数 =====================
/**
 * @brief 生成长方体的8个世界坐标角点
 * @param center 长方体中心（世界坐标系）
 * @param size   长方体尺寸（x/y/z轴长度）
 * @param R      长方体姿态（旋转矩阵，世界坐标系）
 * @return 8个角点
 */
std::vector<Eigen::Vector3d> generateCubeCorners(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& size,
    const Eigen::Matrix3d& R) {
    std::vector<Eigen::Vector3d> corners;
    Eigen::Vector3d half_size = size / 2.0;
    // 8个角点的局部偏移（以中心为原点）
    std::vector<Eigen::Vector3d> offsets = {
        {-half_size.x(), -half_size.y(), -half_size.z()},
        { half_size.x(), -half_size.y(), -half_size.z()},
        { half_size.x(),  half_size.y(), -half_size.z()},
        {-half_size.x(),  half_size.y(), -half_size.z()},
        {-half_size.x(), -half_size.y(),  half_size.z()},
        { half_size.x(), -half_size.y(),  half_size.z()},
        { half_size.x(),  half_size.y(),  half_size.z()},
        {-half_size.x(),  half_size.y(),  half_size.z()}
    };
    // 转换到世界坐标系
    for (const auto& off : offsets) {
        corners.push_back(center + R * off);
    }
    return corners;
}

/**
 * @brief 世界点投影到像素坐标系
 * @param cam_params 相机参数
 * @param world_pt   世界点
 * @param pixel_pt   输出像素点
 * @return 是否投影有效（在图像范围内）
 */
bool projectWorldToPixel(
    const CameraParams& cam_params,
    const Eigen::Vector3d& world_pt,
    cv::Point2d& pixel_pt) {
    // 转换为OpenCV格式
    std::vector<cv::Point3d> obj_pts;
    obj_pts.push_back(cv::Point3d(world_pt.x(), world_pt.y(), world_pt.z()));
    std::vector<cv::Point2d> img_pts;

    // 投影（OpenCV的projectPoints：世界→相机→像素）
    cv::projectPoints(obj_pts, cam_params.R, cam_params.T, 
                      cam_params.K, cam_params.dist_coeffs, img_pts);
    pixel_pt = img_pts[0];

    // 判断是否在图像范围内
    return (pixel_pt.x >= 0 && pixel_pt.x < cam_params.img_width &&
            pixel_pt.y >= 0 && pixel_pt.y < cam_params.img_height);
}

/**
 * @brief Slab算法：射线与OBB相交检测（世界坐标系）
 * @param ray 射线
 * @param cube 待检测的立方体
 * @param t_out 输出相交点的t参数（P = origin + t*direction）
 * @return 是否相交
 */
bool rayIntersectsCubeOBB(
    const Ray& ray,
    const Cube& cube,
    double& t_out) {
    double t_min = 0.0;
    double t_max = std::numeric_limits<double>::infinity();
    Eigen::Vector3d origin_local = ray.origin - cube.center;

    for (int i = 0; i < 3; ++i) {
        const Eigen::Vector3d& axis = cube.axes[i];
        double ext = cube.extents[i];
        double dir_dot = ray.direction.dot(axis);
        double orig_dot = origin_local.dot(axis);

        if (std::fabs(dir_dot) < 1e-8) {
            if (std::fabs(orig_dot) > ext) return false;
            continue;
        }

        double t1 = (ext - orig_dot) / dir_dot;
        double t2 = (-ext - orig_dot) / dir_dot;
        if (t1 > t2) std::swap(t1, t2);

        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        if (t_min > t_max) return false;
    }

    t_out = t_min;
    return t_min > 1e-8; // 排除射线起点（相机光心）的误判
}

/**
 * @brief 检测角点是否被遮挡（自身/其他方块）
 * @param cam_params 相机参数
 * @param cube_list  所有立方体列表
 * @param target_cube_idx 目标立方体索引
 * @param corner_idx      目标角点索引
 * @return 是否被遮挡
 */
bool isCornerOccluded(
    const CameraParams& cam_params,
    const std::vector<Cube>& cube_list,
    int target_cube_idx,
    int corner_idx) {
    // 1. 获取目标角点和相机光心（世界坐标系）
    const Cube& target_cube = cube_list[target_cube_idx];
    const Eigen::Vector3d& corner = target_cube.world_corners[corner_idx];
    // 相机光心：外参T的逆（T是世界→相机的平移，相机光心的世界坐标 = -R^T * T）
    Eigen::Matrix3d R_cam;
    cv2eigen(cam_params.R, R_cam); // 关键修复：去掉cv::命名空间
    Eigen::Vector3d T_cam;
    cv2eigen(cam_params.T, T_cam); // 关键修复：去掉cv::命名空间
    Eigen::Vector3d cam_center = -R_cam.transpose() * T_cam;

    // 2. 生成从相机光心到角点的射线
    Ray ray;
    ray.origin = cam_center;
    ray.direction = (corner - cam_center).normalized();
    double corner_dist = (corner - cam_center).norm(); // 角点到相机的距离

    // 3. 检测是否被其他立方体遮挡
    for (int i = 0; i < cube_list.size(); ++i) {
        if (i == target_cube_idx) continue; // 跳过自身（先检测跨方块遮挡）
        double t_intersect;
        if (rayIntersectsCubeOBB(ray, cube_list[i], t_intersect)) {
            // 若相交点距离 < 角点距离 → 被遮挡
            if (t_intersect < corner_dist - 1e-8) {
                return true;
            }
        }
    }

    // 4. 检测是否被自身遮挡（角点是否在相机视角的“背面”）
    // 计算角点相对于立方体中心的向量
    Eigen::Vector3d corner_rel = corner - target_cube.center;
    // 相机视角方向（立方体中心→相机光心）
    Eigen::Vector3d view_dir = (cam_center - target_cube.center).normalized();
    // 若角点在立方体“背面”（点积<0）→ 自身遮挡
    if (corner_rel.dot(view_dir) < -1e-8) {
        return true;
    }

    return false;
}

/**
 * @brief 筛选所有立方体的有效角点
 * @param cam_params 相机参数
 * @param cube_list  所有立方体列表
 * @return 有效角点（立方体索引，角点索引，世界坐标，像素坐标）
 */
std::vector<std::tuple<int, int, Eigen::Vector3d, cv::Point2d>> filterValidCorners(
    const CameraParams& cam_params,
    const std::vector<Cube>& cube_list) {
    std::vector<std::tuple<int, int, Eigen::Vector3d, cv::Point2d>> valid_corners;

    for (int cube_idx = 0; cube_idx < cube_list.size(); ++cube_idx) {
        const Cube& cube = cube_list[cube_idx];
        for (int corner_idx = 0; corner_idx < cube.world_corners.size(); ++corner_idx) {
            const Eigen::Vector3d& world_corner = cube.world_corners[corner_idx];
            cv::Point2d pixel_corner;

            // 条件1：投影在相机视野内
            if (!projectWorldToPixel(cam_params, world_corner, pixel_corner)) {
                continue;
            }

            // 条件2：无遮挡（自身+其他方块）
            if (isCornerOccluded(cam_params, cube_list, cube_idx, corner_idx)) {
                continue;
            }

            // 所有条件满足，加入有效列表
            valid_corners.emplace_back(cube_idx, corner_idx, world_corner, pixel_corner);
        }
    }

    return valid_corners;
}

// ===================== 主函数（测试示例） =====================
int main(int argc, char** argv) {
    ros::init(argc, argv, "cube_corner_filter_node");
    ros::NodeHandle nh;

    // 1. 定义相机参数（示例值，替换为你的实际参数）
    CameraParams cam_params;
    // 内参：fx=800, fy=800, cx=640, cy=480（1280x960图像）
    cam_params.K = (cv::Mat_<double>(3,3) << 800, 0, 640, 0, 800, 480, 0, 0, 1);
    cam_params.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F); // 无畸变
    // 外参：相机在世界坐标系的(0,0,5)位置，朝向原点（旋转矩阵为单位矩阵）
    cam_params.R = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);;
    cam_params.T = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849); // 世界→相机的平移
    cam_params.img_width = 1280;
    cam_params.img_height = 960;

    // 2. 定义两个立方体（示例）
    std::vector<Cube> cube_list;

    // 立方体1：中心(1,0,0)，尺寸(1,1,1)，无旋转
    Cube cube1;
    cube1.center = Eigen::Vector3d(1.0, 0.0, 0.0);
    cube1.axes[0] = Eigen::Vector3d(1,0,0);
    cube1.axes[1] = Eigen::Vector3d(0,1,0);
    cube1.axes[2] = Eigen::Vector3d(0,0,1);
    cube1.extents = Eigen::Vector3d(0.5, 0.5, 0.5);
    cube1.world_corners = generateCubeCorners(
        cube1.center, Eigen::Vector3d(1,1,1), Eigen::Matrix3d::Identity());
    cube_list.push_back(cube1);

    // 立方体2：中心(3,0,0)，尺寸(1,1,1)，无旋转（在立方体1后方）
    Cube cube2;
    cube2.center = Eigen::Vector3d(1.5, 0.0, 0.0);
    cube2.axes[0] = Eigen::Vector3d(1,0,0);
    cube2.axes[1] = Eigen::Vector3d(0,1,0);
    cube2.axes[2] = Eigen::Vector3d(0,0,1);
    cube2.extents = Eigen::Vector3d(0.5, 0.5, 0.5);
    cube2.world_corners = generateCubeCorners(
        cube2.center, Eigen::Vector3d(1,1,1), Eigen::Matrix3d::Identity());
    cube_list.push_back(cube2);

    // 3. 筛选有效角点
    auto valid_corners = filterValidCorners(cam_params, cube_list);

    // 4. 输出结果
    std::cout << "==================== 有效角点列表 ====================" << std::endl;
    std::cout << "共筛选出 " << valid_corners.size() << " 个有效角点：" << std::endl;
    for (const auto& corner : valid_corners) {
        int cube_idx = std::get<0>(corner);
        int corner_idx = std::get<1>(corner);
        Eigen::Vector3d world_pt = std::get<2>(corner);
        cv::Point2d pixel_pt = std::get<3>(corner);

        std::cout << "立方体" << cube_idx+1 << " 角点" << corner_idx+1 << ":" << std::endl;
        std::cout << "  世界坐标：(" << std::fixed << std::setprecision(2)
                  << world_pt.x() << ", " << world_pt.y() << ", " << world_pt.z() << ")" << std::endl;
        std::cout << "  像素坐标：(" << pixel_pt.x << ", " << pixel_pt.y << ")" << std::endl;
    }

    ros::spinOnce();
    return 0;
}
