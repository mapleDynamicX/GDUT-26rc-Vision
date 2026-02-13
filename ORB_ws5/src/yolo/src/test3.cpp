// ROS1核心头文件
#include <ros/ros.h>
// OpenCV核心+相机标定头文件
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// 标准容器+输入输出+格式控制头文件
#include <vector>
#include <iostream>
#include <iomanip>  // 用于std::fixed/std::setprecision控制浮点打印精度

/**
 * @brief 鲁棒求解相机外参（旋转矩阵R+平移向量T），基于OpenCV封装的RANSAC+solvePnP
 * @param world_pts 输入：3D世界点集（一一对应2D图像点，float类型）
 * @param image_pts 输入：2D图像点集（一一对应3D世界点，float类型）
 * @param K         输入：相机内参矩阵（3×3，CV_32F类型，需提前标定）
 * @param dist_coeffs 输入：相机畸变系数（1×5/1×8，CV_32F类型，无畸变传全0矩阵即可）
 * @param R         输出：3×3旋转矩阵（cv::Mat，CV_32F类型）
 * @param T         输出：3×1平移向量（cv::Mat，CV_32F类型）
 * @param reproj_err_thresh 可选：重投影误差阈值（像素），默认3.0（普通相机2~5，工业相机1~2）
 * @param confidence 可选：RANSAC置信度，默认0.99（99%概率找到正确模型）
 * @param max_iter  可选：RANSAC最大迭代次数，默认2000
 * @return bool     求解成功返回true，失败返回false
 */
bool solvePnPRobust(const std::vector<cv::Point3f>& world_pts,
                    const std::vector<cv::Point2f>& image_pts,
                    const cv::Mat& K,
                    const cv::Mat& dist_coeffs,
                    cv::Mat& R,
                    cv::Mat& T,
                    float reproj_err_thresh = 3.0f,
                    double confidence = 0.99,
                    int max_iter = 2000)
{
    // ************************ 步骤1：严格输入校验 ************************
    // 点对数量至少4个（SOLVEPNP_IPPE最小4点，避免3点共线导致解退化）
    if (world_pts.size() < 4 || image_pts.size() < 4)
    {
        std::cout << "[ERROR] PnP求解失败：点对数量不足，至少需要4个！当前3D点：" 
                  << world_pts.size() << "个，2D点：" << image_pts.size() << "个" << std::endl;
        return false;
    }
    // 3D点和2D点必须一一对应，数量一致
    if (world_pts.size() != image_pts.size())
    {
        std::cout << "[ERROR] PnP求解失败：3D点与2D点数量不匹配！3D点：" 
                  << world_pts.size() << "个，2D点：" << image_pts.size() << "个" << std::endl;
        return false;
    }
    // 内参矩阵必须是3×3的CV_32F类型
    if (K.rows != 3 || K.cols != 3 || K.type() != CV_32F)
    {
        std::cout << "[ERROR] PnP求解失败：内参矩阵格式错误！要求3×3的CV_32F类型，当前行：" 
                  << K.rows << "，列：" << K.cols << "，类型：" << K.type() << std::endl;
        return false;
    }
    // 畸变系数至少为1行，且为float类型（支持5参数/8参数畸变）
    if (!dist_coeffs.empty() && (dist_coeffs.rows < 1 || (dist_coeffs.type() != CV_32F && dist_coeffs.type() != CV_32FC1)))
    {
        std::cout << "[WARN] 畸变系数格式非CV_32F，将自动转换为CV_32F类型" << std::endl;
    }

    // ************************ 步骤2：初始化输出变量 ************************
    cv::Mat rvec; // 旋转向量（3×1），solvePnP先求旋转向量，再转旋转矩阵
    cv::Mat tvec; // 平移向量（3×1），直接输出为T
    R.release();  // 清空旋转矩阵，避免残留数据
    T.release();  // 清空平移向量，避免残留数据

    // ************************ 步骤3：RANSAC+PnP核心求解 ************************
    // 调用OpenCV封装的RANSAC版solvePnP，自动剔除外点（无效点对）
    // 求解方法：SOLVEPNP_IPPE（4点求解，速度快、鲁棒性高，工业级首选）
    bool solve_flag = cv::solvePnPRansac(world_pts,
                                         image_pts,
                                         K,
                                         dist_coeffs,
                                         rvec,
                                         tvec,
                                         false,       // 不使用初始外参猜测
                                         max_iter,    // RANSAC最大迭代次数
                                         reproj_err_thresh, // 重投影误差阈值（核心筛选参数）
                                         confidence,  // RANSAC置信度
                                         cv::noArray(), // 若需要内点索引，可传std::vector<int>
                                         cv::SOLVEPNP_IPPE);

    // ************************ 步骤4：结果转换与赋值 ************************
    if (!solve_flag)
    {
        std::cout << "[ERROR] PnP求解失败：点对退化（如共线/共面）或无效点对占比过高！" << std::endl;
        return false;
    }

    // 旋转向量 → 3×3旋转矩阵（cv::Rodrigues为OpenCV旋转向量/矩阵互转核心函数）
    cv::Rodrigues(rvec, R);
    // 平移向量直接克隆赋值（tvec本身为3×1 CV_32F，无需转换）
    T = tvec.clone();

    // ************************ 步骤5：打印求解成功日志 ************************
    std::cout << std::fixed << std::setprecision(1); // 浮点精度1位
    std::cout << "[INFO] PnP求解成功！重投影误差阈值：" << reproj_err_thresh 
              << "像素，RANSAC置信度：" << std::setprecision(2) << confidence << std::endl;
    
    std::cout << "[INFO] 求解得到3×3旋转矩阵R：" << std::endl;
    std::cout << std::fixed << std::setprecision(3); // 浮点精度3位，与原日志一致
    std::cout << R.at<float>(0,0) << "\t" << R.at<float>(0,1) << "\t" << R.at<float>(0,2) << std::endl;
    std::cout << R.at<float>(1,0) << "\t" << R.at<float>(1,1) << "\t" << R.at<float>(1,2) << std::endl;
    std::cout << R.at<float>(2,0) << "\t" << R.at<float>(2,1) << "\t" << R.at<float>(2,2) << std::endl;

    std::cout << "[INFO] 求解得到3×1平移向量T（单位：米）：" << std::endl;
    std::cout << T.at<float>(0,0) << std::endl;
    std::cout << T.at<float>(1,0) << std::endl;
    std::cout << T.at<float>(2,0) << std::endl;

    return true;
}

// ************************ ROS测试节点主函数 ************************
int main(int argc, char** argv)
{
    // 1. 初始化ROS节点
    ros::init(argc, argv, "pnp_ransac_solver_node");
    ros::NodeHandle nh;
    std::cout << "[INFO] PnP鲁棒求解节点已启动！" << std::endl;

    // 2. 设置相机内参和畸变系数【替换为你自己的标定值！】
    // 内参矩阵K：3×3 CV_32F，格式[fx,0,cx; 0,fy,cy; 0,0,1]
    cv::Mat K = (cv::Mat_<float>(3, 3) << 800.0f, 0.0f, 320.0f,
                                           0.0f, 800.0f, 240.0f,
                                           0.0f, 0.0f, 1.0f);
    // 畸变系数：1×5 CV_32F，无畸变则全0（k1,k2,p1,p2,k3）
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_32F);

    // 3. 构造测试点对【替换为你的实际点对！】
    // 包含4个有效点对 + 1个无效点对（模拟噪声/错误匹配，RANSAC会自动剔除）
    std::vector<cv::Point3f> world_pts; // 3D世界点（单位：米）
    std::vector<cv::Point2f> image_pts; // 2D图像点（单位：像素）
    // 有效点对（平面正方形，符合投影规律）
    world_pts.emplace_back(0.0f, 0.0f, 0.0f); image_pts.emplace_back(320.0f, 240.0f);
    world_pts.emplace_back(0.5f, 0.0f, 0.0f); image_pts.emplace_back(400.0f, 240.0f);
    world_pts.emplace_back(0.5f, 0.5f, 0.0f); image_pts.emplace_back(400.0f, 320.0f);
    world_pts.emplace_back(0.0f, 0.5f, 0.0f); image_pts.emplace_back(320.0f, 320.0f);
    // 无效点对（明显偏离，RANSAC会标记为外点并剔除）
    world_pts.emplace_back(10.0f, 10.0f, 10.0f); image_pts.emplace_back(10.0f, 10.0f);

    // 4. 调用PnP求解函数
    cv::Mat R, T; // 输出的旋转矩阵和平移向量
    bool solve_success = solvePnPRobust(world_pts, image_pts, K, dist_coeffs, R, T);


    // 6. ROS节点自旋（保持节点运行）
    ros::spin();

    return 0;
}
