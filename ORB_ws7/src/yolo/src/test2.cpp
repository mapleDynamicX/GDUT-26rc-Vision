#include <iostream>
#include <vector>
#include <random>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// 全局随机数生成器（std::，避免多次初始化导致采样重复）
static std::mt19937 rng(std::random_device{}());

/**
 * @brief 辅助函数：判断3个3D点是否共线（P3P无解的情况）
 * @param pts3d 3个3D点，std::vector<cv::Point3f>
 * @return true-共线，false-不共线
 */
bool is3DPointsColinear(const std::vector<cv::Point3f>& pts3d) {
    if (pts3d.size() != 3) return true;
    // 计算向量v1=P2-P1, v2=P3-P1（cv::Vec3f）
    cv::Vec3f v1 = pts3d[1] - pts3d[0];
    cv::Vec3f v2 = pts3d[2] - pts3d[0];
    // 叉乘：共线点的叉乘模长接近0（cv::norm计算模长）
    cv::Vec3f cross = v1.cross(v2);
    float cross_norm = cv::norm(cross);
    return cross_norm < 1e-6; // 阈值可根据坐标精度调整
}

/**
 * @brief 辅助函数：cv::Mat转Eigen::Matrix3d（旋转矩阵）
 * @param cv_mat OpenCV旋转矩阵（3x3, CV_32F/CV_64F）
 * @return Eigen::Matrix3d 旋转矩阵
 */
Eigen::Matrix3d cvMat2EigenMat3d(const cv::Mat& cv_mat) {
    Eigen::Matrix3d eigen_mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigen_mat(i, j) = cv_mat.at<double>(i, j);
        }
    }
    return eigen_mat;
}

/**
 * @brief 辅助函数：cv::Mat转Eigen::Vector3d（平移向量）
 * @param cv_mat OpenCV平移向量（3x1, CV_32F/CV_64F）
 * @return Eigen::Vector3d 平移向量
 */
Eigen::Vector3d cvMat2EigenVec3d(const cv::Mat& cv_mat) {
    Eigen::Vector3d eigen_vec;
    eigen_vec(0) = cv_mat.at<double>(0, 0);
    eigen_vec(1) = cv_mat.at<double>(1, 0);
    eigen_vec(2) = cv_mat.at<double>(2, 0);
    return eigen_vec;
}

/**
 * @brief 核心函数：RANSAC+PnP求解相机外参（世界→相机）
 * @param world_pts 3D世界点集（硬编码，非齐次），std::vector<cv::Point3f>
 * @param img_pts 2D图像点集（YOLO识别中心，像素坐标），std::vector<cv::Point2f>
 * @param img_confidences YOLO识别的置信度数组，与img_pts一一对应，std::vector<float>
 * @param K 相机内参矩阵（3x3），cv::Mat(3,3,CV_64F)
 * @param dist_coeffs 相机畸变系数（默认无畸变），cv::Mat(1,5,CV_64F)
 * @param reproj_thresh 重投影误差阈值（像素，核心参数），double
 * @param confidence RANSAC置信度，double
 * @param max_iter 最大迭代次数，int
 * @param conf_thresh YOLO置信度筛选阈值，float
 * @param R 输出：旋转矩阵（世界→相机），Eigen::Matrix3d
 * @param t 输出：平移向量（世界→相机），Eigen::Vector3d
 * @param best_inliers 输出：最优内点对（3D点索引, 2D点索引），std::vector<std::pair<int, int>>
 * @param noise_img_indices 输出：图像点中噪声外点的索引，std::vector<int>
 * @return true-求解成功，false-求解失败（无有效内点）
 */
bool ransacPnP(
    const std::vector<cv::Point3f>& world_pts,
    const std::vector<cv::Point2f>& img_pts,
    const std::vector<float>& img_confidences,
    const cv::Mat& K,
    const cv::Mat& dist_coeffs,
    double reproj_thresh,
    double confidence,
    int max_iter,
    float conf_thresh,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t,
    std::vector<std::pair<int, int>>& best_inliers,
    std::vector<int>& noise_img_indices
) {
    // -------------------------- 步骤1：预处理（核心，减少无效计算） --------------------------
    int N = world_pts.size();  // 3D世界点数量
    int M = img_pts.size();    // 2D图像点原始数量
    if (N < 3 || M < 3) {
        std::cerr << "Error: 3D点数量≥3且2D点数量≥3才能求解P3P！" << std::endl;
        return false;
    }
    if (img_confidences.size() != M) {
        std::cerr << "Error: 图像点数量与置信度数量不匹配！" << std::endl;
        return false;
    }

    // 1.1 初筛：剔除YOLO低置信度图像点（提前减少噪声外点）
    std::vector<cv::Point2f> filtered_img_pts;   // 筛选后的2D点
    std::vector<int> img_ori2filtered;           // 筛选后点→原始点的索引映射
    for (int i = 0; i < M; ++i) {
        if (img_confidences[i] >= conf_thresh) {
            filtered_img_pts.push_back(img_pts[i]);
            img_ori2filtered.push_back(i);
        }
    }
    int M_filtered = filtered_img_pts.size();
    if (M_filtered < 3) {
        std::cerr << "Error: 低置信度筛选后2D点数量<3！" << std::endl;
        return false;
    }

    // 1.2 初始化RANSAC相关变量
    best_inliers.clear();
    noise_img_indices.clear();
    cv::Mat best_rvec, best_tvec;  // OpenCV格式的旋转向量/平移向量
    double w = 0.7;                // 初始内点比例估计（你场景：大部分2D点有对应）
    int iter = 0;                  // 当前迭代次数

    // 随机采样器：均匀分布（0~N-1，0~M_filtered-1）（std::uniform_int_distribution）
    std::uniform_int_distribution<int> world_sampler(0, N - 1);
    std::uniform_int_distribution<int> img_sampler(0, M_filtered - 1);

    // -------------------------- 步骤2：RANSAC核心迭代 --------------------------
    while (iter < max_iter) {
        // 2.1 随机采样：3个不共线的3D世界点 + 3个2D图像点（std::unordered_set去重）
        std::unordered_set<int> world_sampled_indices;
        std::vector<cv::Point3f> world_sample_pts;
        while (world_sampled_indices.size() < 3) {
            world_sampled_indices.insert(world_sampler(rng));
        }
        for (int idx : world_sampled_indices) {
            world_sample_pts.push_back(world_pts[idx]);
        }
        // 过滤共线的3D点（P3P无解，直接跳过本次迭代）
        if (is3DPointsColinear(world_sample_pts)) {
            iter++;
            continue;
        }

        std::unordered_set<int> img_sampled_indices;
        std::vector<cv::Point2f> img_sample_pts;
        while (img_sampled_indices.size() < 3) {
            img_sampled_indices.insert(img_sampler(rng));
        }
        for (int idx : img_sampled_indices) {
            img_sample_pts.push_back(filtered_img_pts[idx]);
        }

        // 2.2 P3P拟合外参：最多返回4组解（rvec/tvec）（cv::solveP3P）
        std::vector<cv::Mat> rvecs, tvecs;
        bool p3p_ok = cv::solveP3P(
            world_sample_pts, img_sample_pts, K, dist_coeffs,
            rvecs, tvecs, cv::SOLVEPNP_P3P  // 强制P3P解法
        );
        if (!p3p_ok || rvecs.empty()) {
            iter++;
            continue;
        }

        // 2.3 遍历P3P所有解，验证内点并统计数量
        for (int s = 0; s < rvecs.size(); ++s) {
            cv::Mat curr_rvec = rvecs[s];
            cv::Mat curr_tvec = tvecs[s];
            // 全量3D世界点投影到图像平面（cv::projectPoints）
            std::vector<cv::Point2f> proj_img_pts;
            cv::projectPoints(
                world_pts, curr_rvec, curr_tvec,
                K, dist_coeffs, proj_img_pts
            );

            // 2.4 重投影误差计算 + 一对一匹配（核心：避免一个2D点匹配多个3D点）
            std::vector<std::pair<int, int>> curr_inliers;  // 当前内点对（3D索引, 筛选后2D索引）
            std::unordered_set<int> matched_img_idx;       // 已匹配的筛选后2D点索引
            // 先将采样的3个点对加入内点（保证最小样本集的内点）
            auto world_idx_iter = world_sampled_indices.begin();
            auto img_idx_iter = img_sampled_indices.begin();
            for (int i = 0; i < 3; ++i) {
                int w_idx = *world_idx_iter++;
                int i_idx = *img_idx_iter++;
                curr_inliers.emplace_back(w_idx, i_idx);
                matched_img_idx.insert(i_idx);
            }
            // 遍历剩余3D点，匹配未被占用的2D点
            for (int w_idx = 0; w_idx < N; ++w_idx) {
                // 跳过已采样的3D点
                if (world_sampled_indices.count(w_idx)) continue;
                double min_error = 1e9;
                int best_i_idx = -1;
                // 遍历所有未匹配的2D点，找重投影误差最小的
                for (int i_idx = 0; i_idx < M_filtered; ++i_idx) {
                    if (matched_img_idx.count(i_idx)) continue;
                    double dx = proj_img_pts[w_idx].x - filtered_img_pts[i_idx].x;
                    double dy = proj_img_pts[w_idx].y - filtered_img_pts[i_idx].y;
                    double error = std::sqrt(dx*dx + dy*dy);  // 欧氏距离（std::sqrt）
                    if (error < min_error) {
                        min_error = error;
                        best_i_idx = i_idx;
                    }
                }
                // 误差小于阈值，标记为内点
                if (min_error < reproj_thresh && best_i_idx != -1) {
                    curr_inliers.emplace_back(w_idx, best_i_idx);
                    matched_img_idx.insert(best_i_idx);
                }
            }

            // 2.5 更新最优模型（内点数量更多则更新）
            if (curr_inliers.size() > best_inliers.size()) {
                best_inliers = curr_inliers;
                best_rvec = curr_rvec;
                best_tvec = curr_tvec;
                // 自适应更新内点比例，减少无效迭代
                w = static_cast<double>(best_inliers.size()) / M_filtered;
                // 自适应更新迭代次数：k = log(1-p)/log(1-w^3)（std::log/std::pow）
                if (w > 0.01) {
                    double new_iter = std::log(1 - confidence) / std::log(1 - std::pow(w, 3));
                    max_iter = std::min(max_iter, static_cast<int>(std::round(new_iter)));
                }
            }
        }
        iter++;
    }

    // -------------------------- 步骤3：最优模型精修（EPnP，必须做！提升精度） --------------------------
    if (best_inliers.size() < 4) {
        std::cerr << "Warning: 有效内点数量<4，无法精修外参，求解失败！" << std::endl;
        return false;
    }
    // 提取最优内点对的3D-2D坐标
    std::vector<cv::Point3f> inlier_world_pts;
    std::vector<cv::Point2f> inlier_img_pts;
    for (auto& pair : best_inliers) {
        inlier_world_pts.push_back(world_pts[pair.first]);
        inlier_img_pts.push_back(filtered_img_pts[pair.second]);
    }
    // EPnP重拟合外参（支持任意n≥4，精度远高于P3P）（cv::solvePnP）
    cv::Mat refine_rvec, refine_tvec;
    bool epnp_ok = cv::solvePnP(
        inlier_world_pts, inlier_img_pts, K, dist_coeffs,
        refine_rvec, refine_tvec, false, cv::SOLVEPNP_EPNP  // false：不使用初始值
    );
    if (epnp_ok) {
        best_rvec = refine_rvec;
        best_tvec = refine_tvec;
    } else {
        std::cerr << "Warning: EPnP精修失败，使用RANSAC原始结果！" << std::endl;
    }

    // -------------------------- 步骤4：结果转换与后处理 --------------------------
    // 4.1 OpenCV旋转向量→旋转矩阵（3x3, CV_64F）（cv::Rodrigues）
    cv::Mat R_cv;
    cv::Rodrigues(best_rvec, R_cv);
    // 4.2 转换为Eigen格式（工程中更常用）
    R = cvMat2EigenMat3d(R_cv);
    t = cvMat2EigenVec3d(best_tvec);
    // 4.3 转换内点对的2D索引为**原始图像点索引**（因之前做了置信度筛选）
    for (auto& pair : best_inliers) {
        pair.second = img_ori2filtered[pair.second];
    }
    // 4.4 提取噪声外点（原始图像点索引）
    std::unordered_set<int> matched_ori_img_idx;
    for (auto& pair : best_inliers) {
        matched_ori_img_idx.insert(pair.second);
    }
    for (int i = 0; i < M; ++i) {
        if (!matched_ori_img_idx.count(i)) {
            noise_img_indices.push_back(i);
        }
    }

    std::cout << "RANSAC+PnP求解成功！" << std::endl;
    std::cout << "有效内点对数量：" << best_inliers.size() << std::endl;
    std::cout << "YOLO噪声外点数量：" << noise_img_indices.size() << std::endl;
    std::cout << "重投影误差阈值：" << reproj_thresh << "像素" << std::endl;
    return true;
}

// -------------------------- 测试主函数（替换你的数据即可运行） --------------------------
int main() {
    // 1. 相机内参（替换为你的实际内参！3x3, CV_64F）
    // 示例：fx=800, fy=800, cx=320, cy=240（640x480分辨率相机）
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        800.0, 0.0, 320.0,
        0.0, 800.0, 240.0,
        0.0, 0.0, 1.0
    );
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);  // 无畸变（有畸变则替换为你的系数）

    // 2. 硬编码3D世界点（替换为你的实际世界点！单位：m/mm，保持统一）
    std::vector<cv::Point3f> world_pts = {
        cv::Point3f(0.0, 0.0, 0.0),    // 世界点0
        cv::Point3f(1.0, 0.0, 0.0),    // 世界点1
        cv::Point3f(0.0, 1.0, 0.0),    // 世界点2
        cv::Point3f(0.0, 0.0, 1.0),    // 世界点3
        cv::Point3f(1.0, 1.0, 0.0),    // 世界点4
        cv::Point3f(1.0, 0.0, 1.0),    // 世界点5
        cv::Point3f(0.0, 1.0, 1.0),    // 世界点6
        cv::Point3f(1.0, 1.0, 1.0)     // 世界点7
    };

    // 3. YOLO识别的2D图像点（替换为你的实际识别结果！像素坐标）
    std::vector<cv::Point2f> img_pts = {
        cv::Point2f(320, 240),   // 对应世界点0（有效）
        cv::Point2f(400, 240),   // 对应世界点1（有效）
        cv::Point2f(320, 320),   // 对应世界点2（有效）
        cv::Point2f(320, 160),   // 对应世界点3（有效）
        cv::Point2f(400, 320),   // 对应世界点4（有效）
        cv::Point2f(500, 500),   // 噪声点1（YOLO误识别）
        cv::Point2f(100, 100),   // 噪声点2（YOLO误识别）
        cv::Point2f(400, 160),    // 对应世界点5（有效）
    };

    // 4. YOLO识别的置信度（替换为你的实际置信度！与img_pts一一对应）
    std::vector<float> img_confidences = {
        0.95, 0.92, 0.90, 0.88, 0.85,
        0.40, 0.35, 0.82,
    };

    // 5. RANSAC+PnP参数配置（根据你的场景调参，参考之前的参数指南）
    double reproj_thresh = 3.0;    // 重投影误差阈值（像素）
    double confidence = 0.99;      // RANSAC置信度
    int max_iter = 1000;           // 最大迭代次数
    float conf_thresh = 0.5;       // YOLO置信度筛选阈值

    // 6. 输出变量
    Eigen::Matrix3d R;                    // 旋转矩阵（世界→相机）
    Eigen::Vector3d t;                    // 平移向量（世界→相机）
    std::vector<std::pair<int, int>> best_inliers;  // 最优内点对（3D索引, 2D原始索引）
    std::vector<int> noise_img_indices;        // 噪声外点的原始2D索引

    // 7. 调用核心函数求解
    bool success = ransacPnP(
        world_pts, img_pts, img_confidences,
        K, dist_coeffs, reproj_thresh,
        confidence, max_iter, conf_thresh,
        R, t, best_inliers, noise_img_indices
    );

    // 8. 打印结果（工程中可根据需求保存/传输）
    if (success) {
        std::cout << "-------------------------- 外参结果 --------------------------" << std::endl;
        std::cout << "旋转矩阵R（世界→相机）：" << std::endl << R << std::endl;
        std::cout << "平移向量t（世界→相机，单位与世界点一致）：" << std::endl << t << std::endl;

        std::cout << "-------------------------- 内点匹配关系 --------------------------" << std::endl;
        for (auto& pair : best_inliers) {
            std::cout << "世界点" << pair.first << " → 图像点" << pair.second << std::endl;
        }

        std::cout << "-------------------------- YOLO噪声外点 --------------------------" << std::endl;
        for (int idx : noise_img_indices) {
            std::cout << "图像点" << idx << "（置信度：" << img_confidences[idx] << "）" << std::endl;
        }
    } else {
        std::cerr << "RANSAC+PnP求解失败！" << std::endl;
        return -1;
    }

    return 0;
}
