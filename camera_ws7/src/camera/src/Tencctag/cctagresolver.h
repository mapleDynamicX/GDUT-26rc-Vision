#ifndef __CCTAGRESOLVER_H_
#define __CCTAGRESOLVER_H_
#include "cctagbase.h"

namespace Ten
{

    namespace Tencctag
    {

        #define _IDX_0_ 0
        #define _IDX_1_ 1
        #define _distance_1_2_ 0.041


        struct cctagresult
        {
            // 默认构造函数：创建实例时自动初始化所有成员
            cctagresult()
            {
                // 1. 初始化旋转向量：3行1列，64位浮点型，全0
                rvec_ = cv::Mat::zeros(3, 1, CV_64F);
                // 2. 初始化平移向量：3行1列，64位浮点型，全0
                tvec_ = cv::Mat::zeros(3, 1, CV_64F);
            }
            cv::Mat rvec_;
            cv::Mat tvec_;
            double x_ = 0.0;
            double y_ = 0.0;
            int id_ = -1; //识别到的id号
            int flag_ = 0;//rt转换是否成功
        };


        class cctagresolver
        {
        public:

            /**
             * @param real_outer_radius: 圆半径
             * @param nCrowns: 指定环数
             */
            cctagresolver(double real_outer_radius = 0.1, double scale = 1.0, int nCrowns = 3)
            :cb_(nCrowns)
            ,real_outer_radius_(real_outer_radius)
            ,scale_(scale)
            {
                //K_ = (cv::Mat_<double>(3,3) <<1384.43505859375, 0, 974.018310546875, 0,  1385.07885742188, 541.430114746094, 0, 0, 1);
                K_ = (cv::Mat_<double>(3,3) <<615.304504394531, 0, 326.230346679688, 0, 615.590637207031, 240.635604858398, 0, 0, 1);
            }

            /**
             * @brief 处理得到的识别数据
             * @param image: RGB图像
             * @return std::vector<cctagresult>: 处理结果
             */
            std::vector<cctagresult> resolver(cv::Mat image)
            {
                
                std::vector<cctagresult> results;
                //处理图片
                cctag::CCTag::List r_markers = cb_.process(image);
                cctag::CCTag* id_first = nullptr;
                cctag::CCTag* id_second = nullptr;
                double confidence_first = 0.0;
                double confidence_second = 0.0;
                for(cctag::CCTag& marker : r_markers)
                {
                    // 只绘制有效标记
                    if(marker.getStatus() != 1)
                    continue;
                    //记录id在容器中的位置
                    if(marker.id() == _IDX_0_)
                    {
                        //std::cout << "_IDX_0_ quality: " << marker.quality() << std::endl;
                        if(confidence_first < marker.quality())
                        {
                            id_first = &marker;
                            confidence_first = marker.quality();
                        }
                    }
                    else if(marker.id() == _IDX_1_)
                    {
                        //std::cout << "_IDX_1_ quality: " << marker.quality() << std::endl;
                        if(confidence_second < marker.quality())
                        {
                            id_second = &marker;
                            confidence_second = marker.quality();
                        }
                    }
                    else
                    {
                        //idx不匹配跳过
                        continue;
                    }
                }

                //如果有两个
                if(id_first && id_second)
                {
                    //方法一联合求解
                    results = computeCCTagRealPoseDual(*id_first, *id_second, K_, real_outer_radius_, _distance_1_2_);
                    //方法二单独求解
                    // cctagresult result = computeCCTagRealPose(*id_first, K_, real_outer_radius_);
                    // results.push_back(result);
                    // cctagresult result2 = computeCCTagRealPose(*id_second, K_, real_outer_radius_);
                    // results.push_back(result2);
                }
                //一个或没有
                else
                {
                    if(id_first)
                    {
                        //获取rt
                        cctagresult result = computeCCTagRealPose(*id_first, K_, real_outer_radius_);
                        results.push_back(result);
                    }
                    else if(id_second)
                    {
                        cctagresult result = computeCCTagRealPose(*id_second, K_, real_outer_radius_);
                        results.push_back(result);
                    }
                }
                return results;
            }

            /**
             * @brief 生成调试图像
             * @param results: 处理结果
             * @param image: RGB图像
             */
            void debug(const std::vector<cctagresult>& results, cv::Mat& image)
            {
                for(auto& result : results)
                {
                    if(result.id_ == -1)
                    {
                        continue;
                    }
                    // std::cout << "--------result:------------"<<std::endl;
                    // std::cout << "rvec: " << result.rvec_ << std::endl;
                    // std::cout << "tvec: " << result.tvec_ << std::endl;
                    drawCCTagPose(image, result.rvec_, result.tvec_, cv::Point2d(result.x_, result.y_), result.id_, K_, real_outer_radius_, result.flag_);
                }
            }

            /**
             * @brief 在图像上绘制CCTag的圆心、ID、3D姿态坐标系 和 平移向量数值(mm)
             * @param image 要绘制的图像（CV_8UC3格式）
             * @param rvec 旋转向量 (标签->相机)
             * @param tvec 平移向量 (单位：米)
             * @param center 检测到的圆心像素坐标 cv::Point2d
             * @param id CCTag标签ID
             * @param cameraMatrix 相机内参矩阵 K (3x3 CV_64F)
             * @param axis_length 坐标系轴长（米，建议=标签真实半径）
             * @param flag 是否画坐标系
             * @param distCoeffs 相机畸变系数 (默认无畸变)
             */
            static void drawCCTagPose(
                cv::Mat& image,
                const cv::Mat& rvec,
                const cv::Mat& tvec,
                const cv::Point2d& center,
                int id,
                const cv::Mat& cameraMatrix,
                double axis_length,
                int flag = 0,
                const cv::Mat& distCoeffs = cv::Mat::zeros(5,1,CV_64F)
            )
            {
                // ===================== 1. 绘制圆心（红色实心圆） =====================
                cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);

                // ===================== 2. 绘制标签ID（白色粗体文字） =====================
                std::string id_text = "ID: " + std::to_string(id);
                cv::putText(image, id_text, 
                            cv::Point(center.x + 10, center.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                            cv::Scalar(255, 255, 255), 2);

                
                // 如果 flag 为 0，不画坐标系，直接返回
                if(flag == 0)
                {
                    return;
                }
                // ===================== 3. 绘制平移向量 tvec (精确到 mm) =====================
                // 从 tvec 中取出 X/Y/Z (单位：米)
                double tx_m = tvec.at<double>(0);
                double ty_m = tvec.at<double>(1);
                double tz_m = tvec.at<double>(2);

                // 转换为毫米 (mm) 并保留整数，或者保留3位小数(米)
                // 这里采用【保留3位小数(米)】的方式，等价于 mm 精度
                char tvec_text[256];
                snprintf(tvec_text, sizeof(tvec_text), 
                        "T: [%.3f, %.3f, %.3f] m", 
                        tx_m, ty_m, tz_m);

                // 绘制位置：在 ID 下方，避免遮挡
                cv::putText(image, tvec_text, 
                            cv::Point(center.x + 10, center.y + 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                            cv::Scalar(0, 255, 255), 1); // 黄色文字，醒目

                // ===================== 4. 绘制3D坐标系（核心） =====================
                // X轴=红色  Y轴=绿色  Z轴=蓝色
                cv::drawFrameAxes(
                    image,
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec,
                    axis_length,
                    2
                );
            }


            cv::Mat K_;
        private:
            cctagbase cb_;
            double real_outer_radius_;
            double scale_;
            /**
             * @brief 从单个CCTag标记解算真实3D位姿
             * @param marker 有效CCTag对象（必须getStatus()==1）
             * @param cameraMatrix 相机内参矩阵 K (3x3, CV_64F)
             * @param real_outer_radius 标签【真实外圆半径】，单位：米 (如 0.05 代表5cm)
             * @return cctagresult: 结算出的rt
             */
            cctagresult computeCCTagRealPose(cctag::CCTag& marker, const cv::Mat& cameraMatrix, double real_outer_radius)
            {
                cctagresult result;
                result.id_ = marker.id();
                auto& cctag_pt = marker.centerImg();
                result.x_ = static_cast<double>(cctag_pt.x());
                result.y_ = static_cast<double>(cctag_pt.y());

                // ===================== 1. 获取单应矩阵 H =====================
                Eigen::Matrix3f H_eigen = marker.homography();
                cv::Mat H = (cv::Mat_<double>(3, 3) <<
                    H_eigen(0,0), H_eigen(0,1), H_eigen(0,2),
                    H_eigen(1,0), H_eigen(1,1), H_eigen(1,2),
                    H_eigen(2,0), H_eigen(2,1), H_eigen(2,2)
                );

                // ===================== 2. 定义 3D 世界点 =====================
                std::vector<cv::Point3f> objectPoints;
                objectPoints.emplace_back(0.0, 0.0, 0.0);
                objectPoints.emplace_back(real_outer_radius, 0.0, 0.0); 
                objectPoints.emplace_back(0.0, real_outer_radius, 0.0); 
                objectPoints.emplace_back(-real_outer_radius, 0.0, 0.0);
                objectPoints.emplace_back(0.0, -real_outer_radius, 0.0);

                //std::cout << "scale: " << scale_ << std::endl;
                // ===================== 3. 利用 H 投影得到 2D 图像点 =====================
                std::vector<cv::Point2f> imagePoints;
                for (const auto& pt3d : objectPoints)
                {
                    cv::Mat p_world = (cv::Mat_<double>(3, 1) << pt3d.x / real_outer_radius, pt3d.y / real_outer_radius, 1.0);
                    cv::Mat p_img = H * p_world;
                    double u = p_img.at<double>(0) / p_img.at<double>(2) / scale_;
                    double v = p_img.at<double>(1) / p_img.at<double>(2) / scale_;
                    imagePoints.emplace_back(u, v);
                }

                // ===================== 4. 调用 PnP 求解 =====================
                cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
                cv::Mat rvec, tvec;
                
                bool success = cv::solvePnP(
                    objectPoints,
                    imagePoints,
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec,
                    false,
                    cv::SOLVEPNP_IPPE
                );

                if (!success)
                {
                    result.flag_ = 0;
                    return result;
                }

                // ===================== 5. 第一步：消除二义性，确保Z轴指向相机 =====================
                cv::Mat R;
                cv::Rodrigues(rvec, R); 

                cv::Mat normal_vector_cam = R.col(2); 
                if (normal_vector_cam.at<double>(2) > 0) 
                {
                    R.col(1) *= -1;
                    R.col(2) *= -1;
                }

                // ===================== 6. 【新增】第二步：绕Z轴旋转，使Y轴朝上对齐 =====================
                // 原理：我们要最大化 Y轴 与 相机负Y轴(0, -1, 0) 的点积
                // 设旋转角为 theta，构建优化函数 f(theta) = A*sin(theta) + B*cos(theta)
                // 其中 A = R.col(0)[1], B = -R.col(1)[1]
                // 最优解 theta_opt = atan2(A, B)

                // 6.1 提取当前旋转矩阵在相机Y轴(索引1)上的分量
                double R0_y = R.at<double>(1, 0); // 原X轴在相机Y上的分量
                double R1_y = R.at<double>(1, 1); // 原Y轴在相机Y上的分量

                // 6.2 计算最优旋转角 theta_opt
                // 目标函数: maximize ( R0_y * sin(theta) - R1_y * cos(theta) )
                double theta_opt = std::atan2(R0_y, -R1_y);

                // 6.3 构造绕Z轴旋转 theta_opt 的旋转矩阵 Rz
                double cos_theta = std::cos(theta_opt);
                double sin_theta = std::sin(theta_opt);
                cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
                    cos_theta, -sin_theta, 0,
                    sin_theta,  cos_theta, 0,
                    0,          0,         1);

                // 6.4 更新旋转矩阵: R_new = R * Rz
                // 注意：是右乘，因为我们是在世界坐标系下绕Z轴旋转
                cv::Mat R_new = R * Rz;

                // ===================== 7. 转换回旋转向量并保存 =====================
                cv::Rodrigues(R_new, rvec);

                result.rvec_ = rvec;
                result.tvec_ = tvec; // 平移向量不变，因为只是绕中心旋转
                result.flag_ = 1;

                return result;
            }


            /**
             * @brief 利用两个共面且已知中心距的CCTag进行融合位姿解算
             *        自动构建世界坐标系：1为原点，1->2连线为X轴
             * @param marker1 第一个标记
             * @param marker2 第二个标记
             * @param cameraMatrix 相机内参
             * @param real_outer_radius 单个标签的半径
             * @param center_distance 两个标签圆心之间的【真实物理距离】(单位：米)
             * @return std::vector<cctagresult>
             */
            std::vector<cctagresult> computeCCTagRealPoseDual(
                cctag::CCTag& marker1, 
                cctag::CCTag& marker2, 
                const cv::Mat& cameraMatrix, 
                double real_outer_radius,
                double center_distance
            )
            {
                std::vector<cctagresult> results(2);
                results[0].id_ = marker1.id();
                results[1].id_ = marker2.id();
                
                // 填充像素坐标用于显示
                auto& pt1 = marker1.centerImg();
                auto& pt2 = marker2.centerImg();
                results[0].x_ = static_cast<double>(pt1.x());
                results[0].y_ = static_cast<double>(pt1.y());
                results[1].x_ = static_cast<double>(pt2.x());
                results[1].y_ = static_cast<double>(pt2.y());

                // ===================== 核心修改：分别单独解算两个标记 =====================
                // 1. 单独解算 Marker1
                cctagresult pose1 = computeCCTagRealPose(marker1, cameraMatrix, real_outer_radius);
                // 2. 单独解算 Marker2
                cctagresult pose2 = computeCCTagRealPose(marker2, cameraMatrix, real_outer_radius);

                // 解算失败判断
                if (!pose1.flag_ || !pose2.flag_)
                {
                    results[0].flag_ = 0;
                    results[1].flag_ = 0;
                    return results;
                }

                // ===================== 打印验证数据（写在本函数内，方便后续融合） =====================
                // std::cout << "\n=====================================" << std::endl;
                // std::cout << "【双标记 单独解算原始数据】" << std::endl;
                // std::cout << "Marker1 ID: " << pose1.id_ << "  像素坐标: (" << pose1.x_ << ", " << pose1.y_ << ")" << std::endl;
                // std::cout << "Marker1 旋转向量 rvec: " << pose1.rvec_.t() << std::endl;
                // std::cout << "Marker1 平移向量 tvec: " << pose1.tvec_.t() << std::endl;
                // std::cout << "-----------------------------------------------------" << std::endl;
                // std::cout << "Marker2 ID: " << pose2.id_ << "  像素坐标: (" << pose2.x_ << ", " << pose2.y_ << ")" << std::endl;
                // std::cout << "Marker2 旋转向量 rvec: " << pose2.rvec_.t() << std::endl;
                // std::cout << "Marker2 平移向量 tvec: " << pose2.tvec_.t() << std::endl;
                // std::cout << "-----------------------------------------------------" << std::endl;

                // 计算3D空间物理距离
                double dx = pose2.tvec_.at<double>(0) - pose1.tvec_.at<double>(0);
                double dy = pose2.tvec_.at<double>(1) - pose1.tvec_.at<double>(1);
                double dz = pose2.tvec_.at<double>(2) - pose1.tvec_.at<double>(2);
                double measured_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                // 计算像素距离
                double pixel_dist = std::hypot(pose2.x_ - pose1.x_, pose2.y_ - pose1.y_);

                // 打印距离对比（核心验证数据）
                // std::cout << "像素平面距离: " << pixel_dist << " px" << std::endl;
                // std::cout << "理论真实距离: " << center_distance << " m" << std::endl;
                // std::cout << "单独解算实测3D距离: " << measured_dist << " m" << std::endl;
                // std::cout << "=====================================\n" << std::endl;

                // ===================== 【预留位：后续在这里写融合算法】 =====================
                // 目前先返回 单独解算的原始结果
                results[0] = pose1;
                results[1] = pose2;

                // ===================== 后期融合代码示例（你后续直接替换这里即可） =====================
                // 1. 统一旋转矩阵
                // 2. 优化平移向量
                // 3. 刚性约束校准
                // ==========================================================================

                return results;
            }



        };


    }


}





#endif
