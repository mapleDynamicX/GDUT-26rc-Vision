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
            cctagresolver(double real_outer_radius = 0.1, int nCrowns = 3)
            :cb_(nCrowns)
            ,real_outer_radius_(real_outer_radius)
            {
                K_ = (cv::Mat_<double>(3,3) <<1380.4350, 0, 974.0183,0,  1385.0788, 541.4301, 0, 0, 1);
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
                        std::cout << "_IDX_0_ quality: " << marker.quality() << std::endl;
                        if(confidence_first < marker.quality())
                        {
                            id_first = &marker;
                            confidence_first = marker.quality();
                        }
                    }
                    else if(marker.id() == _IDX_1_)
                    {
                        std::cout << "_IDX_1_ quality: " << marker.quality() << std::endl;
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
            void debug(const std::vector<cctagresult>& results, cv::Mat image)
            {
                for(auto& result : results)
                {
                    if(result.id_ == -1)
                    {
                        continue;
                    }
                    std::cout << "--------result:------------"<<std::endl;
                    std::cout << "rvec: " << result.rvec_ << std::endl;
                    std::cout << "tvec: " << result.tvec_ << std::endl;
                    drawCCTagPose(image, result.rvec_, result.tvec_, cv::Point2d(result.x_, result.y_), result.id_, K_, real_outer_radius_, result.flag_);
                }
            }

            /**
             * @brief 在图像上绘制CCTag的圆心、ID和3D姿态坐标系
             * @param image 要绘制的图像（CV_8UC3格式）
             * @param rvec 旋转向量 (标签->相机)
             * @param tvec 平移向量 (单位：米)
             * @param center 检测到的圆心像素坐标 cv::Point2d
             * @param id CCTag标签ID
             * @param cameraMatrix 相机内参矩阵 K (3x3 CV_64F)
             * @param axis_length 坐标系轴长（米，建议=标签真实半径）
             * @param flag 是否化坐标系
             *@param distCoeffs 相机畸变系数 (默认无畸变)
             */
            static void drawCCTagPose(
                cv::Mat& image,
                const cv::Mat& rvec,
                const cv::Mat& tvec,
                const cv::Point2d& center,
                int id,
                const cv::Mat& cameraMatrix,
                double axis_length,  // 默认5cm，和你的真实标签半径一致
                int flag = 0,
                const cv::Mat& distCoeffs = cv::Mat::zeros(5,1,CV_64F)
            )
            {
                // ===================== 1. 绘制圆心（红色实心圆） =====================
                cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);

                // ===================== 2. 绘制标签ID（白色文字） =====================
                std::string id_text = "ID: " + std::to_string(id);
                cv::putText(image, id_text, 
                            cv::Point(center.x + 10, center.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                            cv::Scalar(255,255,255), 2);
                if(flag == 0)
                {
                    return;
                }
                // ===================== 3. 绘制3D坐标系（核心） =====================
                // X轴=红色  Y轴=绿色  Z轴=蓝色
                // 坐标系原点 = CCTag圆心，Z轴垂直标签平面向外
                cv::drawFrameAxes(
                    image,
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec,
                    axis_length,  // 坐标轴长度（米）
                    2  // 线条粗细
                );
            }

        private:
            cctagbase cb_;
            cv::Mat K_;
            double real_outer_radius_;

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

                // ===================== 2. 定义【真实尺度】的 3D 世界点 =====================
                // 注意：这里直接使用 real_outer_radius，不再是单位尺度
                std::vector<cv::Point3f> objectPoints;
                objectPoints.emplace_back(0.0, 0.0, 0.0);
                objectPoints.emplace_back(real_outer_radius, 0.0, 0.0); // X轴
                objectPoints.emplace_back(0.0, real_outer_radius, 0.0); // Y轴
                objectPoints.emplace_back(-real_outer_radius, 0.0, 0.0);
                objectPoints.emplace_back(0.0, -real_outer_radius, 0.0);

                // ===================== 3. 利用 H 投影得到对应的 2D 图像点 =====================
                std::vector<cv::Point2f> imagePoints;
                for (const auto& pt3d : objectPoints)
                {
                    // 注意：因为objectPoints已经包含了real_outer_radius，这里投影逻辑不变
                    // 因为 H 是尺度无关的
                    cv::Mat p_world = (cv::Mat_<double>(3, 1) << pt3d.x / real_outer_radius, pt3d.y / real_outer_radius, 1.0);
                    cv::Mat p_img = H * p_world;
                    double u = p_img.at<double>(0) / p_img.at<double>(2);
                    double v = p_img.at<double>(1) / p_img.at<double>(2);
                    imagePoints.emplace_back(u, v);
                }

                // ===================== 4. 调用 PnP 求解 =====================
                cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
                cv::Mat rvec, tvec;
                
                bool success = cv::solvePnP(
                    objectPoints,      // 传入真实尺度的点
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

                // ===================== 5. 关键步骤：消除二义性，确保Z轴方向 =====================
                
                cv::Mat R;
                cv::Rodrigues(rvec, R); // 把旋转向量转为旋转矩阵

                cv::Mat normal_vector_cam = R.col(2); // 取出旋转矩阵的第三列 (世界Z轴在相机下的方向)
                if (normal_vector_cam.at<double>(2) > 0) 
                {
                    // 翻转策略：绕X轴旋转180度
                    // 1. 旋转矩阵取反 Y 和 Z 列
                    R.col(1) *= -1;
                    R.col(2) *= -1;
                    // 3. 把修正后的旋转矩阵转回向量
                    cv::Rodrigues(R, rvec);
                }
                // 保存结果
                result.rvec_ = rvec;
                result.tvec_ = tvec;
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
                std::cout << "像素平面距离: " << pixel_dist << " px" << std::endl;
                std::cout << "理论真实距离: " << center_distance << " m" << std::endl;
                std::cout << "单独解算实测3D距离: " << measured_dist << " m" << std::endl;
                std::cout << "=====================================\n" << std::endl;

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
