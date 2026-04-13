#ifndef __CCTAGRESOLVER_H_
#define __CCTAGRESOLVER_H_
#include "cctagbase.h"

namespace Ten
{

    namespace Tencctag
    {

        #define _IDX_0_ 0
        #define _IDX_1_ 1
        #define _IDX_2_ 2



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
                //设置为3个
                std::vector<cctagresult> results;
                results.resize(3);
                //处理图片
                cctag::CCTag::List r_markers = cb_.process(image);
                for(auto& marker : r_markers)
                {
                    // 只绘制有效标记
                    if(marker.getStatus() != 1)
                    continue;
                    //记录id属于那个容器
                    int flag = -1;
                    if(marker.id() == _IDX_0_)
                    {
                        flag = 0;
                    }
                    else if(marker.id() == _IDX_1_)
                    {
                        flag = 1;
                    }
                    else if(marker.id() == _IDX_2_)
                    {
                        flag = 2;
                    }
                    else
                    {
                        //idx不匹配跳过
                        continue;
                    }
                   //获取rt
                    results[flag] = computeCCTagRealPose(marker, K_, real_outer_radius_);
                    results[flag].id_ = marker.id();
                    // 坐标转换
                    auto& cctag_pt = marker.centerImg();
                    results[flag].x_ = static_cast<double>(cctag_pt.x());
                    results[flag].y_ = static_cast<double>(cctag_pt.y());
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
            cctagresult computeCCTagRealPose(const cctag::CCTag& marker, const cv::Mat& cameraMatrix, double real_outer_radius)
            {
                cctagresult result;

                // ===================== 1. 获取单应矩阵 H =====================
                Eigen::Matrix3f H_eigen = marker.homography();
                cv::Mat H = (cv::Mat_<double>(3, 3) <<
                    H_eigen(0,0), H_eigen(0,1), H_eigen(0,2),
                    H_eigen(1,0), H_eigen(1,1), H_eigen(1,2),
                    H_eigen(2,0), H_eigen(2,1), H_eigen(2,2)
                );

                // ===================== 2. 定义【单位尺度】的 3D 世界点 =====================
                // 注意：这里我们假设标签半径为 1.0（归一化单位），暂时不代入真实尺寸
                std::vector<cv::Point3f> objectPoints_unit;
                objectPoints_unit.emplace_back(0.0, 0.0, 0.0);          // 中心
                objectPoints_unit.emplace_back(1.0, 0.0, 0.0);          // 右 (单位半径)
                objectPoints_unit.emplace_back(0.0, 1.0, 0.0);          // 上 (单位半径)
                objectPoints_unit.emplace_back(-1.0, 0.0, 0.0);         // 左 (单位半径)
                objectPoints_unit.emplace_back(0.0, -1.0, 0.0);         // 下 (单位半径)

                // ===================== 3. 利用 H 投影得到对应的 2D 图像点 =====================
                std::vector<cv::Point2f> imagePoints;
                for (const auto& pt3d : objectPoints_unit)
                {
                    cv::Mat p_world = (cv::Mat_<double>(3, 1) << pt3d.x, pt3d.y, 1.0);
                    cv::Mat p_img = H * p_world;
                    double u = p_img.at<double>(0) / p_img.at<double>(2);
                    double v = p_img.at<double>(1) / p_img.at<double>(2);
                    imagePoints.emplace_back(u, v);
                }

                // ===================== 4. 调用 PnP 求解【单位尺度】位姿 =====================
                cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
                cv::Mat rvec, tvec_unit; // 注意：这里的 tvec 是单位尺度的
                
                bool success = cv::solvePnP(
                    objectPoints_unit,
                    imagePoints,
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec_unit,
                    false,
                    cv::SOLVEPNP_IPPE
                );

                if (!success)
                {
                    result.flag_ = 0;
                    return result;
                }

                result.flag_ = 1;
                result.rvec_ = rvec; // 旋转向量不受尺度影响，直接赋值

                // ===================== 5. 【关键一步】恢复真实物理尺度 =====================
                // tvec_unit 的单位是“标签半径数”
                // 真实 tvec = 单位 tvec * 真实标签半径
                result.tvec_ = tvec_unit * real_outer_radius;

                // 调试输出
                std::cout << "单位尺度距离: " << cv::norm(tvec_unit) << " (R)" << std::endl;
                std::cout << "真实物理距离: " << cv::norm(result.tvec_) << " (m)" << std::endl;

                return result;
            }

        };


    }


}





#endif
