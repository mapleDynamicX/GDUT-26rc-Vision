#ifndef __POSE_RESOLVING_H_
#define __POSE_RESOLVING_H_
#include "./pre_pcl.h"
#include "./set_pcl.h"
#include "./../yolo/yolo_26obb.h"
#include "./../camera.h"

namespace Ten
{
    namespace Plane_FitLocator
    {
        #define _model_path_ "/home/maple/study3/li/best_openvino_model_op13/model"
        #define _xpu_ "cpu"

        class pose_resolving
        {
        public:
            /**
             * @brief 初始化函数
             * @param color_intr: 相机参数
             */
            pose_resolving(rs2_intrinsics color_intr)
                :yobb_(_model_path_, _xpu_)
            {
                color_intr_ = color_intr;
            }

            /**
             * @brief 处理图片，返回位姿
             * @param cf: rgb图片和深度图
             * @param pose: 返回的结果
             * @return bool:是否成功
             */
            bool process(Ten::camera_frame cf, Plane_Info& pose, pcl::PointCloud<pcl::PointXYZ>::Ptr& debug)
            {
                Ten::yolo::Detection det;
                //检测矩形框
                if(!run_yolo(cf.bgr_image, det))
                {
                    return false;
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                //提取其中点云
                if(!set_pcl_.set_Pcl_Cloud(cf.raw_depth_frame, color_intr_, det, raw_cloud))
                {
                    return false;
                }
                //debug = raw_cloud;
                pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if(!pre_pcl_.cloud_filter(raw_cloud, filter_cloud))
                {
                    return false;
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if(!pre_pcl_.Plane_fitter(filter_cloud, plane_cloud))
                {
                    return false;
                }
                Plane_Info resultpose = pre_pcl_.computeCenterAndNormal(plane_cloud);
                
                pose = resultpose;
                //调试函数可去
                drawDebugCoordinate(cf.bgr_image, color_intr_, resultpose);
                //debug_.publish_pointcloud(plane_cloud);
                debug = plane_cloud;
                return true;
            }

        private:
        Ten::yolo::yolo_26obb yobb_;
        Ten_set_pcl set_pcl_;
        rs2_intrinsics color_intr_;
        Ten_pre_pcl pre_pcl_;
        

            /**
             * @brief 返回最有检测结果
             * @param image: 输入图片
             * @param result: 输出结果
             * @return bool:处理是否成功
             */
            bool run_yolo(cv::Mat image, Ten::yolo::Detection& result)
            {
                std::vector<Ten::yolo::Detection> results = yobb_.worker(image);
                //调试，可不用
                drawDetections(image, results);
                if(results.size() == 0)
                {
                    return false;
                }
                result = results[0];
                return true;
            }


            /**
             * @brief 绘制平面3D坐标系到2D图像
             * @param image         输入/输出图像（直接在图上绘制）
             * @param color_intr     彩色相机内参（RealSense格式）
             * @param plane_info     平面信息（中心+姿态）
             * @param axis_length    坐标系轴长（单位：米，默认0.1米=10厘米，可调整）
             */
            void drawDebugCoordinate(
                cv::Mat& image,
                const rs2_intrinsics& color_intr,
                const Plane_Info& plane_info,
                double axis_length = 0.1
            )
            {
                // 1. 安全判断：图像为空直接返回
                if (image.empty()) return;

                // ===================== 步骤1：欧拉角(RPY) -> 旋转矩阵 =====================
                // 旋转顺序：Z(YAW) -> Y(PITCH) -> X(ROLL) （机械视觉/相机标准顺序）
                Eigen::Matrix3d rot_mat;
                rot_mat = Eigen::AngleAxisd(plane_info.plane_euler._yaw, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(plane_info.plane_euler._pitch, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(plane_info.plane_euler._roll, Eigen::Vector3d::UnitX());

                // ===================== 步骤2：定义3D坐标系坐标轴 =====================
                // 坐标系原点（平面中心点）
                Eigen::Vector3d center = plane_info.plane_center;
                // 三个坐标轴单位向量 * 轴长
                Eigen::Vector3d axis_x = rot_mat * Eigen::Vector3d(1, 0, 0) * axis_length;  // X轴
                Eigen::Vector3d axis_y = rot_mat * Eigen::Vector3d(0, 1, 0) * axis_length;  // Y轴
                Eigen::Vector3d axis_z = rot_mat * Eigen::Vector3d(0, 0, 1) * axis_length;  // Z轴

                // 坐标轴终点
                Eigen::Vector3d x_end = center + axis_x;
                Eigen::Vector3d y_end = center + axis_y;
                Eigen::Vector3d z_end = center + axis_z;

                // ===================== 步骤3：3D点投影到2D像素坐标 =====================
                auto project3dToPixel = [&](const Eigen::Vector3d& pt) -> cv::Point2f {
                    float point3d[3] = { (float)pt.x(), (float)pt.y(), (float)pt.z() };
                    float pixel[2];
                    // RealSense官方函数：3D点 -> 2D像素
                    rs2_project_point_to_pixel(pixel, &color_intr, point3d);
                    return { pixel[0], pixel[1] };
                };

                // 投影所有点
                cv::Point2f px_center = project3dToPixel(center);
                cv::Point2f px_x = project3dToPixel(x_end);
                cv::Point2f px_y = project3dToPixel(y_end);
                cv::Point2f px_z = project3dToPixel(z_end);

                // ===================== 步骤4：像素坐标越界判断 =====================
                auto isPointInImage = [&](const cv::Point2f& p) -> bool {
                    return p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows;
                };

                if (!isPointInImage(px_center)) return;

                // ===================== 步骤5：OpenCV绘制坐标系 =====================
                const int line_thickness = 2;   // 线宽
                const int circle_radius = 3;    // 原点半径
                // 颜色惯例：X红 / Y绿 / Z蓝
                const cv::Scalar COLOR_X = cv::Scalar(0, 0, 255);
                const cv::Scalar COLOR_Y = cv::Scalar(0, 255, 0);
                const cv::Scalar COLOR_Z = cv::Scalar(255, 0, 0);

                // 1. 绘制坐标系原点（实心圆）
                cv::circle(image, px_center, circle_radius, COLOR_Z, -1);

                // 2. 绘制X轴
                if (isPointInImage(px_x))
                    cv::line(image, px_center, px_x, COLOR_X, line_thickness);

                // 3. 绘制Y轴
                if (isPointInImage(px_y))
                    cv::line(image, px_center, px_y, COLOR_Y, line_thickness);

                // 4. 绘制Z轴（平面法向量方向）
                if (isPointInImage(px_z))
                    cv::line(image, px_center, px_z, COLOR_Z, line_thickness);


                // // 可选：添加轴文字标注
                // cv::putText(image, "X", px_x, cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_X, 1);
                // cv::putText(image, "Y", px_y, cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_Y, 1);
                // cv::putText(image, "Z", px_z, cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_Z, 1);
            }

        };


    }       
}       
#endif 