#ifndef __CCTAGCOORDINATE_H_
#define __CCTAGCOORDINATE_H_
#include "cctagresolver.h"
#include "./../method_math.h"

namespace Ten
{

    namespace Tencctag
    {
        #define _proportion_1_2_ 0.5   //_IDX_0_指向_IDX_1_， 然后取比例为_proportion_1_2_
        #define _length_of_offset_ 0.0   //距离_proportion_1_2_计算出线上的点，并垂直与直线向上的距离
        #define _length_of_pole_ 0.06 //杆的长度

        #define _id0_target_x_ 0.0
        #define _id0_target_y_ 0.0
       
        #define _id1_target_x_ 0.0
        #define _id1_target_y_ 0.0

        #define _pole_target_x_ 0.0
        #define _pole_target_y_ 0.0

        //后期精度不够还可以做像素优化

        class cctagcoordinate
        {
        public:
            /**
             * @param real_outer_radius: 圆半径
             * @param scale 缩放比例
             * @param nCrowns: 指定环数
             */
            cctagcoordinate(double real_outer_radius = 0.1, double scale = 1.0, int nCrowns = 3)
            :rel_pose_(real_outer_radius, scale, nCrowns)
            ,scale_(scale)
            {

            }

            /**
             * @brief 用于返回坐标和偏置
             * @param image: 图片
             * @param bias: 偏置（返回值）
             * @param pose: 武器头位姿
             * @param frame_num: 第几帧
             * @return int: 0 位姿解算失败, 1 位姿解算成功, -1 bias和位姿解算都失败
             */
            int getpose(cv::Mat& image, double& biasx, double& biasy, Ten::XYZRPY& pose, size_t frame_num = 0)
            {
                if(image.empty())
                {
                    return -2;
                }
                cv::Mat resize;
                if(scale_ > 0.99)
                {
                    resize = image;
                }
                else
                {
                    cv::resize(image, resize, cv::Size(640*scale_, 480*scale_));
                }
                
                //std::cout << "resize: " << resize.cols << std::endl;
                //获取检测结果
                std::vector<cctagresult> rets = rel_pose_.resolver(resize, frame_num);
                //std::cout << "rets.size(): " << rets.size() << std::endl;
                //两个标签
                cctagresult* id0 = nullptr;
                cctagresult* id1 = nullptr;
                //情况标志位
                int flag = 0;

                if(rets.size() < 0)
                {
                    return -1;
                }
                else if(rets.size() == 1)
                {
                    for(auto& ret : rets)
                    {
                        if(ret.id_ == _IDX_0_)
                        {
                            id0 = &ret;
                            rel_pose_.debug({*id0}, image);
                        }
                        else if(ret.id_ == _IDX_1_)
                        {
                            id1 = &ret;
                            rel_pose_.debug({*id1}, image);
                        }
                    }

                    if(id0)
                    {
                        biasx = id0->tvec_.at<double>(0, 0) - _id0_target_x_;
                        biasy = id0->tvec_.at<double>(1, 0) - _id0_target_y_;
                    }
                    else if(id1)
                    {
                        biasx = id1->tvec_.at<double>(0, 0) - _id1_target_x_;
                        biasy = id1->tvec_.at<double>(1, 0) - _id1_target_y_;
                    }
                    else
                    {
                        return -1;
                    }
                }
                else if(rets.size() == 2)
                {
                    //取1,2
                    for(auto& ret : rets)
                    {
                        if(ret.id_ == _IDX_0_)
                        {
                            id0 = &ret;
                        }
                        else if(ret.id_ == _IDX_1_)
                        {
                            id1 = &ret;
                        }
                    }
                    if(id0 && id1)
                    {
                        cctagresult camera_to_pole = computeTargetPoseFromDualTags(*id0, *id1, rel_pose_.K_);
                        rel_pose_.debug({camera_to_pole}, image);
                        pose._xyz._x = camera_to_pole.tvec_.at<double>(0, 0); 
                        pose._xyz._y = camera_to_pole.tvec_.at<double>(1, 0); 
                        pose._xyz._z = camera_to_pole.tvec_.at<double>(2, 0);
                        
                        biasx = pose._xyz._x - _pole_target_x_;
                        biasy = pose._xyz._y - _pole_target_y_;
                    }
                    else
                    {
                        return -1;
                    }
                    return 1;
                }
                else
                {
                    return -1;
                }
                return 0;
            }
            
        private:
            cctagresolver rel_pose_;
            double scale_;

            /**
             * @brief 基于双标签计算目标点b的位姿 (直接相对于相机)
             * @param id0 第一个标签的位姿结果 (基准坐标系)
             * @param id1 第二个标签的位姿结果
             * @param K 相机内参矩阵 (3x3 CV_64F)
             * @return cctagresult 点b相对于相机的位姿，且包含b在图像上的像素坐标
             */
            cctagresult computeTargetPoseFromDualTags(
                const cctagresult& id0, 
                const cctagresult& id1,
                const cv::Mat& K) 
            {
                cctagresult result;
                // 姿态与id0一致，直接复用id0的rvec
                result.rvec_ = id0.rvec_.clone(); 
                result.flag_ = 1; 
                result.id_ = -2;

                // ==========================================
                // 1. 准备工作：将旋转向量转为旋转矩阵
                // ==========================================
                cv::Mat R0;
                cv::Rodrigues(id0.rvec_, R0); // R0: id0 -> 相机 的旋转矩阵

                // ==========================================
                // 2. 将 id1 的位置转换到 id0 的局部坐标系下
                // ==========================================
                cv::Mat t0_cam = id0.tvec_;          // id0原点在相机下的坐标
                cv::Mat t1_cam = id1.tvec_;          // id1原点在相机下的坐标
                
                // 计算 id1 在 id0 坐标系中的坐标: P_id0 = R0.t() * (P_cam - t0_cam)
                cv::Mat p1_id0_mat = R0.t() * (t1_cam - t0_cam);
                cv::Vec3d p1_id0(p1_id0_mat.at<double>(0), 
                                p1_id0_mat.at<double>(1), 
                                p1_id0_mat.at<double>(2));

                cv::Vec3d p0_id0(0.0, 0.0, 0.0); // id0 自身在局部坐标系的原点

                // ==========================================
                // 3. 计算点 a: id0->id1 连线上的比例点 (中点)
                // ==========================================
                cv::Vec3d vec_dir = p1_id0 - p0_id0; 
                cv::Vec3d point_a = p0_id0 + vec_dir * _proportion_1_2_;

                // ==========================================
                // 4. 计算垂直偏置方向 (在 id0 的 XY 平面上)
                // ==========================================
                cv::Vec3d vec_dir_xy(vec_dir[0], vec_dir[1], 0.0); 
                cv::Vec3d candidate1(-vec_dir_xy[1],  vec_dir_xy[0], 0.0);
                cv::Vec3d candidate2( vec_dir_xy[1], -vec_dir_xy[0], 0.0);

                cv::Vec3d y_axis(0.0, 1.0, 0.0);
                cv::Vec3d vertical_dir;
                
                if (candidate1.dot(y_axis) > 0)
                {
                    vertical_dir = candidate1;
                }
                else
                {
                    vertical_dir = candidate2;
                }

                // 归一化并应用偏置长度
                double norm = cv::norm(vertical_dir);
                if (norm < 1e-6) 
                {
                    vertical_dir = y_axis; 
                }
                else
                {
                    vertical_dir = vertical_dir / norm * _length_of_offset_;
                }

                // ==========================================
                // 5. 计算最终点 b (在 id0 局部坐标系下)
                // ==========================================
                cv::Vec3d point_on_plane = point_a + vertical_dir;
                cv::Vec3d z_axis(0.0, 0.0, 1.0);
                cv::Vec3d point_b_id0 = point_on_plane + z_axis * _length_of_pole_;

                // ==========================================
                // 6. 将点 b 转换到相机坐标系下
                // ==========================================
                // 公式: P_cam = R0 * P_id0 + t0_cam
                cv::Mat P_b_id0_mat = (cv::Mat_<double>(3, 1) << point_b_id0[0], point_b_id0[1], point_b_id0[2]);
                cv::Mat P_b_cam_mat = R0 * P_b_id0_mat + t0_cam;

                // 赋值给 result.tvec_ (现在是相对于相机的)
                result.tvec_ = P_b_cam_mat;

                // ==========================================
                // 7. 【修复】手动将 3D 点 b 投影到 2D 图像
                // ==========================================
                
                // 提取相机坐标系下的坐标
                double X = P_b_cam_mat.at<double>(0);
                double Y = P_b_cam_mat.at<double>(1);
                double Z = P_b_cam_mat.at<double>(2);

                // 安全检查：防止除零或点在相机后方
                if (Z <= 1e-6) {
                    std::cerr << "警告：目标点 Z 坐标 <= 0，位于相机后方或重合，投影无效。" << std::endl;
                    result.flag_ = 0; // 标记失败
                    result.x_ = 0;
                    result.y_ = 0;
                    return result;
                }

                // 从内参矩阵 K 中提取 fx, fy, cx, cy
                // 假设 K 的格式为：
                // [ fx  0  cx ]
                // [  0 fy  cy ]
                // [  0  0   1 ]
                double fx = K.at<double>(0, 0);
                double fy = K.at<double>(1, 1);
                double cx = K.at<double>(0, 2);
                double cy = K.at<double>(1, 2);

                // 计算投影
                // 1. 归一化坐标
                double x_norm = X / Z;
                double y_norm = Y / Z;

                // 2. 乘以内参得到像素坐标
                double u = fx * x_norm + cx;
                double v = fy * y_norm + cy;

                // 赋值结果
                result.x_ = u;
                result.y_ = v;

                // std::cout << "Target 3D in Cam: [" << X << ", " << Y << ", " << Z << "]" << std::endl;
                // std::cout << "Projected Pixel:  [" << result.x_ << ", " << result.y_ << "]" << std::endl;
                return result;
            }



        };

    }



}








#endif
