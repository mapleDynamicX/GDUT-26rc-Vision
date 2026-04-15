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
        #define _length_of_pole_ 0.05 //杆的长度

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
             * @param nCrowns: 指定环数
             */
            cctagcoordinate(double real_outer_radius = 0.1, int nCrowns = 3)
            :rel_pose_(real_outer_radius, nCrowns)
            {

            }

            /**
             * @brief 用于返回坐标和偏置
             * @param image: 图片
             * @param bias: 偏置（返回值）
             * @param pose: 武器头位姿
             * @return int: 0 位姿解算失败, 1 位姿解算成功, -1 bias和位姿解算都失败
             */
            int getpose(cv::Mat& image, double& biasx, double& biasy, Ten::XYZRPY& pose)
            {
                //获取检测结果
                std::vector<cctagresult> rets = rel_pose_.resolver(image);
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
                        }
                        else if(ret.id_ == _IDX_1_)
                        {
                            id1 = &ret;
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
                const cv::Mat& K) // 【新增】传入相机内参
            {
                cctagresult result;
                // 注意：因为要求姿态与id0一致，且现在是相对于相机，所以rvec直接复用id0的rvec
                result.rvec_ = id0.rvec_.clone(); 
                result.flag_ = 1; 
                result.id_ = 1000;

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
                
                // 计算 id1 在 id0 坐标系中的坐标
                cv::Mat p1_id0_mat = R0.t() * (t1_cam - t0_cam);
                cv::Vec3d p1_id0(p1_id0_mat.at<double>(0), 
                                p1_id0_mat.at<double>(1), 
                                p1_id0_mat.at<double>(2));

                cv::Vec3d p0_id0(0.0, 0.0, 0.0);

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

                // 归一化
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
                // 6. 【核心修改1】将点 b 转换到相机坐标系下
                // ==========================================
                // 公式: P_cam = R0 * P_id0 + t0
                cv::Mat P_b_id0_mat = (cv::Mat_<double>(3, 1) << point_b_id0[0], point_b_id0[1], point_b_id0[2]);
                cv::Mat P_b_cam_mat = R0 * P_b_id0_mat + t0_cam;

                // 赋值给 result.tvec_ (现在是相对于相机的)
                result.tvec_ = P_b_cam_mat;

                // ==========================================
                // 7. 【核心修改2】将3D点 b 投影到2D图像，计算像素坐标 (x_, y_)
                // ==========================================
                // 准备输入数据
                std::vector<cv::Point3f> points_3d;
                // 注意：PnP解算的tvec单位通常是米，这里要确保单位一致
                points_3d.emplace_back(
                    static_cast<float>(P_b_cam_mat.at<double>(0)),
                    static_cast<float>(P_b_cam_mat.at<double>(1)),
                    static_cast<float>(P_b_cam_mat.at<double>(2))
                );

                std::vector<cv::Point2f> points_2d;
                cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // 无畸变

                // 调用OpenCV投影函数
                // 注意：因为点b的姿态和id0一致，我们直接用id0的rvec和新的tvec进行投影
                cv::projectPoints(
                    points_3d,
                    result.rvec_,    // 旋转向量 (和id0一致)
                    result.tvec_,    // 平移向量 (点b在相机下的坐标)
                    K,               // 相机内参
                    distCoeffs,      // 畸变系数
                    points_2d
                );

                // 赋值像素坐标
                result.x_ = static_cast<double>(points_2d[0].x);
                result.y_ = static_cast<double>(points_2d[0].y);

                return result;
            }


        };

    }



}








#endif
