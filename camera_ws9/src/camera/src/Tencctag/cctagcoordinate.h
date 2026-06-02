#ifndef __CCTAGCOORDINATE_H_
#define __CCTAGCOORDINATE_H_
#include "cctagresolver.h"
#include "./../method_math.h"

namespace Ten
{

    namespace Tencctag
    {
        #define _length_of_pole_ 0.0 //杆的长度
        #define _id0_target_x_ 0.0
        #define _id0_target_y_ 0.0

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
                    cv::resize(image, resize, cv::Size(image.cols*scale_, image.rows*scale_));
                }
                
                //std::cout << "resize: " << resize.cols << std::endl;
                //获取检测结果
                std::vector<cctagresult> rets = rel_pose_.resolver(resize, frame_num);
                if(rets.size() <= 0)
                {
                    return -1;
                }
                else if(rets.size() == 1)
                {
    
                    cctagresult id0 = computeTargetPoseFromDualTags(rets[0], rel_pose_.K_);
                    
                    rel_pose_.debug({id0}, image);
                    biasx = id0.tvec_.at<double>(0, 0) - _id0_target_x_;
                    biasy = id0.tvec_.at<double>(1, 0) - _id0_target_y_;
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
             * @brief 基于单标签计算目标点位姿：沿标签自身Z轴正方向移动杆长后的位姿（相对于相机）
             * @param tag 单个标签的位姿结果（标签坐标系原点）
             * @param K 相机内参矩阵 (3x3 CV_64F)
             * @return cctagresult 目标点相对于相机的位姿，包含图像像素坐标
             */
            cctagresult computeTargetPoseFromDualTags(
                const cctagresult& tag, 
                const cv::Mat& K) 
            {
                cctagresult result;
                // 1. 姿态完全继承原标签（旋转不变）
                result.rvec_ = tag.rvec_.clone(); 
                result.flag_ = 1; 
                result.id_ = -2; // 自定义目标点ID

                // 2. 旋转向量 → 旋转矩阵（标签坐标系 → 相机坐标系）
                cv::Mat R_tag2cam;
                cv::Rodrigues(tag.rvec_, R_tag2cam);

                // 3. 标签局部坐标系下的目标点：沿Z轴正方向移动杆长
                // 标签坐标系原点：(0,0,0) → 目标点：(0, 0, _length_of_pole_)
                cv::Mat P_local = (cv::Mat_<double>(3, 1) << 0.0, 0.0, _length_of_pole_);

                // 4. 坐标转换：局部坐标系 → 相机坐标系
                // 公式：P_cam = R * P_local + t（标签在相机下的平移向量）
                result.tvec_ = R_tag2cam * P_local + tag.tvec_;

                // 5. 将3D目标点投影到2D图像像素（核心投影公式）
                double X = result.tvec_.at<double>(0);
                double Y = result.tvec_.at<double>(1);
                double Z = result.tvec_.at<double>(2);

                // 安全校验：点不能在相机后方
                if (Z <= 1e-6) {
                    std::cerr << "警告：目标点位于相机后方，投影无效！" << std::endl;
                    result.flag_ = 0;
                    result.x_ = 0;
                    result.y_ = 0;
                    return result;
                }

                // 提取相机内参
                double fx = K.at<double>(0, 0);
                double fy = K.at<double>(1, 1);
                double cx = K.at<double>(0, 2);
                double cy = K.at<double>(1, 2);

                // 针孔相机投影计算像素坐标
                double u = fx * (X / Z) + cx;
                double v = fy * (Y / Z) + cy;
                result.x_ = u;
                result.y_ = v;

                return result;
            }

        };

    }



}








#endif
