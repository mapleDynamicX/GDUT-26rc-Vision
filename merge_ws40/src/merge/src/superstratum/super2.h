#ifndef __SUPPER2_H_
#define __SUPPER2_H_

#include"super.h"
#include"./../yolo/yolo_han2.h"

namespace Ten
{
    namespace superstratum
    {

        class supper2
        {
        public:
            supper2()
            {
                //设置稳态误差
                Ten::XYZRPY xyzrpy_error;
                xyzrpy_error._xyz._x = 0;
                xyzrpy_error._xyz._y = 0;
                xyzrpy_error._xyz._z = 0;
                xyzrpy_error._rpy._roll = 0;
                xyzrpy_error._rpy._pitch = 0;
                xyzrpy_error._rpy._yaw = 0;
                //coordinate_transformation_.set_stead_state_error(xyzrpy_error);
                _CAMERA_TRANSFORMATION_.set_error(xyzrpy_error);
                lidar_to_camera_transform_matrix_ << 
                -0.0293067,  -0.999359,  -0.0205564,  0.0519757,  
                0.0195515,  0.0199882,  -0.999609,  0.47424,  
                0.999379,  -0.0296971,  0.0189532,  0.336381,  
                0.0         ,  0.0        ,  0.0        ,  1.0; 
                _CAMERA_TRANSFORMATION_.camerainfo_.set_Extrinsic_Matrix(lidar_to_camera_transform_matrix_);
               
            }

            void set_img(cv::Mat image, cv::Mat rvec, cv::Mat tvec)
            {

            }

            void process_img()
            {
                //世界点和box_list的类对象，对里面数据进行处理
                Ten::init_3d_box world_point;
                //camera_transformation_.camerainfo_.set_RT(rvec_, tvec_);
                camera_transformation_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
                world_point.pcl_to_C();
                //zbuffer_.set_exist_boxes(exist_box);
                //zbuffer_.set_box_lists_(image, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            }

        private:
            Eigen::Matrix4d lidar_to_camera_transform_matrix_ = Eigen::Matrix4d::Identity(); //雷达到相机
            Ten::Ten_worldtocamera camera_transformation_; //坐标点转换器，用于将世界坐标系下的点变换到当前坐标系，以及像素坐标系
            Ten::Ten_occlusion_handing zbuffer_; //zb处理器，用于处理遮挡关系
        };

    }


}


#endif
