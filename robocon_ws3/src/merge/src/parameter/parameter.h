#ifndef __PARAMETER_H_
#define __PARAMETER_H_
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

namespace Ten
{    
    namespace superstratum
    {
        //r1车相对雷达
        extern double _r1_xyzrpy_car_xyz_x_;
        extern double _r1_xyzrpy_car_xyz_y_;
        extern double _r1_xyzrpy_car_xyz_z_;
        extern double _r1_xyzrpy_car_rpy_roll_;
        extern double _r1_xyzrpy_car_rpy_pitch_;
        extern double _r1_xyzrpy_car_rpy_yaw_;
        //r1建图误差
        extern double _r1_xyzrpy_error_xyz_x_;
        extern double _r1_xyzrpy_error_xyz_y_;
        extern double _r1_xyzrpy_error_xyz_z_;
        extern double _r1_xyzrpy_error_rpy_roll_;
        extern double _r1_xyzrpy_error_rpy_pitch_;
        extern double _r1_xyzrpy_error_rpy_yaw_;
        //r2车相对雷达
        extern double _r2_xyzrpy_car_xyz_x_;
        extern double _r2_xyzrpy_car_xyz_y_;
        extern double _r2_xyzrpy_car_xyz_z_;
        extern double _r2_xyzrpy_car_rpy_roll_;
        extern double _r2_xyzrpy_car_rpy_pitch_;
        extern double _r2_xyzrpy_car_rpy_yaw_;
        //r2建图误差
        extern double _r2_xyzrpy_error_xyz_x_;
        extern double _r2_xyzrpy_error_xyz_y_;
        extern double _r2_xyzrpy_error_xyz_z_ ;
        extern double _r2_xyzrpy_error_rpy_roll_;
        extern double _r2_xyzrpy_error_rpy_pitch_;
        extern double _r2_xyzrpy_error_rpy_yaw_;
        //识别卷轴
        extern std::string _coner_path_;
        extern std::string _juanzhou_path_;
        extern int _box_num_;
    }
    //串口最大遍历数值
    extern int _max_serial_num_;
    //重定位参数
    extern double _voxeldownsample_threshold_for_teaser_;
    extern double _voxeldownsample_threshold_for_icp_;
    extern double _setmaxcorrespondencedistance_nano_gicp_;
    extern size_t _min_num_of_point_cloud_for_relocation_;
    //point_lio建图下采样深度
    extern double _voxeldownsample_threshold_;
    // 640x480
    extern double camera_fx_640;
    extern double camera_fy_640;
    extern double camera_cx_640;
    extern double camera_cy_640;
    // 1920x1080
    extern double camera_fx_1080;
    extern double camera_fy_1080;
    extern double camera_cx_1080;
    extern double camera_cy_1080;

    namespace parameter
    {

        class loadyaml
        {
        public:
        loadyaml()
        {
            // 直接读取 yaml 文件
            YAML::Node config = YAML::LoadFile(std::string(ROOT_DIR) + std::string("src/parameter/config.yaml"));

            // ==============================================
            // 【superstratum 命名空间】所有参数赋值
            // ==============================================
            superstratum::_r1_xyzrpy_car_xyz_x_                = config["_r1_xyzrpy_car_xyz_x_"].as<double>();
            superstratum::_r1_xyzrpy_car_xyz_y_                = config["_r1_xyzrpy_car_xyz_y_"].as<double>();
            superstratum::_r1_xyzrpy_car_xyz_z_                = config["_r1_xyzrpy_car_xyz_z_"].as<double>();
            superstratum::_r1_xyzrpy_car_rpy_roll_             = config["_r1_xyzrpy_car_rpy_roll_"].as<double>();
            superstratum::_r1_xyzrpy_car_rpy_pitch_            = config["_r1_xyzrpy_car_rpy_pitch_"].as<double>();
            superstratum::_r1_xyzrpy_car_rpy_yaw_              = config["_r1_xyzrpy_car_rpy_yaw_"].as<double>();

            superstratum::_r1_xyzrpy_error_xyz_x_              = config["_r1_xyzrpy_error_xyz_x_"].as<double>();
            superstratum::_r1_xyzrpy_error_xyz_y_              = config["_r1_xyzrpy_error_xyz_y_"].as<double>();
            superstratum::_r1_xyzrpy_error_xyz_z_              = config["_r1_xyzrpy_error_xyz_z_"].as<double>();
            superstratum::_r1_xyzrpy_error_rpy_roll_           = config["_r1_xyzrpy_error_rpy_roll_"].as<double>();
            superstratum::_r1_xyzrpy_error_rpy_pitch_          = config["_r1_xyzrpy_error_rpy_pitch_"].as<double>();
            superstratum::_r1_xyzrpy_error_rpy_yaw_            = config["_r1_xyzrpy_error_rpy_yaw_"].as<double>();

            superstratum::_r2_xyzrpy_car_xyz_x_                = config["_r2_xyzrpy_car_xyz_x_"].as<double>();
            superstratum::_r2_xyzrpy_car_xyz_y_                = config["_r2_xyzrpy_car_xyz_y_"].as<double>();
            superstratum::_r2_xyzrpy_car_xyz_z_                = config["_r2_xyzrpy_car_xyz_z_"].as<double>();
            superstratum::_r2_xyzrpy_car_rpy_roll_             = config["_r2_xyzrpy_car_rpy_roll_"].as<double>();
            superstratum::_r2_xyzrpy_car_rpy_pitch_            = config["_r2_xyzrpy_car_rpy_pitch_"].as<double>();
            superstratum::_r2_xyzrpy_car_rpy_yaw_              = config["_r2_xyzrpy_car_rpy_yaw_"].as<double>();

            superstratum::_r2_xyzrpy_error_xyz_x_              = config["_r2_xyzrpy_error_xyz_x_"].as<double>();
            superstratum::_r2_xyzrpy_error_xyz_y_              = config["_r2_xyzrpy_error_xyz_y_"].as<double>();
            superstratum::_r2_xyzrpy_error_xyz_z_              = config["_r2_xyzrpy_error_xyz_z_"].as<double>();
            superstratum::_r2_xyzrpy_error_rpy_roll_           = config["_r2_xyzrpy_error_rpy_roll_"].as<double>();
            superstratum::_r2_xyzrpy_error_rpy_pitch_          = config["_r2_xyzrpy_error_rpy_pitch_"].as<double>();
            superstratum::_r2_xyzrpy_error_rpy_yaw_            = config["_r2_xyzrpy_error_rpy_yaw_"].as<double>();

            superstratum::_coner_path_                         = config["_coner_path_"].as<std::string>();
            superstratum::_juanzhou_path_                      = config["_juanzhou_path_"].as<std::string>();
            superstratum::_box_num_                            = config["_box_num_"].as<int>();

            // ==============================================
            // 全局变量赋值
            // ==============================================
            _max_serial_num_                                   = config["_max_serial_num_"].as<int>();
            _voxeldownsample_threshold_for_teaser_             = config["_voxeldownsample_threshold_for_teaser_"].as<double>();
            _voxeldownsample_threshold_for_icp_                = config["_voxeldownsample_threshold_for_icp_"].as<double>();
            _setmaxcorrespondencedistance_nano_gicp_           = config["_setmaxcorrespondencedistance_nano_gicp_"].as<double>();
            _min_num_of_point_cloud_for_relocation_            = config["_min_num_of_point_cloud_for_relocation_"].as<size_t>();
            _voxeldownsample_threshold_                        = config["_voxeldownsample_threshold_"].as<double>();

            // 相机内参
            // ==============================================
            Ten::camera_fx_640                                 = config["camera_fx_640"].as<double>();
            Ten::camera_fy_640                                 = config["camera_fy_640"].as<double>();
            Ten::camera_cx_640                                 = config["camera_cx_640"].as<double>();
            Ten::camera_cy_640                                 = config["camera_cy_640"].as<double>();

            Ten::camera_fx_1080                                = config["camera_fx_1080"].as<double>();
            Ten::camera_fy_1080                                = config["camera_fy_1080"].as<double>();
            Ten::camera_cx_1080                                = config["camera_cx_1080"].as<double>();
            Ten::camera_cy_1080                                = config["camera_cy_1080"].as<double>();

            // ==============================================
            // 【全部打印】加载完成后输出所有参数
            // ==============================================
            std::cout << "\n==================== 加载 YAML 参数完成 ====================\n" << std::endl;

            std::cout << "--- r1 车相对雷达 ---" << std::endl;
            std::cout << "_r1_xyzrpy_car_xyz_x_:               " << superstratum::_r1_xyzrpy_car_xyz_x_ << std::endl;
            std::cout << "_r1_xyzrpy_car_xyz_y_:               " << superstratum::_r1_xyzrpy_car_xyz_y_ << std::endl;
            std::cout << "_r1_xyzrpy_car_xyz_z_:               " << superstratum::_r1_xyzrpy_car_xyz_z_ << std::endl;
            std::cout << "_r1_xyzrpy_car_rpy_roll_:            " << superstratum::_r1_xyzrpy_car_rpy_roll_ << std::endl;
            std::cout << "_r1_xyzrpy_car_rpy_pitch_:           " << superstratum::_r1_xyzrpy_car_rpy_pitch_ << std::endl;
            std::cout << "_r1_xyzrpy_car_rpy_yaw_:             " << superstratum::_r1_xyzrpy_car_rpy_yaw_ << std::endl;

            std::cout << "\n--- r1 建图误差 ---" << std::endl;
            std::cout << "_r1_xyzrpy_error_xyz_x_:             " << superstratum::_r1_xyzrpy_error_xyz_x_ << std::endl;
            std::cout << "_r1_xyzrpy_error_xyz_y_:             " << superstratum::_r1_xyzrpy_error_xyz_y_ << std::endl;
            std::cout << "_r1_xyzrpy_error_xyz_z_:             " << superstratum::_r1_xyzrpy_error_xyz_z_ << std::endl;
            std::cout << "_r1_xyzrpy_error_rpy_roll_:          " << superstratum::_r1_xyzrpy_error_rpy_roll_ << std::endl;
            std::cout << "_r1_xyzrpy_error_rpy_pitch_:         " << superstratum::_r1_xyzrpy_error_rpy_pitch_ << std::endl;
            std::cout << "_r1_xyzrpy_error_rpy_yaw_:           " << superstratum::_r1_xyzrpy_error_rpy_yaw_ << std::endl;

            std::cout << "\n--- r2 车相对雷达 ---" << std::endl;
            std::cout << "_r2_xyzrpy_car_xyz_x_:               " << superstratum::_r2_xyzrpy_car_xyz_x_ << std::endl;
            std::cout << "_r2_xyzrpy_car_xyz_y_:               " << superstratum::_r2_xyzrpy_car_xyz_y_ << std::endl;
            std::cout << "_r2_xyzrpy_car_xyz_z_:               " << superstratum::_r2_xyzrpy_car_xyz_z_ << std::endl;
            std::cout << "_r2_xyzrpy_car_rpy_roll_:            " << superstratum::_r2_xyzrpy_car_rpy_roll_ << std::endl;
            std::cout << "_r2_xyzrpy_car_rpy_pitch_:           " << superstratum::_r2_xyzrpy_car_rpy_pitch_ << std::endl;
            std::cout << "_r2_xyzrpy_car_rpy_yaw_:             " << superstratum::_r2_xyzrpy_car_rpy_yaw_ << std::endl;

            std::cout << "\n--- r2 建图误差 ---" << std::endl;
            std::cout << "_r2_xyzrpy_error_xyz_x_:             " << superstratum::_r2_xyzrpy_error_xyz_x_ << std::endl;
            std::cout << "_r2_xyzrpy_error_xyz_y_:             " << superstratum::_r2_xyzrpy_error_xyz_y_ << std::endl;
            std::cout << "_r2_xyzrpy_error_xyz_z_:             " << superstratum::_r2_xyzrpy_error_xyz_z_ << std::endl;
            std::cout << "_r2_xyzrpy_error_rpy_roll_:          " << superstratum::_r2_xyzrpy_error_rpy_roll_ << std::endl;
            std::cout << "_r2_xyzrpy_error_rpy_pitch_:         " << superstratum::_r2_xyzrpy_error_rpy_pitch_ << std::endl;
            std::cout << "_r2_xyzrpy_error_rpy_yaw_:           " << superstratum::_r2_xyzrpy_error_rpy_yaw_ << std::endl;

            std::cout << "\n--- 识别卷轴 ---" << std::endl;
            std::cout << "_coner_path_:                        " << superstratum::_coner_path_ << std::endl;
            std::cout << "_juanzhou_path_:                     " << superstratum::_juanzhou_path_ << std::endl;
            std::cout << "_box_num_:                           " << superstratum::_box_num_ << std::endl;

            std::cout << "\n--- 串口 & 重定位 ---" << std::endl;
            std::cout << "_max_serial_num_:                    " << _max_serial_num_ << std::endl;
            std::cout << "_voxeldownsample_threshold_for_teaser_:   " << _voxeldownsample_threshold_for_teaser_ << std::endl;
            std::cout << "_voxeldownsample_threshold_for_icp_:      " << _voxeldownsample_threshold_for_icp_ << std::endl;
            std::cout << "_setmaxcorrespondencedistance_nano_gicp_:  " << _setmaxcorrespondencedistance_nano_gicp_ << std::endl;
            std::cout << "_min_num_of_point_cloud_for_relocation_:   " << _min_num_of_point_cloud_for_relocation_ << std::endl;
            std::cout << "\n--- point_lio建图下采样深度 ---" << std::endl;
            std::cout << "_voxeldownsample_threshold_:   " << _voxeldownsample_threshold_ << std::endl;

            std::cout << "\n--- 相机内参 640x480 ---" << std::endl;
            std::cout << "fx_640:    " << Ten::camera_fx_640  << std::endl;
            std::cout << "fy_640:    " << Ten::camera_fy_640  << std::endl;
            std::cout << "cx_640:    " << Ten::camera_cx_640  << std::endl;
            std::cout << "cy_640:    " << Ten::camera_cy_640  << std::endl;

            std::cout << "\n--- 相机内参 1920x1080 ---" << std::endl;
            std::cout << "fx_1080:   " << Ten::camera_fx_1080 << std::endl;
            std::cout << "fy_1080:   " << Ten::camera_fy_1080 << std::endl;
            std::cout << "cx_1080:   " << Ten::camera_cx_1080 << std::endl;
            std::cout << "cy_1080:   " << Ten::camera_cy_1080 << std::endl;

            std::cout << "\n==========================================================\n" << std::endl;
        }

        
        private:

        };

    }

}


#endif