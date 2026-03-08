#ifndef __VISION_CPP_
#define __VISION_CPP_




#include "serial.h"
#include "openvino.h"
#include "threadpool.h"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "./point_lio/src/laserMapping2.h"
#include "lidar.h"
#include "camera.h"
#include "test.cpp"
#include "test2.cpp"
#include "relocation.h"
#include "coordinate.h"
#include "recognition/camera_calibration.h"
#include "recognition/world_to_camera.h"
#include "recognition/occlusion_handing.h"
#include "velocity.h"
#include "calibration.h"
#include "log/logger.h"
#include "yolo/yolo_v5.h"
#include "yolo/yolo_han.h"
#include "orb/orb_overall_exhaust.h"


void orb_test()
{
    urcu_memb_register_thread();
    //orb
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);

    cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);

    Ten::ORB::orb_optimize_exhaust ooe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);
    std::vector<int> place = ooe.getplace(oees);



    std::cout << "place: ";
    for(auto it : place)
    {
        std::cout << it << ",";
    }
    std::cout << std::endl;


    std::vector<double> losses = ooe.get_loss();
    for(size_t i = 0; i < losses.size(); i++)
    {
        std::cout << "loss[" << i << "]: " << losses[i] << std::endl;
    }

    std::vector<Ten::ORB::orb_exhaust_element> oprt = ooe.get_RT(oees);
    log.record_imageRT(oees);
    log.record_imageRT(oprt);

    //检验
    int exist_box[12] = {0};
    for(size_t k = 0; k < 12 && k < place.size(); k++)
    {
        exist_box[k] = place[k];
    }

    //zubbfer
    for(size_t i = 0; i < oees.size(); i++)
    {
        Ten::init_3d_box world_point;
        Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oees[i].rvec_, oees[i].tvec_);
        Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
        world_point.pcl_to_C();

        Ten::Ten_occlusion_handing zbuffer;
        zbuffer.set_exist_boxes(exist_box);
        zbuffer.set_box_lists_(oees[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
        cv::Mat debug1  = cv::Mat::zeros(480, 640, CV_8UC3); 
        zbuffer.set_debug_roi_image(world_point.box_lists_, debug1);
        cv::imshow("debug1", debug1);
        cv::waitKey(0);
    }

    //zubbfer
    for(size_t i = 0; i < oprt.size(); i++)
    {
        Ten::init_3d_box world_point;
        Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oprt[i].rvec_, oprt[i].tvec_);
        Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
        world_point.pcl_to_C();

        Ten::Ten_occlusion_handing zbuffer;
        zbuffer.set_exist_boxes(exist_box);
        zbuffer.set_box_lists_(oprt[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
        cv::Mat debug2  = cv::Mat::zeros(480, 640, CV_8UC3); 
        zbuffer.set_debug_roi_image(world_point.box_lists_, debug2);
        cv::imshow("debug2", debug2);
        cv::waitKey(0);
    }
    
    urcu_memb_unregister_thread();
}

#endif


