#include "serial.cpp"
#include "openvino.cpp"
#include "threadpool.cpp"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "./point_lio/src/laserMapping2.h"
#include "lidar.h"
#include "camera.cpp"
//#include "test.cpp"
//#include "test2.cpp"
#include "zbuffer_package.cpp"



void yolo()
{
    urcu_memb_register_thread();
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher debug_roi_pub = it.advertise("pub_debug_roi",2);
    image_transport::Publisher zbuffer_pub = it.advertise("pub_image_topic", 2);

    Ten::Ten_zbuffer zbuffer_handler;
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    Ten::Ten_yolo detector("/home/maple/study2/merge_ws9/src/merge/model/yolo11-cls_gazebo/best", "cpu");
    Ten::Ten_map map;

    ros::Rate sl(30);
    while(ros::ok())
    {
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();

        // Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(odo);
        // std::cout<< "xyz: "<<tf._xyz._x << " " << tf._xyz._y << " " << tf._xyz._z << std::endl;
        // std::cout<< "rpy: "<<tf._rpy._roll << " " << tf._rpy._pitch << " " << tf._rpy._yaw << std::endl;

        cv::Mat img = camera.camera_read();
        std::unordered_map<int, cv::Mat> best_roi_image_ = zbuffer_handler.manage_odom_zbuffer_roi(odo, img);
        std::cout << "std::unordered_map<int, cv::Mat> best_roi_image_.size(): " << best_roi_image_.size() << std::endl;
        for (auto& x: best_roi_image_)
        {
            std::vector<Ten::Detection> results = detector.worker(x.second);
            if(results.empty())
            {
                break;
            }
            if(map.object_confidence_[x.first] < results[0].conf_)
            {
                map.object_[x.first] = results[0].cls_id_;
                map.object_confidence_[x.first] = results[0].conf_;
                std::cout<< "map.object_[x.first]" << map.object_[x.first] <<std::endl;
            }
        }
        cv::Mat debug = Ten::roi_best_zbuffer_debug2(best_roi_image_, map.object_);
        sensor_msgs::ImagePtr pub_debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug).toImageMsg();
        debug_roi_pub.publish(pub_debug_msg);
        // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // zbuffer_pub.publish(pub_msg);

        sl.sleep();
    }

    camera.~Ten_camera();
    urcu_memb_unregister_thread();

}

void test_yolo2()
{
    //创建检测器
    ros::NodeHandle nh("~");
    //Ten::Ten_yolo detector("/home/maple/study2/merge_ws10/src/merge/model/houli_1023_v5s/best", "cpu", 1);
    Ten::Ten_yolo detector("/home/maple/study2/merge_ws9/src/merge/model/yolo11-cls_gazebo/best", "cpu");
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    ros::Publisher debug_pub_ = nh.advertise<sensor_msgs::Image>("/yolo/debug_image", 2);

    ros::Rate sl(30);
    while(ros::ok())
    {
        cv::Mat img = camera.camera_read();
        //调用worker函数
        std::vector<Ten::Detection> results = detector.worker(img);
        // for(int i = 0; i < results.size(); i++)
        // {
        //     Ten::Detection best = results[i];
        //     float x1 = best.cx_ - best.w_ / 2;
        //     float x2 = best.cx_ + best.w_ / 2;
        //     float y1 = best.cy_ - best.h_ / 2;
        //     float y2 = best.cy_ + best.h_ / 2;
        //     cv::rectangle(img, 
        //     cv::Point(x1, y1),
        //     cv::Point(x2, y2),
        //     cv::Scalar(0,255,0), 2);
        //     cv::putText(img, "cls: " + std::to_string(best.cls_id_) + "cof: " + std::to_string(best.conf_), cv::Point(best.cx_, best.cy_), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255), 2);
        // }
        if(results.size() <= 0) continue;;
        Ten::Detection best = results[0];
        cv::putText(img, "cls: " + std::to_string(best.cls_id_) + "cof: " + std::to_string(best.conf_), cv::Point(1920/2,1080/2), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255), 2);
        std::cout<< "results.size(): " << results.size() << std::endl;

        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        debug_pub_.publish(debug_msg);
        sl.sleep();
    }
    camera.~Ten_camera();
    std::cout<< "camera.~Ten_camera();" << std::endl;
}


// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     //test4();

//     Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws9/src/merge/src/livox_ros_driver2/config/MID360_config.json");
//     Ten::ThreadPool pool(1);
//     pool.enqueue(yolo);
//     //pool.enqueue(test_speed);
//     laserMapping();
//     Ten::Ten_lidar::GetInstance().~Ten_lidar();
//     return 0;
// }

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "merge_node");
    ros::NodeHandle nh;

    test_yolo2();


    return 0;
}