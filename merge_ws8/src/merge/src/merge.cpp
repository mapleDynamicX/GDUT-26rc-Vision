#include "serial.cpp"
#include "openvino.cpp"
#include "threadpool.cpp"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "./point_lio/src/laserMapping2.h"
#include "lidar.h"
#include "camera.cpp"
//#include "test.cpp"
#include "zbuffer_package.cpp"



void yolo()
{
    urcu_memb_register_thread();
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher debug_roi_pub = it.advertise("pub_debug_roi",2);

    Ten::Ten_zbuffer zbuffer_handler;
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    Ten::Ten_yolo detector("/home/maple/study2/merge_ws8/src/merge/model/weights/best", "cpu");
    Ten::Ten_map map;
    while(ros::ok())
    {
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        cv::Mat img = camera.camera_read();
        std::unordered_map<int, cv::Mat> best_roi_image_ = zbuffer_handler.manage_odom_zbuffer_roi(odo, img);
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
            }
        }
        cv::Mat debug = Ten::roi_best_zbuffer_debug2(best_roi_image_, map.object_);
        sensor_msgs::ImagePtr pub_debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug).toImageMsg();
        debug_roi_pub.publish(pub_debug_msg);
    }

    camera.~Ten_camera();
    urcu_memb_unregister_thread();

}


int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "merge_node");
    ros::NodeHandle nh;

    //test4();

    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws8/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(yolo);
    laserMapping();
    Ten::Ten_lidar::GetInstance().~Ten_lidar();
    return 0;
}