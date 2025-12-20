#include "serial.h"
#include "openvino.h"
#include "threadpool.cpp"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "./point_lio/src/laserMapping2.h"
#include "lidar.h"
#include "camera.h"
#include "test.cpp"
#include "test2.cpp"
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


    int arr[31] = {1, 10, 11, 12, 13, 14, 15, 16, 17,18, 19, 2 ,20 ,21 ,22 ,23 ,24, 25 ,26 ,27 ,28 ,29, 3 ,30, 31, 4 ,5 ,6, 7, 8 ,9};
    std::vector<int> map;
    for(int i = 0; i < 31; i++)
    {
        map.push_back(arr[i]);
    }    
    Ten::Ten_yolo_cls detector("/home/maple/study2/merge_ws11/src/merge/model/yolo11-cls_gazebo/best", map);
    Ten::Ten_map map_s;

    ros::Rate sl(10);
    while(ros::ok())
    {
        // Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(odo);
        // std::cout<< "xyz: "<<tf._xyz._x << " " << tf._xyz._y << " " << tf._xyz._z << std::endl;
        // std::cout<< "rpy: "<<tf._rpy._roll << " " << tf._rpy._pitch << " " << tf._rpy._yaw << std::endl;

        cv::Mat img = camera.camera_read();
        sl.sleep();
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();

        std::unordered_map<int, cv::Mat> best_roi_image_ = zbuffer_handler.manage_odom_zbuffer_roi(odo, img);
        std::cout << "std::unordered_map<int, cv::Mat> best_roi_image_.size(): " << best_roi_image_.size() << std::endl;
        for (auto& x: best_roi_image_)
        {
            std::vector<Ten::Detection> results = detector.worker(x.second);
            if(results.empty())
            {
                continue;;
            }
            std::cout<< "x.first: " << x.first << "results[0].conf_: " << results[0].conf_ << std::endl;
            if(map_s.object_confidence_[x.first - 1] < results[0].conf_)
            {
                map_s.object_[x.first - 1] = results[0].cls_id_;
                map_s.object_confidence_[x.first - 1] = results[0].conf_;
                std::cout<< "map.object_[x.first]" << map_s.object_[x.first - 1] <<std::endl;
            }
        }
        cv::Mat debug = Ten::roi_best_zbuffer_debug2(best_roi_image_, map_s.object_);
        sensor_msgs::ImagePtr pub_debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug).toImageMsg();
        debug_roi_pub.publish(pub_debug_msg);
        // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // zbuffer_pub.publish(pub_msg);

        
    }

    camera.~Ten_camera();
    urcu_memb_unregister_thread();

}




// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     //test4();

//     Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws11/src/merge/src/livox_ros_driver2/config/MID360_config.json");
//     Ten::ThreadPool pool(1);
//     pool.enqueue(yolo);
//     //pool.enqueue(test_speed);
//     laserMapping();
//     Ten::Ten_lidar::GetInstance().~Ten_lidar();
//     return 0;
// }


////////////////test









void serial_send_lidar()
{
    urcu_memb_register_thread();
    Ten::Ten_serial::GetInstance();
    float arr[4] = {0};
    ros::Rate sl(100);
    while(ros::ok())
    {
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        float yaw = pose._rpy._yaw;
        arr[0] = pose._xyz._x  - 0.28*cos(yaw) + 0.28;
        arr[1] = pose._xyz._y - 0.28*sin(yaw);
        arr[2] = pose._xyz._z;
        arr[3] = yaw * 180.0 / M_PI;
        Ten::Ten_serial::GetInstance().serial_send(arr, 1, sizeof(arr));
        //std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;
        sl.sleep();
    }
    urcu_memb_unregister_thread();
}







// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     //test4();
//     Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws11/src/merge/src/livox_ros_driver2/config/MID360_config.json");
//     Ten::ThreadPool pool(1);
//     pool.enqueue(serial_send_lidar);

//     laserMapping();
//     Ten::Ten_lidar::GetInstance().~Ten_lidar();
//     return 0;
// }

// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     test_relocation();
//     return 0;
// }


int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "merge_node");
    ros::NodeHandle nh;

    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws13/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    //pool.enqueue(yolo);
    pool.enqueue(test_speed);
    laserMapping();
    Ten::Ten_lidar::GetInstance().~Ten_lidar();

    return 0;
}