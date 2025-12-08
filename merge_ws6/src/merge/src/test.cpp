
#ifndef __TEST_H_
#define __TEST_H_
#include "openvino.cpp"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "threadpool.cpp"
#include "./point_lio/src/laserMapping2.h"
#include "serial.cpp"
#include "camera.cpp"
#include "methon_math.cpp"

void test()
{
    //创建检测器
    Ten::Ten_yolo detector("/home/maple/study2/merge_ws3/src/merge/model/weights/best", "cpu");

    
    //加载图片
    cv::Mat img = cv::imread("/home/maple/study2/merge_ws3/src/merge/image/1.jpg");
    

    if (img.empty()) {
        std::cout << "无法加载图像！" << std::endl;
        return;
    }

    std::cout << "成功加载" << std::endl;
    std::cout << "图片尺寸: " << img.cols << "x" << img.rows << std::endl;


    
    //调用worker函数
    std::vector<Ten::Detection> results = detector.worker(img);

    
    std::cout << 0 << std::endl;

    
    //输出结果
    if (results.empty()) {
        std::cout << "没检测到目标" << std::endl;
    } 
    else {
        std::cout << "检测到 " << results.size() << " 个目标：" << std::endl;
        Ten::Detection det;
        for (int i = 0; i < results.size(); i++) {
            det = results[i];
            std::cout << "类别" << det.cls_id_ << ": 置信度" << det.conf_ 
                      << ", 位置(" << det.cx_ << "," << det.cy_ 
                      << "), 尺寸" << det.w_ << "x" << det.h_ << std::endl;
        }
    }
}

void test2()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws3/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    while (ros::ok()) { usleep(10000); }

}

void test3()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws3/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(laserMapping);

    while (ros::ok()) { usleep(10000); }

}

void test4()
{
    Ten::Ten_serial::GetInstance("/dev/ttyUSB0");
    int arr[3] = {1,2,3};
    ros::Rate sl(1);
    while(ros::ok())
    {
        Ten::Ten_serial::GetInstance().serial_send(arr, 1, sizeof(arr));
        sl.sleep();
    }
    
}

void test5()
{
    Ten::Ten_serial::GetInstance("/dev/ttyUSB0");
    float arr[10] = {0};
    ros::Rate sl(1);
    uint8_t frameid = 0;
    uint8_t length  = 0;
    while(ros::ok())
    {
        bool ok = Ten::Ten_serial::GetInstance().serial_read(arr, frameid, length);
        if(!ok)
        {
            std::cout<<"no receive! "<<std::endl;
            sl.sleep();
            continue;
        }
        std::cout<<"frameid: "<<(int)frameid<<std::endl;
        length = length / sizeof(float);
        std::cout<<"length: "<< (int)length <<std::endl;
        for(int i = 0; i < length; i++)
        {
            std::cout << arr[i] << " ";
        } 
        std::cout<<std::endl;
        sl.sleep();
    }
    
}

void test6(std::string s = "camera")
{
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();

    while(ros::ok())
    {
        cv::Mat* img = camera.camera_read(1);
        // cv::imshow(s.c_str(), *img);
        // cv::waitKey(30);
        delete img;
    }
    

}

void test7()
{
    urcu_memb_register_thread();
    ros::Rate sl(10);
    while(ros::ok())
    {
        Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
        std::cout<< "xyz: "<<tf._xyz._x << " " << tf._xyz._y << " " << tf._xyz._z << std::endl;
        std::cout<< "rpy: "<<tf._rpy._roll << " " << tf._rpy._pitch << " " << tf._rpy._yaw << std::endl;
        sl.sleep();
    }
    urcu_memb_unregister_thread();

}

void test8()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws3/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(2);
    pool.enqueue(laserMapping);
    pool.enqueue(test7);
    while (ros::ok()) { usleep(10000); }
}

void test9()
{
    Ten::ThreadPool pool(3);
    pool.enqueue(test6, "camera1");
    pool.enqueue(test6, "camera2");
    pool.enqueue(test6, "camera3");
}

#endif

