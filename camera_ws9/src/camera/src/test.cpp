#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdexcept>
#include <ros/ros.h>
// CCTag 完整核心头文件
#include "Tencctag/cctagbase.h"
#include "Tencctag/cctagresolver.h"
#include "camera.h"
#include "Tencctag/cctagcoordinate.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Tencctag/cctagspeed.h"

/**
 * @brief  官方标准CCTag 3环标记生成（环宽比例编码ID，可被算法识别）
 * @param size    图像尺寸（正方形，如64/128/256）
 * @param savePath 保存路径
 * @param id      标记ID（支持 0~9 标准3环ID）
 * @return 生成成功返回true，失败返回false
 * @note   3环结构：外黑环 → 白环 → 内黑环 → 白中心（黑白交替，环宽比例编码）
 */
bool generateCCTag3Ring(int size, const std::string& savePath, int id)
{
    // ===================== 日志输出 =====================
    std::cout << "\n[INFO] 开始生成 标准CCTag 3环标记" << std::endl;
    std::cout << "[INFO] 标记ID: " << id << std::endl;
    std::cout << "[INFO] 图像尺寸: " << size << "x" << size << std::endl;
    std::cout << "[INFO] 保存路径: " << savePath << std::endl;

    // ===================== 参数校验 =====================
    if (size < 32) {
        std::cerr << "[ERROR] 尺寸过小，最小支持32x32" << std::endl;
        return false;
    }
    // 官方标准3环预定义ID范围：0 ~ 9
    if (id < 0 || id > 9) {
        std::cerr << "[ERROR] ID不支持！仅支持 0~9 标准3环ID" << std::endl;
        return false;
    }

    // ===================== 官方CCTag 3环 环宽比例表（核心编码） =====================
    // 格式：[ID] = {外环宽, 中环宽, 内环宽}  总和=总半径
    std::vector<std::vector<int>> cctagRatio = {
        {4,2,1},   // ID0
        {3,3,1},   // ID1
        {3,2,2},   // ID2
        {2,4,1},   // ID3
        {2,3,2},   // ID4
        {2,2,3},   // ID5
        {1,5,1},   // ID6
        {1,4,2},   // ID7
        {1,3,3},   // ID8
        {1,2,4}    // ID9
    };

    // 获取当前ID的环宽比例
    int outerW = cctagRatio[id][0];  // 外环宽度
    int midW   = cctagRatio[id][1];  // 中环宽度
    int innerW = cctagRatio[id][2];  // 内环宽度
    int sumW   = outerW + midW + innerW;

    std::cout << "[INFO] ID" << id << " 环宽比例: 外环=" << outerW 
              << " 中环=" << midW << " 内环=" << innerW << std::endl;

    // ===================== 创建图像 =====================
    cv::Mat img = cv::Mat::zeros(cv::Size(size, size), CV_8UC1);
    img.setTo(255);  // 背景：白色（CCTag标准背景）
    cv::Point center(size / 2, size / 2);
    int maxR = size / 2 - 3;  // 最大半径（留边距）

    // 按比例计算各层半径
    int r1 = maxR;                          // 最外层圆半径
    int r2 = maxR * (sumW - outerW) / sumW;  // 第二层圆半径
    int r3 = maxR * innerW / sumW;           // 最内层圆半径

    // ===================== 绘制同心圆（平滑抗锯齿） =====================
    // 1. 最外层：黑色
    cv::circle(img, center, r1, cv::Scalar(0), -1, cv::LINE_AA);
    // 2. 中间层：白色
    cv::circle(img, center, r2, cv::Scalar(255), -1, cv::LINE_AA);
    // 3. 最内层：黑色
    cv::circle(img, center, r3, cv::Scalar(0), -1, cv::LINE_AA);

    std::cout << "[INFO] 绘制完成 | 外半径:" << r1 << " 中半径:" << r2 << " 内半径:" << r3 << std::endl;

    // ===================== 保存图像 =====================
    bool ret = cv::imwrite(savePath, img);
    if (ret) {
        std::cout << "[SUCCESS] CCTag ID" << id << " 保存成功！" << std::endl;
    } else {
        std::cerr << "[ERROR] 保存失败，请检查路径权限" << std::endl;
    }

    return ret;
}


void test()
{
    //generateCCTag3Ring(640, "/home/maple/study3/camera_ws/src/camera/image/cctag5.png", 5);
    std::string s = "/home/maple/study3/camera_ws/src/camera/image/cctag";
    for(size_t i = 0; i < 10; i++)
    {
        generateCCTag3Ring(640, s + std::to_string(i) + std::string(".png"), i);
    }
    // 执行完成后直接退出
}






void test1()
{
    // 相机实例
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
    ros::Rate sl(10);

    // CCTag 参数（保留你设置的所有参数）
    cctag::Parameters params;
    // ✅ 修复1：必须指定3环（核心！你注释掉了，导致算法异常）
    params._nCrowns = 3;
    // 保留你的自定义性能参数
    params._maximumNbSeeds = 9;
    params._maximumNbCandidatesLoopTwo = 3;
    //params._minIdentProba = 0.35;
    std::cout << "params._maximumNbSeeds: " << params._maximumNbSeeds <<std::endl;
    std::cout << "params._maximumNbCandidatesLoopTwo: " << params._maximumNbCandidatesLoopTwo <<std::endl;
    //std::cout << "params._minIdentProba: " << params._minIdentProba <<std::endl;

    // 3环标记库
    cctag::CCTagMarkersBank bank(3); 

    // 官方指定结果容器
    cctag::CCTag::List markers;

    // 显示窗口
    cv::namedWindow("CCTag Detection", cv::WINDOW_NORMAL);

    while(ros::ok())
    {
        cv::Mat raw_image = camera.camera_read();

        if(raw_image.empty())
        {
            ROS_WARN("无相机图像");
            sl.sleep();
            continue;
        }

        // ===================== 图像预处理 =====================
        cv::Mat resize_image = raw_image;
        // ✅ 修复2：正确执行图像缩放（你原来只是赋值，没有缩放，打开后速度更快）
        //cv::resize(raw_image, resize_image, cv::Size(640, 360));
        
        // 转灰度图
        cv::Mat gray_image;
        if(resize_image.channels() == 3)
        {
            cv::cvtColor(resize_image, gray_image, cv::COLOR_BGR2GRAY);
        }
        else
        {
            // ✅ 修复3：统一使用缩放后的图像，避免尺寸不匹配
            gray_image = resize_image.clone();
        }

        // ✅ 修复4：每帧清空上一次的检测结果（防止内存溢出+崩溃）
        markers.clear();

        // ===================== 异常捕获：彻底解决程序崩溃 =====================
        try
        {
            // 官方标准检测函数
            cctag::cctagDetection(
                markers,
                0,        // pipeId
                0,        // frame
                gray_image,
                params,
                bank
            );
        }
        catch(const std::exception& e)
        {
            // 打印错误信息，跳过当前帧，程序继续运行
            ROS_ERROR("CCTag 检测异常: %s", e.what());
            continue;
        }

        // 绘制结果（clone防止修改原图）
        //cv::Mat show_img = gray_image.clone();
        cv::Mat show_img = gray_image;

        for(auto& marker : markers)
        {
            // 只绘制有效标记
            if(marker.getStatus() != 1)
                continue;

            // 坐标转换
            auto& cctag_pt = marker.centerImg();
            cv::Point2d center(
                static_cast<double>(cctag_pt.x()),
                static_cast<double>(cctag_pt.y())
            );

            // 画圆心 + ID
            cv::circle(show_img, center, 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(show_img,
                "ID: " + std::to_string(marker.id()),
                cv::Point(center.x + 10, center.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("CCTag Detection", show_img);
        cv::waitKey(1);
    }

    // 释放窗口
    cv::destroyAllWindows();
}

void test2()
{
    // 相机实例
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance(640,480,60);
    Ten::Tencctag::cctagresolver cr(0.049 / 2);
    // 显示窗口
    cv::namedWindow("CCTag Detection", cv::WINDOW_NORMAL);
    while(ros::ok())
    {
        cv::Mat raw_image = camera.camera_read();
        cv::Mat resize;
        //cv::resize(raw_image, resize, cv::Size(640,360));
        cr.debug(cr.resolver(raw_image), raw_image);
        cv::imshow("CCTag Detection", raw_image);
        cv::waitKey(1);
    }
    // 释放窗口
    cv::destroyAllWindows();

}


void test3()
{
    // ros::NodeHandle nh("~");
    // image_transport::ImageTransport it(nh);
    // // ✅ 修复3：队列改大（10）
    // image_transport::Publisher pub_img = it.advertise("pub_image_topic", 10);

    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
    camera.reset_camera(640,480,60);
    Ten::Tencctag::cctagcoordinate cd(0.049 / 2);
    size_t num = 0;
    //ros::Rate sl(60);
    auto start = std::chrono::high_resolution_clock::now();
    cv::namedWindow("CCTag Detection", cv::WINDOW_NORMAL);
    while(ros::ok())
    {
        
        double biasx = 0.0, biasy = 0.0;
        Ten::XYZRPY pose;

        cv::Mat raw_image = camera.camera_read();
        
        // ✅ 新增：防止空图崩溃
        //if(raw_image.empty()) { usleep(16);; continue; }

        cd.getpose(raw_image, biasx, biasy, pose);

        // ✅ 修复2：必须加时间戳（ROS强制要求）
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera";

        auto end = std::chrono::high_resolution_clock::now();
        // sensor_msgs::ImagePtr pub_debug_img_msg = cv_bridge::CvImage(header, "bgr8", raw_image).toImageMsg();
        // pub_img.publish(pub_debug_img_msg);
        // ✅ 修复1：最关键！必须加这一行
        //ros::spinOnce();
        num++;
        if(num >= 30)
        {
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << "hz: " << 1000.0 / std::chrono::duration<double, std::milli>(end - start).count() * 30.0<< std::endl;
            start = std::chrono::high_resolution_clock::now();
            num = 0;
        }

        //cv::imshow("CCTag Detection", raw_image);
        //cv::waitKey(1);
        //sl.sleep();
        //usleep(16000);
    }
    // 释放窗口
    cv::destroyAllWindows();
}

// void test4()
// {
//     Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
//     camera.reset_camera(640,480,60);
//     Ten::Tencctag::cctagspeed sp(2, 4, 0.049);
//     cv::namedWindow("CCTag Detection", cv::WINDOW_NORMAL);
//     auto start = std::chrono::high_resolution_clock::now();
//     int num = 0;
//     while(ros::ok())
//     {
//         sp.input_image(camera.camera_read());
//         Ten::Tencctag::TaskOutput result;
//         if (sp.get_latest_result(result)) {
//             cv::imshow("CCTag Detection", result.image_);
//             auto end = std::chrono::high_resolution_clock::now();
//             // sensor_msgs::ImagePtr pub_debug_img_msg = cv_bridge::CvImage(header, "bgr8", raw_image).toImageMsg();
//             // pub_img.publish(pub_debug_img_msg);
//             // ✅ 修复1：最关键！必须加这一行
//             //ros::spinOnce();
//             num++;
//             if(num >= 30)
//             {
//                 auto end = std::chrono::high_resolution_clock::now();
//                 std::cout << "hz: " << 1000.0 / std::chrono::duration<double, std::milli>(end - start).count() * 30.0<< std::endl;
//                 start = std::chrono::high_resolution_clock::now();
//                 num = 0;
//             }
//             cv::waitKey(60);
//         }
//         else
//         {
//             usleep(20000);
//         }
//     }
// }


void test4()
{
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
    camera.reset_camera(640,480,30);
    Ten::Tencctag::cctagspeed sp(2, 4, 0.049/2);
    while(ros::ok())
    {
        sp.input_image(camera.camera_read());
        usleep(33000);
    }
}




int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    test4();
    return 0;
}
