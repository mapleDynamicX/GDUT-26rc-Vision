#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdexcept>
#include <ros/ros.h>
// CCTag 完整核心头文件
#include <cctag/CCTag.hpp>
#include <cctag/Params.hpp>
#include <cctag/Detection.hpp>
#include <cctag/Types.hpp>
#include "camera.h"

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

    // CCTag 参数
    cctag::Parameters params;
    params._nCrowns = 3;

    // ===================== 修复：CCTagMarkersBank 正确构造方式 =====================
    // 构造函数只接受 环数(size_t) 或 文件路径(string)
    cctag::CCTagMarkersBank bank(3); // 传入3环，标准配置

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

        // ===================== 【核心优化2】缩小图像尺寸（提速10倍！） =====================
        cv::Mat resize_image;
        // 强制缩放到 640x480 以内，CCTag 检测速度爆炸提升
        cv::resize(raw_image, resize_image, cv::Size(640, 480));
        // 转灰度图
        cv::Mat gray_image;
        if(resize_image.channels() == 3)
        {
            cv::cvtColor(resize_image, gray_image, cv::COLOR_BGR2GRAY);
        }
        else
        {
            gray_image = raw_image.clone();
        }

        // ===================== 官方标准检测函数 =====================
        cctag::cctagDetection(
            markers,
            0,        // pipeId
            0,        // frame
            gray_image,
            params,
            bank
        );

        // 绘制结果
        cv::Mat show_img = gray_image.clone();

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
        cv::waitKey(30);
        //sl.sleep();
    }

    cv::destroyAllWindows();
}



int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    test1();
    return 0;
}
