#ifndef __INSPECTION_H_
#define __INSPECTION_H_


#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include "./../method_math.h"
#include "./../usb_cam_multi.h"
#include "../parameter/parameter.h"
#include <cmath>

namespace Ten
{
    class inspection
    {
    public:
        void camera()
        {
            std::vector<int> idxs = {Ten::_usb_device_num1_, Ten::_usb_device_num2_};
            Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,30);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                cv::Mat img_left = usbcam.camera_read(Ten::_usb_device_num1_);
                cv::Mat img_right = usbcam.camera_read(Ten::_usb_device_num2_);
                cv::Mat debug = concatAndLabel(img_left, img_right);
                cv::imshow("debugrl", debug);
                int key = cv::waitKey(30);
                if(key == 'q')
                break;
            }
        }

    private:

    /**
     * @brief 核心图像处理函数：两张图水平拼接→缩放至640x480→左右添加标注文字
     * @param img_left  左侧输入图像
     * @param img_right 右侧输入图像
     * @param text_left 左侧标注文字
     * @param text_right 右侧标注文字
     * @param output_size 最终输出尺寸（宽, 高），默认640x480
     * @return 处理后的图像
     */
    cv::Mat concatAndLabel(cv::Mat& img_left, cv::Mat& img_right,
                        const std::string& text_left = "left",
                        const std::string& text_right = "right",
                        cv::Size output_size = cv::Size(640, 480))
    {
        // 1. 空图替换为纯黑640*480
        cv::Mat left_fixed, right_fixed;
        bool left_empty = img_left.empty();
        bool right_empty = img_right.empty();

        if (left_empty)
        {
            left_fixed = cv::Mat::zeros(480, 640, CV_8UC3);
        }
        else
        {
            cv::resize(img_left, left_fixed, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
        }

        if (right_empty)
        {
            right_fixed = cv::Mat::zeros(480, 640, CV_8UC3);
        }
        else
        {
            cv::resize(img_right, right_fixed, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
        }

        // 2. 水平拼接为 1280x480 大图
        cv::Mat concat_img;
        cv::hconcat(left_fixed, right_fixed, concat_img);

        // 3. 缩放回目标尺寸 640x480
        cv::Mat resized_img;
        cv::resize(concat_img, resized_img, output_size, 0, 0, cv::INTER_AREA);

        // 4. 绘制文字：全部红色，无描边
        const int font_face = cv::FONT_HERSHEY_SIMPLEX;
        const double font_scale = 1.2;
        const int text_thickness = 2;
        const cv::Scalar text_color = cv::Scalar(0, 0, 255); // 统一红色(BGR)
        // 文字距离画面顶部偏移量，可自行调整
        const int top_offset = 40;

        int baseline = 0;
        int img_w = resized_img.cols;
        int img_h = resized_img.rows;
        // 拼接缩放后整体宽640，左右原图各占一半宽度
        int half_w = img_w / 2;

        // ========== 左侧文字：左半张图水平居中，顶部偏上 ==========
        cv::Size text_size_left = cv::getTextSize(text_left, font_face, font_scale, text_thickness, &baseline);
        int left_text_x = half_w / 2 - text_size_left.width / 2;
        int text_y = top_offset + text_size_left.height;
        cv::Point left_pos(left_text_x, text_y);
        cv::putText(resized_img, text_left, left_pos, font_face, font_scale, text_color, text_thickness, cv::LINE_AA);

        // ========== 右侧文字：右半张图水平居中，顶部偏上 ==========
        cv::Size text_size_right = cv::getTextSize(text_right, font_face, font_scale, text_thickness, &baseline);
        int right_text_x = half_w + half_w / 2 - text_size_right.width / 2;
        cv::Point right_pos(right_text_x, text_y);
        cv::putText(resized_img, text_right, right_pos, font_face, font_scale, text_color, text_thickness, cv::LINE_AA);

        return resized_img;
    }


    
    };
}

#endif
