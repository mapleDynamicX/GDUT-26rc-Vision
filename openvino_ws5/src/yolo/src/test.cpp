#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "./yolo/yolo_v11_cls.h"
#include "./yolo/yolo_v5.h"
#include "./yolo/yolo_v11.h"
#include "./yolo/yolo_han.h"
#include "./yolo/yolo_han2.h"
#include <iostream>
#include <regex>
#include <dirent.h>   // Linux目录遍历
#include <sys/stat.h> // Linux文件状态
#include <filesystem>
#include "./yolo/yolo_26obb.h"
#include "camera.h"

void test()
{
    int arr[31] = {1, 10, 11, 12, 13, 14, 15, 16, 17,18, 19, 2 ,20 ,21 ,22 ,23 ,24, 25 ,26 ,27 ,28 ,29, 3 ,30, 31, 4 ,5 ,6, 7, 8 ,9};
    std::vector<int> map;
    for(int i = 0; i < 31; i++)
    {
        map.push_back(arr[i]);
    }
    //创建检测器
    Ten::yolo::yolo_v11_cls detector("/home/maple/study2/model/yolo11-cls_gazebo/best", "cpu", map);

    
    //加载图片
    cv::Mat img = cv::imread("/home/maple/study2/model/test_map50_cla_/images_13.png");
    

    if (img.empty()) {
        std::cout << "无法加载图像！" << std::endl;
        return;
    }

    std::cout << "成功加载" << std::endl;
    std::cout << "图片尺寸: " << img.cols << "x" << img.rows << std::endl;


    
    //调用worker函数
    std::vector<Ten::yolo::Detection> results = detector.worker(img);

    
    std::cout << 0 << std::endl;

    
    //输出结果
    if (results.empty()) {
        std::cout << "没检测到目标" << std::endl;
    } 
    else {
        std::cout << "检测到 " << results.size() << " 个目标：" << std::endl;
        Ten::yolo::Detection det;
        for (int i = 0; i < results.size(); i++) {
            det = results[i];
            std::cout << "类别" << det.cls_id_ << ": 置信度" << det.conf_ 
                      << ", 位置(" << det.cx_ << "," << det.cy_ 
                      << "), 尺寸" << det.w_ << "x" << det.h_ << std::endl;
        }
    }
}

void calculate_square(const Ten::yolo::Detection& box, float& x1, float& y1, float& x2, float& y2)
{
    x1 = box.cx_ - box.w_ / 2.0f;
    y1 = box.cy_ - box.h_ / 2.0f;
    x2 = box.cx_ + box.w_ / 2.0f;
    y2 = box.cy_ + box.h_ / 2.0f; 

}

void draw_boxes(cv::Mat& img, const std::vector<Ten::yolo::Detection>& detections) {

    if (img.empty()) {
        ROS_WARN("输入图像为空，跳过绘制");
        return;
    }
    
    // for (const auto& area : targetAreas) {
    //     cv::Rect area_rect(cv::Point(area.x1, area.y1), cv::Point(area.x2, area.y2));
    //     cv::rectangle(img, area_rect, cv::Scalar(255, 0, 0), 2);
    //     // 标注区域ID
    //     cv::putText(img, "Area " + std::to_string(area.id), 
    //                 cv::Point(area.x1, area.y1 - 5), 
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    // }

    // 2. 绘制YOLO检测框（红色 BGR: 0,0,255，线宽2）
    for (size_t i = 0; i < detections.size(); ++i) {
        float box_x1, box_y1, box_x2, box_y2;
        calculate_square(detections[i], box_x1, box_y1, box_x2, box_y2);
        cv::Rect detect_rect(cv::Point(box_x1, box_y1), cv::Point(box_x2, box_y2));
        cv::rectangle(img, detect_rect, cv::Scalar(0, 0, 255), 2);
        // 标注检测框编号
        cv::putText(img, "Det " + std::to_string(i+1), 
                    cv::Point(box_x1, box_y1 - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }
}

void test2()
{
    Ten::yolo::yolo_v5 detector("/home/maple/study3/li/corner/best", "cpu", 0.3, 0.3, 0);
    //Ten::yolo::yolo_v11 detector("/home/maple/study2/model/shelf_model_yolo12/best", "cpu", 0, 0);
    cv::Mat img1 = cv::imread("/home/maple/study3/maple/place_debug/3/test/2/image_1.png");
    if (img1.empty()) {
        ROS_ERROR("图片读取失败，请检查路径！");
        return;
    }
    std::vector<Ten::yolo::Detection> detections1 = detector.worker(img1);
    std::cout<< "detections1.size()" << detections1.size() << std::endl;
    draw_boxes(img1, detections1);
    cv::imshow("img", img1);
    cv::waitKey(0);
}

std::vector<cv::Mat> loadSortedImages(const std::string& folderPath) {
    // 1. 检查文件夹是否存在
    namespace fs = std::filesystem;
    if (!fs::is_directory(folderPath)) {
        throw std::invalid_argument("文件夹不存在: " + folderPath);
    }

    // 定义图片后缀（可根据需要扩展，如.tiff/.gif）
    const std::vector<std::string> imageExts = {".jpg", ".jpeg", ".png", ".bmp"};
    // 用于临时存储：键=提取的整数，值=图片路径（确保1-12每个数对应一个路径）
    std::vector<std::string> imgPaths(12, "");
    // 正则表达式：匹配第一个连续的数字序列（提取文件名中的第一个整数）
    const std::regex numRegex(R"(\d+)");

    // 2. 遍历文件夹中的所有文件
    for (const auto& entry : fs::directory_iterator(folderPath)) {
        // 跳过目录，只处理文件
        if (!entry.is_regular_file()) {
            continue;
        }

        // 获取文件路径和后缀
        const fs::path filePath = entry.path();
        const std::string ext = filePath.extension().string();
        // 转换为小写，避免大小写问题（如.JPG/.jpg）
        std::string extLower = ext;
        std::transform(extLower.begin(), extLower.end(), extLower.begin(), ::tolower);

        // 3. 过滤出图片文件
        bool isImage = false;
        for (const auto& e : imageExts) {
            if (extLower == e) {
                isImage = true;
                break;
            }
        }
        if (!isImage) {
            continue;
        }

        // 4. 提取文件名中的第一个整数
        const std::string fileName = filePath.filename().string();
        std::smatch match;
        if (!std::regex_search(fileName, match, numRegex)) {
            std::cerr << "警告：文件 " << fileName << " 中未找到整数，已跳过" << std::endl;
            continue;
        }

        // 转换为整数并验证范围
        int imgNum = std::stoi(match.str());
        if (imgNum < 1 || imgNum > 12) {
            std::cerr << "警告：文件 " << fileName << " 提取的整数 " << imgNum 
                      << " 不在1-12范围内，已跳过" << std::endl;
            continue;
        }

        // 检查是否重复（同一数字对应多个文件）
        if (!imgPaths[imgNum - 1].empty()) {
            throw std::runtime_error("发现重复整数 " + std::to_string(imgNum) + 
                                     " 的图片：" + imgPaths[imgNum - 1] + " 和 " + fileName);
        }

        // 存储路径（imgNum-1是因为vector索引从0开始，对应1→0，12→11）
        imgPaths[imgNum - 1] = filePath.string();
    }

    // 5. 验证是否收集到12个有效路径
    for (int i = 0; i < 12; ++i) {
        if (imgPaths[i].empty()) {
            throw std::runtime_error("未找到整数为 " + std::to_string(i + 1) + " 的图片");
        }
    }

    // 6. 按1-12顺序读取图片到cv::Mat容器
    std::vector<cv::Mat> result;
    result.reserve(12); // 预分配空间，提升性能
    for (int i = 0; i < 12; ++i) {
        cv::Mat img = cv::imread(imgPaths[i], cv::IMREAD_COLOR);
        if (img.empty()) {
            throw std::runtime_error("图片读取失败：" + imgPaths[i]);
        }
        result.push_back(img);
    }

    return result;
}

void printHanContainer(const std::vector<Ten::yolo::han>& hanContainer) {
    // 打印表头，提升可读性
    std::cout << std::fixed << std::setprecision(2);  // 浮点数保留2位小数（可按需调整）
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "序号 | invalid | valid_empty | valid_exist" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 遍历容器（范围for循环，简洁且安全）
    int index = 1;  // 序号（从1开始，符合直观习惯）
    for (const auto& item : hanContainer) {
        // 格式化打印每个属性，对齐输出
        std::cout << std::setw(3) << index << " | "
                  << std::setw(7) << item.invalid_ << " | "
                  << std::setw(10) << item.valid_empty_ << " | "
                  << std::setw(10) << item.valid_exist_ << std::endl;
        index++;
    }

    std::cout << "----------------------------------------" << std::endl;
}

void printHanContainer2(const std::vector<Ten::yolo::han2>& hanContainer) {
    // 打印表头，提升可读性
    std::cout << std::fixed << std::setprecision(2);  // 浮点数保留2位小数（可按需调整）
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "序号 | valid_empty | valid_exist" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 遍历容器（范围for循环，简洁且安全）
    int index = 1;  // 序号（从1开始，符合直观习惯）
    for (const auto& item : hanContainer) {
        // 格式化打印每个属性，对齐输出
        std::cout << std::setw(3) << index << " | "
                  << std::setw(7) << item.valid_empty_ << " | "
                  << std::setw(10) << item.valid_exist_<< std::endl;
        index++;
    }

    std::cout << "----------------------------------------" << std::endl;
}


void test3()
{
    Ten::yolo::yolo_han2 detector("/home/maple/study3/han/model_2/yolo11s_roi12_atten_2", "cpu");
    std::vector<cv::Mat> image = loadSortedImages("/home/maple/study2/han/roi_images/roi_1");
    printHanContainer2(detector.worker(image));
}

// void test4()
// {
//     Ten::yolo::yolo_26obb yobb("/home/maple/study3/li/best_openvino_model_op13/model", "cpu", 0.9);
//     Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
//     ros::Rate sl(30);
//     while(ros::ok())
//     {
//         cv::Mat image = camera.camera_read();
//         std::vector<Ten::yolo::Detection> results = yobb.worker(image);
//         cv::Mat result = drawDetections(image, results);
        
//     }
// }

void test4()
{
    // 1. 初始化检测模型与相机
    Ten::yolo::yolo_26obb yobb("/home/maple/study3/li/best_openvino_model_op13/model", "cpu");
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();

    // ===================== 新增：ROS图像发布初始化 =====================
    ros::NodeHandle nh;
    // 图像传输对象（比直接发布Image消息更高效）
    image_transport::ImageTransport it(nh);
    // 创建发布者：话题名 /detection/result_image，队列长度1
    image_transport::Publisher image_pub = it.advertise("/detection/result_image", 1);
    // ==================================================================

    ros::Rate sl(30);
    while(ros::ok())
    {
        // 2. 读取图像 + 目标检测 + 绘制结果
        cv::Mat image = camera.camera_read();
        std::vector<Ten::yolo::Detection> results = yobb.worker(image);
        cv::Mat result = drawDetections(image, results);

        // ===================== 新增：发布检测后的图片 =====================
        if (!result.empty()) // 安全判断：图像非空再发布
        {
            // 封装OpenCV图像为ROS消息
            cv_bridge::CvImage cv_img;
            cv_img.header.stamp = ros::Time::now();  // 时间戳（同步必备）
            cv_img.header.frame_id = "camera";       // 坐标系ID（可自定义）
            cv_img.encoding = "bgr8";                // OpenCV默认彩色图像编码
            cv_img.image = result;                   // 绑定绘制后的图像

            // 转换为ROS标准消息并发布
            sensor_msgs::ImagePtr ros_img_msg = cv_img.toImageMsg();
            image_pub.publish(ros_img_msg);
        }
        // ==================================================================

        // 必须添加：处理ROS回调函数（不写则话题无法正常发布）
        ros::spinOnce();
        // 必须添加：控制循环频率为30Hz（不写会占满CPU）
        sl.sleep();
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    test4();
    return 0;
}

