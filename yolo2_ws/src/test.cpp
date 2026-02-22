#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <algorithm>
#include "./yolo/yolo_v11_cls.h"
#include "./yolo/yolo_v5.h"


struct TargetArea {
    int id;         //区域编号
    float x1;       //左上x坐标
    float y1;       //左上y坐标
    float x2;       //右下x坐标
    float y2;       //右下y坐标
};   
int global_area[12] = {0};

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


float calculate_IOU(
    float detect_x1, float detect_y1, float detect_x2, float detect_y2,
    float target_x1, float target_y1, float target_x2, float target_y2) {
    
    //计算交集区域的边界
    float inter_x1 = std::max(detect_x1, target_x1);
    float inter_y1 = std::max(detect_y1, target_y1);
    float inter_x2 = std::min(detect_x2, target_x2);
    float inter_y2 = std::min(detect_y2, target_y2);

    //计算交集面积（无交集则为0）
    float inter_width = std::max(0.0f, inter_x2 - inter_x1);
    float inter_height = std::max(0.0f, inter_y2 - inter_y1);
    float inter_area = inter_width * inter_height;

    //计算YOLO检测框的面积
    float detect_area = (detect_x2 - detect_x1) * (detect_y2 - detect_y1);

    //避免除0错误（检测框面积为0时返回0）
    if (detect_area <= 0.0001f) {
        return 0.0f;
    }

    return inter_area / detect_area;
}



std::vector<TargetArea> TargetAreas1To6() {
    std::vector<TargetArea> areas;
    areas.push_back({1, 11.0f, 597.0f, 68.0f, 761.0f});   
    areas.push_back({2, 265.0f, 764.0f, 523.0f, 867.0f});  
    areas.push_back({3, 728.0f, 744.0f, 944.0f, 792.0f});  
    areas.push_back({4, 85.0f, 621.0f, 244.0f, 695.0f}); 
    areas.push_back({5, 530.0f, 651.0f, 622.0f, 714.0f}); 
    areas.push_back({6, 806.0f, 516.0f, 938.0f, 563.0f});
    return areas;
}

// 拆分目标区域：7-12号区域
std::vector<TargetArea> TargetAreas7To12() {
    std::vector<TargetArea> areas;
    areas.push_back({7, 200.0f, 756.0f, 271.0f, 1004.0f});  
    areas.push_back({8, 813.0f, 830.0f, 1089.0f, 883.0f});  
    areas.push_back({9, 1560.0f, 729.0f, 1725.0f, 813.0f});  
    areas.push_back({10, 550.0f, 718.0f, 624.0f, 769.0f}); 
    areas.push_back({11, 866.0f, 643.0f, 1041.0f, 740.0f}); 
    areas.push_back({12, 1258.0f, 713.0f, 1348.0f, 783.0f});
    return areas;
}


void Check_Box(const std::vector<Ten::yolo::Detection>& detectBoxes, 
                     const std::vector<TargetArea>& targetAreas,
                     float iouThreshold) {

                        
    // 1. 初始化区域计数（key=区域ID，value=检测框数量）
    std::unordered_map<int, int> areaBoxCount;
    for (const auto& area : targetAreas) {
        areaBoxCount[area.id] = 0;
    }

    for (size_t i = 0; i < detectBoxes.size(); ++i) {
        const Ten::yolo::Detection & box = detectBoxes[i];

        //转换检测框为左上/右下坐标
        float box_x1, box_y1, box_x2, box_y2;
        calculate_square(box, box_x1, box_y1, box_x2, box_y2);

        //遍历每个目标区域，计算IOU并更新计数
        for (const auto& area : targetAreas) {
            float iou = calculate_IOU(box_x1, box_y1, box_x2, box_y2,
                                    area.x1, area.y1, area.x2, area.y2);
            bool isInArea = (iou > iouThreshold);
            
            if (isInArea) {
                areaBoxCount[area.id]++;
            }
        }
    }

    std::cout << "\n=== 各区域内YOLO识别框数量统计 ===" << std::endl;
    for (const auto& pair : areaBoxCount) {
        std::cout << "区域" << pair.first << ": " << pair.second << " 个识别框" << std::endl;
        // 核心新增：将存在性值赋值给全局数组（区域ID-1 = 数组索引）
        global_area[pair.first - 1] = (pair.second > 0) ? 1 : 0;
    }
}


void draw_boxes(cv::Mat& img, const std::vector<Ten::yolo::Detection>& detections, const std::vector<TargetArea>& targetAreas) {

    if (img.empty()) {
        ROS_WARN("输入图像为空，跳过绘制");
        return;
    }
    
    for (const auto& area : targetAreas) {
        cv::Rect area_rect(cv::Point(area.x1, area.y1), cv::Point(area.x2, area.y2));
        cv::rectangle(img, area_rect, cv::Scalar(255, 0, 0), 2);
        // 标注区域ID
        cv::putText(img, "Area " + std::to_string(area.id), 
                    cv::Point(area.x1, area.y1 - 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }

    //2. 绘制YOLO检测框（红色 BGR: 0,0,255，线宽2）
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

    const float IOU_THRESHOLD = 0.9f;

    Ten::yolo::yolo_v5 detector("/home/hou/rong_code/src/yolo/src/corner_v5s_200_260116_openvino_model/corner_v5s_200_260116", "cpu", 0.2, 0.2, 0);
 
    

    cv::Mat img1 = cv::imread("/home/hou/rong_code/src/yolo/src/photo/test1_1.jpg");
    cv::Mat img2 = cv::imread("/home/hou/rong_code/src/yolo/src/photo/test1_2.jpg");
    if (img1.empty() || img2.empty()) {
        ROS_ERROR("图片读取失败，请检查路径！");
        return;
    }

    std::vector<Ten::yolo::Detection> detections1 = detector.worker(img1);
    std::vector<Ten::yolo::Detection> detections2 = detector.worker(img2);

    std::vector<TargetArea> targetAreas1 = TargetAreas1To6();   // 第一张图用1-6
    std::vector<TargetArea> targetAreas2 = TargetAreas7To12();  // 第二张图用7-1

    Check_Box(detections1, targetAreas1, IOU_THRESHOLD);
    draw_boxes(img1, detections1, targetAreas1);
    
    Check_Box(detections2, targetAreas2, IOU_THRESHOLD);
    draw_boxes(img2, detections2, targetAreas2);
    

    int area8_val = global_area[7];
    // 2. 获取11号区域原始值（11号索引=10）
    int area11_original = global_area[10];
    if (area8_val == 0) {
        global_area[10] = area11_original;
    } else {
        int count_ones = 0;
        for (int i = 0; i < 12; ++i) {
            // 跳过11号区域（索引10）
            if (i == 10) continue;
            if (global_area[i] == 1) {
                count_ones++;
            }
        }
        if (count_ones <= 7) {
            global_area[10] = 1;
        } else if (count_ones >= 8) {
            global_area[10] = 0;
        }
    }
    
    const std::vector<std::vector<int>> matrixAreaIds = {
        {10, 11, 12}, 
        {7, 8, 9},     
        {4, 5, 6},    
        {1, 2, 3}     
    };

    // 输出最终合并矩阵：直接从全局数组按「区域ID-1」取索引值
    std::cout << "\n=== 最终合并的区域检测框存在性矩阵（1=有框，0=无框） ===" << std::endl;
    for (const auto& row : matrixAreaIds) {
        for (size_t colIdx = 0; colIdx < row.size(); ++colIdx) {
            int areaId = row[colIdx];
         //draw_boxes(img2, detections2, targetAreas2);       int val = global_area[areaId - 1]; // 区域ID-1 = 数组索引
            std::cout << val;
            if (colIdx < row.size() - 1) {
                std::cout << " ";
            }
        }
        std::cout << std::endl;
    }


    std::cout<< "detections1.size()" << detections1.size() << std::endl;
    cv::imshow("img1", img1);
    cv::imshow("img2", img2);

    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    test2();
    return 0;
}

