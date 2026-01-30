#ifndef __ZBUFFER_H_
#define __ZBUFFER_H_

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace Ten
{

#define L_ 1.2f         // 台阶长度
#define H_ 0.2f         // 台阶高度
#define ly1_ 0.425f     // 台阶到方块的间距
#define lx1_ 0.425f     // 台阶到方块的间距
#define lh_ 0.35f       // 方块的长度
#define X_ 2.58f        // 初始位置到梅花林1号位置边角的x轴距离
#define Y_ 3.395f       // 初始位置到梅花林1号位置边角的y轴距离
#define LIDAR_HEIGHT_ 0.717     // 雷达的高度 

struct surface_2d_point {        
    int idx;                       // 对应方块索引
    cv::Point2f left_up;           // 左上2D点
    cv::Point2f right_up;          // 右上2D点
    cv::Point2f right_down;        // 右下2D点
    cv::Point2f left_down;         // 左下2D点
    float surface_depth;           // 该表面的深度值
};

struct box{
    int idx;                             // 表示位置的下标索引
    cv::Mat roi_image;                   // 裁剪出来的roi图片
    int cls = 0;                             // 识别类别
    float confidence = 0.0f;                // 自信度
    int zbuffer_flag = 0;                    // zbuffer是否处理的标志位， 0 表示未处理， 1 表示已处理， -1 表示异常
    int exist_flag = -1;                      // 是否筛空的标志位， 0 表示空， 1 表示有方块， -1 表示未处理
    int roi_valid_flag = 0;                   // 用于表示当前的roi 图像是否有效
};


}


#endif

