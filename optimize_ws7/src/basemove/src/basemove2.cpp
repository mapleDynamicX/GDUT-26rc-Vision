//44 28
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include "PID.cpp"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovariance.h> 
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <cstring>  // 或者 #include <string.h>
#include <iostream>
#include "rc_msgs/yolo_detector.h"
#include "rc_msgs/yolo_detectorArray.h"
#include "rc_msgs/serial.h"
#include <chrono> 

// 包含文件流库，用于文件操作
#include <fstream>
// 包含字符串流库，用于字符串与其他类型的转换
#include <sstream>
// 包含map容器库，用于存储键值对数据
#include <map>
// 包含string库，用于字符串处理
#include <string>

struct
{
    float _lidar_x;
    float _lidar_y;
    float _lidar_yaw;
    float _target_x = 0;
    float _target_y = 0;
    float _target_yaw = 0;
    int _flag = 0;
    int _put = 0.05;
    std::mutex mtx_lidar;
    std::mutex mtx_target;
    std::mutex mtx_flag;
}global;

struct Point
{
    float x = 0;
    float y = 0;
};



class basemove
{
public:
    basemove(): _nh(), _it(_nh)
    {
        Point point;
        point.x = 3.5;
        point.y = 2.7;
        _points.push_back(point);//1
        point.x = 3.5;
        point.y = 1.5;
        _points.push_back(point);//2
        point.x = 3.5;
        point.y = 0.3;
        _points.push_back(point);//3
        point.x = 4.7;
        point.y = 2.7;
        _points.push_back(point);//4
        point.x = 4.7;
        point.y = 1.5;
        _points.push_back(point);//5
        point.x = 4.7;
        point.y = 0.3;
        _points.push_back(point);//6
        point.x = 5.9;
        point.y = 2.7;
        _points.push_back(point);//7
        point.x = 5.9;
        point.y = 1.5;
        _points.push_back(point);//8
        point.x = 5.9;
        point.y = 0.3;
        _points.push_back(point);//9
        point.x = 7.1;
        point.y = 2.7;
        _points.push_back(point);//10
        point.x = 7.1;
        point.y = 1.5;
        _points.push_back(point);//11
        point.x = 7.0;
        point.y = 0.3;
        _points.push_back(point);//12
        _image_sub = _it.subscribe("/camera/color/image_raw", 2, &basemove::imageCallback, this);
        _pub_image = _it.advertise("/camera/map_server", 2);
    }
    void mapCallback(const rc_msgs::serialConstPtr& msg)
    {
        ROS_INFO("Data array size: %zu", msg->map.size());
        std::cout<<"-------------------------------"<<std::endl;
        std::cout<<"ok"<<std::endl;
        for(int i = 0; i < msg->map.size(); i++)
        {
            if((int)msg->map[i] == 0)
            {
                continue;
            }
            _map[i / 3][i % 3] = (int)msg->map[i];
            std::cout<<"位置 : " << i+1 << "  类别 : " << (int)msg->map[i] <<std::endl;
            std::cout<< _map[i / 3][i % 3] << std::endl;
        }
        std::cout<< _map << std::endl;
        std::cout<<"-------------------------------"<<std::endl;

        _receive_flag = 1;
    }

    void getpath()
    {
        // 创建一个map容器，键为string类型(编号)，值为vector<int>(数字集合)
        // 用于存储文件中读取的编号及其对应的数字
        std::map<std::string, std::vector<int>> data;

        // 创建输入文件流对象，并尝试打开名为"data.txt"的文件
        std::ifstream file("/home/rc/RC_2026/optimize_ws7/src/maple/placements_and_paths_blue_sorted.txt");

        // 检查文件是否成功打开
        if (!file.is_open()) {
            // 如果文件无法打开，输出错误信息到标准错误流
            std::cerr << "无法打开文件" << std::endl;
            // 返回1表示程序异常退出
            return;
        }

        // 定义一个字符串变量line，用于存储从文件中读取的每一行
        std::string line;

        // 循环读取文件中的每一行，直到文件末尾
        while (std::getline(file, line)) {
            // 创建字符串流对象iss，并将当前行内容传入
            std::istringstream iss(line);
            // 定义字符串变量id，用于存储编号
            std::string id;
            // 从字符串流中读取第一个字符串作为编号
            iss >> id;

            // 创建vector<int>容器nums，用于存储当前编号对应的数字
            std::vector<int> nums;
            // 定义整数变量num，用于临时存储读取的数字
            int num;

            // 从字符串流中继续读取整数，直到行尾
            while (iss >> num) {
                // 将读取到的数字添加到vector容器中
                nums.push_back(num);
            }

            // 将编号及其对应的数字集合存入map容器
            data[id] = nums;
        }

        // 关闭文件流
        file.close();
        // 定义字符串变量inputId，用于存储用户输入的编号
        //std::string inputId("242144342121");
        std::string inputId;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                inputId += _map[i][j] + '0';
            }
        }
        // 创建一个包含12个整数的数组arr，并初始化为0
        //int arr[12] = { 0 };

        // 在map中查找用户输入的编号
        auto it = data.find(inputId);

        // 如果找到了对应的编号
        if (it != data.end()) {
            // 将找到的数字填充到数组arr中，最多12个
            for (size_t i = 0; i < it->second.size() && i < 12; ++i) {
                _path[i] = it->second[i];
            }

            // 输出找到的编号及其对应的数组
            std::cout << "找到编号 " << inputId << "，对应的数组为: ";
            for (int i = 0; i < 12; ++i) {
                std::cout << _path[i] << " ";
            }
            std::cout << std::endl;
        }
        else {
            // 如果未找到编号，输出提示信息
            std::cout << "未找到编号 " << inputId << std::endl;
            //std::exit(EXIT_SUCCESS);
            _map_flag = 0;
        }

    }

    void finish()
    {
        int total_r1 = 0;
        int total_r2 = 0;
        int total_f = 0;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                if(_map[i][j] == 1)
                {
                    total_r1++;
                }
                if(_map[i][j] == 2)
                {
                    total_r2++;
                }
                if(_map[i][j] == 3)
                {
                    total_f++;
                }
            }
        }
        if(total_r1 != 3)
        {
            _map[0][0] = 1;
        }
        else if(total_r2 != 4)
        {
            _map[0][0] = 2;
        }
        else if(total_f != 1)
        {
            _map[0][0] = 3;
        }
        else
        {
            _map[0][0] = 4;
        }
    }

    void calibration()
    {
        if(_map_flag == 0)
        {
            {
                std::lock_guard<std::mutex> lock(global.mtx_target);
                global._target_x = 0;
                global._target_y = 0;
                global._target_yaw = 0;
            }
            ros::Rate sl(10);
            while(!gettagetxy(0, 0)&&ros::ok())
            {
                sl.sleep();
            }
            while(!gettagetyaw(0)&&ros::ok())
            {
                sl.sleep();
            }
            ros::Rate s(0.3);
            {
                std::lock_guard<std::mutex> lock(global.mtx_flag);
                global._flag = 1;
            }
            s.sleep();
            ros::Publisher status_pub = _nh.advertise<rc_msgs::serial>("/serial/status", 10);
            ros::Subscriber map_sub = _nh.subscribe("/serial/map", 10, &basemove::mapCallback, this);
            sl.sleep();
            rc_msgs::serial serial_pub;
            serial_pub.stamp = ros::Time::now();
            serial_pub.map.push_back(1);
            ros::Rate sleep(300);
            for(int i = 0; i < 50; i++)
            {
                status_pub.publish(serial_pub);
                sleep.sleep();
            }
            auto start = std::chrono::steady_clock::now();
            
            _receive_flag = 0;
            while(_receive_flag == 0 && ros::ok())
            {
                sleep.sleep();
                auto end = std::chrono::steady_clock::now();
                auto duration = end - start;
                double seconds_double = static_cast<double>(duration.count()) / 1e9;
                if(seconds_double > 0.5)
                {
                    for(int i = 0; i < 50; i++)
                    {
                        status_pub.publish(serial_pub);
                    }
                    start = std::chrono::steady_clock::now();
                }
                ros::spinOnce();
            }
            _receive_flag = 0;
            _map_flag++;
            {
                std::lock_guard<std::mutex> lock(global.mtx_flag);
                global._flag = 0;
            }
        }
        else if(_map_flag == 1)
        {
            Point point = _points[2-1];
            {
                
                std::lock_guard<std::mutex> lock(global.mtx_target);
                global._target_x = point.x;
                global._target_y = point.y;
                global._target_yaw = 0;
            }
            ros::Rate sl(10);
            while(!gettagetxy(point.x, point.y)&&ros::ok())
            {
                sl.sleep();
            }
            while(!gettagetyaw(0)&&ros::ok())
            {
                sl.sleep();
            }
            ros::Rate s(0.3);
            {
                std::lock_guard<std::mutex> lock(global.mtx_flag);
                global._flag = 0;
            }
            s.sleep();
            ros::Publisher status_pub = _nh.advertise<rc_msgs::serial>("/serial/status", 10);
            ros::Subscriber map_sub = _nh.subscribe("/serial/map", 3, &basemove::mapCallback, this);
            sl.sleep();
            rc_msgs::serial serial_pub;
            serial_pub.stamp = ros::Time::now();
            serial_pub.map.push_back(2);
            ros::Rate sleep(300);
            for(int i = 0; i < 50; i++)
            {
                status_pub.publish(serial_pub);
                sleep.sleep();
            }
            auto start = std::chrono::steady_clock::now();
            
            _receive_flag = 0;
            while(_receive_flag == 0 && ros::ok())
            {
                auto end = std::chrono::steady_clock::now();
                auto duration = end - start;
                double seconds_double = static_cast<double>(duration.count()) / 1e9;
                if(seconds_double > 0.5)
                {
                    for(int i = 0; i < 50; i++)
                    {
                        status_pub.publish(serial_pub);
                    }
                    start = std::chrono::steady_clock::now();
                }
                sleep.sleep();
                ros::spinOnce();
                ros::spinOnce();
            }
            _receive_flag = 0;
            _map_flag++;
            {
                std::lock_guard<std::mutex> lock(global.mtx_flag);
                global._flag = 0;
            }
            finish();
            getpath();
        }
    }

    int gettagetyaw(float yaw)
    {
        std::lock_guard<std::mutex> lock(global.mtx_lidar);
        float error = abs(global._lidar_yaw - yaw);
        if(error < 0.02)
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }
    
    int gettagetxy(float x, float y)
    {
        std::lock_guard<std::mutex> lock(global.mtx_lidar);
        float errorx = abs(global._lidar_x - x);
        float errory = abs(global._lidar_y - y);
        if(errorx < 0.02 && errory < 0.02)
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }

    // int getboxok()
    // {

    // }

    void yolodetector(const rc_msgs::yolo_detectorArray::ConstPtr msg)
    {

        for (size_t i = 0; i < msg->boxes.size(); ++i) 
        {
            const auto& bbox = msg->boxes[i];
            _enable_yolo = bbox.cls;
            return;
        }

    } 


    void processimg()
    {
        ros::Rate sl(10);
        ros::Subscriber bbox_sub = _nh.subscribe("/yolo/detections", 2, &basemove::yolodetector, this);
        sl.sleep();
        cv::Rect roi(760, 680, 380, 340);
        cv::Mat image = _image(roi);
        cv::resize(image, image, cv::Size(640,640));
        // 创建ROS图像消息
        sensor_msgs::ImagePtr msg;
        // 将OpenCV图像转换为ROS消息
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        _pub_image.publish(msg);
        auto start = std::chrono::steady_clock::now();
        ros::Rate sleep(300);
        _enable_yolo = 0;
        while(ros::ok())
        {
            auto end = std::chrono::steady_clock::now();
            auto duration = end - start;
            double seconds_double = static_cast<double>(duration.count()) / 1e9;
            std::cout<< "seconds" << seconds_double << std::endl;
            if(seconds_double > 0.3)
            {
                start = std::chrono::steady_clock::now();
                if(_enable_yolo == 0)
                {
                    _get_box_ok = 1;
                }
                break;
            }
            if(_enable_yolo == 4)
            {
                std::cout<<"out"<<std::endl;
                _get_box_ok = 1;
                break;
            }
            sleep.sleep();
            ros::spinOnce();
            std::cout<<"not_outxxx"<<std::endl;
        }
        _enable_yolo = 0;  
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            if(_enable_image)
            {
                // 将ROS图像消息转换为OpenCV格式（BGR8编码）
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                _image =  cv_ptr->image;
                _enable_image = 0;
                processimg();
                if(_get_box_ok == 0)
                {
                    _enable_image = 1;
                }
            }

        }
        catch (cv_bridge::Exception& e)
        {
            // 处理转换异常 
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void getbox()
    {

        int where = _path[_path_flag];
        std::cout<<"wwwwwwwwwww   "<<where<<std::endl;
        if( _path_flag <= 11)
        {
            _path_flag++;
        }
        if(_get_box_num >= 3)
        {
            return;
        }
        if(where + 3 < 13)
        {
            if(_map[(where+3-1) / 3][(where+3-1) % 3] == 2)
            {
                {
                    std::lock_guard<std::mutex> lock(global.mtx_target);
                    global._target_yaw = 0;
                }
                ros::Rate sl(3);
                while(!gettagetyaw(0)&&ros::ok())
                {
                    sl.sleep();
                }

                _get_box_ok = 0;
                ros::Rate sll(300);
                _enable_image = 1;
                while(ros::ok() && !_get_box_ok)
                {
                    ros::spinOnce();
                    sll.sleep();
                }
                _get_box_ok = 0;
                _map[(where+3-1) / 3][(where+3-1) % 3] = 4;      
                _get_box_num++;          
            }
        }
        if(where % 3 != 1)
        {
            if(_map[(where-1-1) / 3][(where-1-1) % 3] == 2)
            {
                {
                    std::lock_guard<std::mutex> lock(global.mtx_target);
                    global._target_yaw = 90;
                }
                ros::Rate sl(3);
                while(!gettagetyaw(90)&&ros::ok())
                {
                    sl.sleep();
                }
                _get_box_ok = 0;
                ros::Rate sll(300);
                _enable_image = 1;
                while(ros::ok() && !_get_box_ok)
                {
                    ros::spinOnce();
                    sll.sleep();
                    std::cout<<"slslslslsl"<<std::endl;
                }
                _get_box_ok = 0;
                _map[(where-1-1) / 3][(where-1-1) % 3] = 4;
                _get_box_num++;
            }
        }
        if(where % 3 != 0)
        {
            if(_map[(where+1-1) / 3][(where+1-1) % 3] == 2)
            {
                {
                    std::lock_guard<std::mutex> lock(global.mtx_target);
                    global._target_yaw = -90;
                }
                ros::Rate sl(3);
                while(!gettagetyaw(-90)&&ros::ok())
                {
                    sl.sleep();
                }
                _get_box_ok = 0;
                ros::Rate sll(300);
                _enable_image = 1;
                while(ros::ok() && !_get_box_ok)
                {
                    ros::spinOnce();
                    sll.sleep();
                }
                _get_box_ok = 0;
                _map[(where+1-1) / 3][(where+1-1) % 3] = 4;
                _get_box_num++;
            }
        }
        if(where - 3 > 0)
        {
            if(_map[(where-3-1) / 3][(where-3-1) % 3] == 2)
            {
                {
                    std::lock_guard<std::mutex> lock(global.mtx_target);
                    global._target_yaw = 180;
                }
                ros::Rate sl(3);
                while(!gettagetyaw(180)&&ros::ok())
                {
                    sl.sleep();
                }
                _get_box_ok = 0;
                ros::Rate sll(300);
                _enable_image = 1;
                while(ros::ok() && !_get_box_ok)
                {
                    ros::spinOnce();
                    sll.sleep();
                }
                _get_box_ok = 0;
                _map[(where-3-1) / 3][(where-3-1) % 3] = 4;
                _get_box_num++;
            }
        }
        _enable_image = 0;
    }

    void tonextpoint()
    {
        int last_where = _path[_path_flag - 1];
        int where = _path[_path_flag];
        std::cout<<"last_where: "<< last_where <<std::endl;
        std::cout<<"whererererere: "<< where <<std::endl;

        if(where == 0)
        {
            return;
        }
        float yaw;

        if(where + 1 == last_where)
        {
            yaw = 90;
        }
        else if(where - 1 == last_where)
        {
            yaw = -90;
        }
        else if(where - 3 == last_where)
        {
            yaw = 0;
        }
        else
        {
            ROS_ERROR("_________________________-----danger----------");
            return;
        }
        Point point = _points[last_where - 1];
        {
            std::lock_guard<std::mutex> lock(global.mtx_target);
            global._target_x = point.x;
            global._target_y = point.y;
            global._target_yaw = yaw;
            std::cout<< "global._target_yaw = yaw;" << global._target_yaw << std::endl;
        }
        ros::Rate sl(3);
        while(!gettagetyaw(yaw)&&ros::ok())
        {
            sl.sleep();
        }

        _get_box_ok = 0;
        ros::Rate sll(300);
        _enable_image = 1;
        while(ros::ok() && !_get_box_ok)
        {
            ros::spinOnce();
            sll.sleep();
        }
        _enable_image = 0;
        _get_box_ok = 0;

        // {
        //     std::lock_guard<std::mutex> lock(global.mtx_target);
        //     global._target_yaw = 0;
        // }
        // while(!gettagetyaw(0)&&ros::ok())
        // {
        //     sl.sleep();
        // }

        Point point2 = _points[where - 1];
        {
            std::lock_guard<std::mutex> lock(global.mtx_target);
            global._target_x = point2.x;
            global._target_y = point2.y;
            global._target_yaw = yaw;
            std::cout<< "point.x" << point.x<< std::endl;
        }
        while(!gettagetxy(point.x, point.y)&&ros::ok())
        {
            sl.sleep();
        }
    }

    void movecontrol()
    {
        if(_map_flag >= 2)
        {
            getbox();
            tonextpoint();
        }
    }

private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;// 创建图像传输对象
    image_transport::Subscriber _image_sub;
    image_transport::Publisher _pub_image;
    int _map[4][3] = {0};
    int _path[12] = {0};
    int _path_flag = 0;
    int _map_flag = 0;
    int _receive_flag = 0;
    int _get_box_num = 0;
    int _enable_image = 0;
    int _get_box_ok = 0;
    int _enable_yolo = 0;
    cv::Mat _image;
    std::vector<Point> _points;
};




// 回调函数：处理接收到的 TF 消息
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    for (const auto& transform : msg->transforms)
    {
        if(transform.child_frame_id != "body")
        {
            continue;
        }
        // 获取四元数
        geometry_msgs::Quaternion quat = transform.transform.rotation;

        // 使用 tf2 库将四元数转换为欧拉角
        tf2::Quaternion tf_quat;
        tf_quat.setX(quat.x);
        tf_quat.setY(quat.y);
        tf_quat.setZ(quat.z);
        tf_quat.setW(quat.w);

        // 转换为旋转矩阵，然后提取欧拉角
        tf2::Matrix3x3 mat(tf_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);  // 得到的是弧度值

        // ROS_INFO_STREAM("Transform from " << transform.header.frame_id 
        //               << " to " << transform.child_frame_id 
        //               << "\nRoll: " << roll 
        //               << " [rad], Pitch: " << pitch 
        //               << " [rad], Yaw: " << yaw 
        //               << " [rad]");
        
        double yaw_deg = yaw * 180.0 / M_PI;
        float _x  = transform.transform.translation.x - 0.28*cos(yaw) + 0.28;
        float _y = transform.transform.translation.y - 0.28*sin(yaw);

        // std::cout << "base x = "<< _x << std::endl;
        // std::cout << "base y = "<< _y << std::endl;
        // std::cout << "base yaw = "<< yaw << std::endl;

        //mtx.lock();
        std::lock_guard<std::mutex> lock(global.mtx_lidar);
        global._lidar_x = _x;
        global._lidar_y = _y;
        global._lidar_yaw = yaw_deg;
        //mtx.unlock();
    }
}




void worker_task1(ros::NodeHandle nh)
{
    ros::Rate sl(1000);
    ros::Subscriber tf_sub = nh.subscribe("/tf", 10, tfCallback);
    while (ros::ok())
    {
        /* code */
        
        ros::spinOnce();
        sl.sleep();
    }
    
}


void worker_task2(ros::NodeHandle nh)
{
    ros::Rate sl(100);
    basemove _basem;
    while (ros::ok())
    {
        _basem.calibration();
        _basem.movecontrol();
        sl.sleep();
    }
    
}




int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "basemove2_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    std::vector<std::thread> workers;
    workers.emplace_back(worker_task1, nh);
    workers.emplace_back(worker_task2, nh);
    // 创建 Publisher，发布 /cmd_vel 话题，消息类型 geometry_msgs::Twist，队列长度 10
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置循环频率
    ros::Rate loop_rate(11); // 10Hz

    PIDcontroler pid_x;
    PIDcontroler pid_y;
    PIDcontroler pid_yaw(0.31, 0.3, 0.02, 0.08, 0.0, 0.0);
    int flag = 0;
    auto start = std::chrono::steady_clock::now();
    while (ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        {
            std::lock_guard<std::mutex> lock(global.mtx_lidar);
            flag = global._flag;

        }
        if(flag == 0)
        {
            auto end = std::chrono::steady_clock::now();
            auto duration = end - start;
            double seconds_double = static_cast<double>(duration.count()) / 1e9;
            // std::cout<< "seconds_double" << seconds_double << std::endl;
            {
                std::lock_guard<std::mutex> lock(global.mtx_lidar);
                std::lock_guard<std::mutex> lock2(global.mtx_target);
                vel_msg.linear.x = pid_x.PIDcalculate(global._target_x, global._lidar_x, seconds_double)*std::cos(global._lidar_yaw * M_PI / 180.0) + pid_y.PIDcalculate(global._target_y, global._lidar_y, seconds_double)*std::sin(global._lidar_yaw* M_PI / 180.0);
                vel_msg.linear.y = pid_x.PIDcalculate(global._target_x, global._lidar_x, seconds_double)*std::sin(global._lidar_yaw * M_PI / 180.0)*-1 + pid_y.PIDcalculate(global._target_y, global._lidar_y, seconds_double)*std::cos(global._lidar_yaw* M_PI / 180.0);
                vel_msg.angular.z = pid_yaw.PIDcalculate(global._target_yaw, global._lidar_yaw, seconds_double);
                // std::cout<<"global._target_x" <<global._target_x<<std::endl;
                // std::cout<<"global._target_y" <<global._target_y<<std::endl;
                // std::cout<<"global._target_yaw" <<global._target_yaw<<std::endl;
            }
        }
        else
        {
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.angular.z = 0;
            std::cout<< "0000000" << std::endl;
        }
        start = std::chrono::steady_clock::now();
        // 发布消息
        cmd_vel_pub.publish(vel_msg);
        // ROS_INFO("Publishing /cmd_vel: linear.x=%f, linear.y=%f, angular.z=%f",
        //          vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);
        ros::spinOnce();
        loop_rate.sleep();
    }




    return 0;
}






















