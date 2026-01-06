#ifndef __LOGGER_H_
#define __LOGGER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <mutex>
#include <unistd.h>
#include <sys/stat.h> // Linux mkdir 函数头文件
#include <sys/types.h>
#include <cerrno>     // errno 错误码（Ubuntu 下必备）
#include "./../method_math.h"
#include "./../recognition/zbuffer_simplify.h"
#include "./../lidar.h"

namespace Ten
{

    class Ten_logger
    {
    public:
        //禁用拷贝构造
        Ten_logger(const Ten_logger& logger) = delete;
        //禁用赋值
        Ten_logger& operator=(const Ten_logger& logger) = delete;

        /**
         * @brief 获得日志实例
         * @param 写入路径
         */
        static Ten_logger& GetInstance(const std::string& path = "./src/merge/log")
        {
            //static Ten_serial* ten_serial = nullptr;
            static std::unique_ptr<Ten_logger> ten_logger = nullptr;
            std::call_once(logger_flag_, [path]() 
            {
                ten_logger = create(path);
            });
            if(!ten_logger)
            {
                std::cerr << "ten_logger == nullptr " <<std::endl;
                exit(1); // 异常退出
            }
            return *ten_logger;
        }

        bool record_odometry(const nav_msgs::Odometry& odo)
        {
            std::lock_guard<std::mutex> lock(tf_mtx_);
            std::string filename = directory_ + std::string("/") + std::string("odometry.txt");
            std::ofstream txtFile(filename, std::ios::out | std::ios::app);
            if (!txtFile.is_open()) {
                std::cerr << "错误：无法打开 txt 文件 " << filename << std::endl;
                return false;
            }
            Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
            std::string time = rosTimeToReadable(odo.header.stamp);
            txtFile << std::endl;
            txtFile << time << std::endl;
            txtFile << "Pose:" << std::endl;
            txtFile << "x: " << std::to_string(pose._xyz._x) << std::endl;
            txtFile << "y: " << std::to_string(pose._xyz._y) << std::endl;
            txtFile << "z: " << std::to_string(pose._xyz._z) << std::endl;
            txtFile << "roll: " << std::to_string(pose._rpy._roll) << std::endl;
            txtFile << "pitch: " << std::to_string(pose._rpy._pitch) << std::endl;
            txtFile << "yaw: " << std::to_string(pose._rpy._yaw) << std::endl;
            txtFile << "Speed: " << std::endl;
            txtFile << "x: " << std::to_string(odo.twist.twist.linear.x) << std::endl;
            txtFile << "y: " << std::to_string(odo.twist.twist.linear.y) << std::endl;
            txtFile << "z: " << std::to_string(odo.twist.twist.linear.z) << std::endl;
            txtFile << "roll: " << std::to_string(odo.twist.twist.angular.x) << std::endl;
            txtFile << "pitch: " << std::to_string(odo.twist.twist.angular.y) << std::endl;
            txtFile << "yaw: " << std::to_string(odo.twist.twist.angular.z)  << std::endl;
            txtFile << std::endl;
            return true;
        }

        bool record_image(std::vector<box>& box_list)
        {
            std::lock_guard<std::mutex> lock(image_mtx_);
            static size_t where = 1;
            bool result = 0;
            for(size_t i = 0; i < box_list.size(); i++)
            {
                std::string savePath = directory_ + std::string("/") + std::to_string(where) + std::string("idx") + 
                    std::to_string(box_list[i].idx) + std::string("cls") + std::to_string(box_list[i].cls) + std::string("conf") + std::to_string(box_list[i].confidence);
                    result = cv::imwrite(savePath, box_list[i].roi_image);
                if (result) {
                    std::cout << "✅ 基础保存图片成功：" << savePath << std::endl;
                } else {
                    std::cerr << "❌ 基础保存图片失败！请检查路径/权限/格式是否合法" << std::endl;
                }
            }
            where++;
            return result;
        }

        bool record_map()
        {
            std::lock_guard<std::mutex> lock(map_mtx_);
            static size_t where = 1;
            std::string savePath = directory_  + std::string("/") + std::string("map") + std::to_string(where); 
            sensor_msgs::PointCloud2 map = Ten::_Map_GET_.read_data();
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_cloud = Ten::sensor_msgs_PointCloud2topcltype<pcl::PointCloud<pcl::PointXYZINormal>>(map);
            int result = pcl::io::savePCDFile(savePath, *local_cloud);
            where++;
            if (result == 0) 
            {
                std::cout << "✅ 成功保存 PCD 点云：" << savePath << std::endl;
                return true;
            } 
            else 
            {
                std::cerr << "❌ 保存 PCD 点云失败！请检查路径/权限" << std::endl;
                return false;
            }
        }



    private:

        static std::string directory_;
        static int flag_;
        std::mutex tf_mtx_;
        std::mutex map_mtx_;
        std::mutex image_mtx_;
        static std::once_flag logger_flag_;

        Ten_logger(){}//禁止外部初始化实例

        static std::unique_ptr<Ten_logger> create(const std::string& path = "./src/merge/log") {
            // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
            create_file_head(path);
            create_directorys(path);
            create_file(directory_);
            return std::unique_ptr<Ten_logger>(new Ten_logger());
        }

        static bool create_file_head(const std::string& path)
        {
            std::string filename = path + "/head.txt";
            std::fstream file(filename, std::ios::in | std::ios::out);
            // 处理文件不存在的情况：如果打开失败，先创建文件再重新打开
            if (!file.is_open()) {
                std::cout << "文件不存在，自动创建：" << filename << std::endl;
                // 用ofstream创建文件（ios::trunc=清空创建，不存在则新建）
                std::ofstream create_file(filename, std::ios::trunc);
                create_file.close(); // 关闭创建的文件
                // 重新以读写模式打开
                file.open(filename, std::ios::in | std::ios::out);
            }

            // 检查文件是否成功打开（兜底判断）
            if (!file.is_open()) {
                std::cerr << "错误：无法打开/创建文件 " << filename << std::endl;
                exit(1); // 异常退出
                return false;
            }
            
            if(file >> flag_)
            {
                std::cout<< "head.txt 打开成功 flag: " << flag_ <<std::endl;
                // 读取操作后，获取状态掩码
                std::ios_base::iostate state = file.rdstate();

                // 按位与判断具体标志位
                std::cout << "=== 精确状态标志位 ===" << std::endl;
                if (state & std::ios_base::goodbit) {
                    std::cout << "流状态正常(goodbit)" << std::endl;
                }
                if (state & std::ios_base::failbit) {
                    std::cout << "操作失败(failbit)已置位" << std::endl;
                }
                if (state & std::ios_base::eofbit) {
                    std::cout << "到达文件末尾(eofbit)已置位" << std::endl;
                }
                if (state & std::ios_base::badbit) {
                    std::cout << "严重错误(badbit)已置位" << std::endl;
                }
            }
            else
            {
                flag_ = 1;
                //file.clear();
            }
            file.clear();
            file.seekp(0);
            file << std::to_string(flag_+1);
            file.close();
            return true;
        }

        static bool create_directorys(const std::string& path)
        {
            if(!(create_directory(path + std::string("/") + getCurrentDateAsString()) 
            && create_directory(path + std::string("/") + getCurrentDateAsString() + std::string("/") + std::to_string(flag_))))
            {
                exit(1);
                return false;
            }
            directory_ = path + std::string("/") + getCurrentDateAsString() + std::string("/") + std::to_string(flag_);
            std::cout << "directory_: " << directory_ << std::endl;
            return true;
        }

        static bool create_directory(const std::string& dirpath)
        {
            // 2. Ubuntu 下调用 mkdir 创建文件夹
            // 第二个参数 0755：Ubuntu 下的权限（所有者 rwx，其他用户 rx）
            // 注：权限值前的 0 表示八进制，不可省略
            int result = mkdir(dirpath.c_str(), 0755);
            if (result == 0) 
            {
                std::cout << "✅ 成功创建文件夹：" << dirpath << std::endl;
                return true;
            } 
            else 
            {
                // 根据 errno 区分错误类型（Ubuntu 下关键）
                switch (errno) {
                    case EEXIST:
                        std::cout << "ℹ️  提示：文件夹已存在，无需重复创建：" << dirpath << std::endl;
                        return true; // 已存在视为“成功”（按需调整）
                    case EACCES:
                        std::cerr << "❌ 错误：权限不足，无法创建文件夹：" << dirpath << std::endl;
                        break;
                    case ENOENT:
                        std::cerr << "❌ 错误：路径中的上级目录不存在（如创建 a/b/c 但 a/b 不存在）：" << dirpath << std::endl;
                        break;
                    default:
                        std::cerr << "❌ 错误: 创建失败, Ubuntu 错误码：" << errno << "（路径：" << dirpath << "）" << std::endl;
                        break;
                }
                return false;
            }
        }

        static bool create_file(const std::string& dirpath)
        {
            std::string filename = dirpath + std::string("/") +std::string("odometry.txt");
            std::ofstream txtFile(filename, std::ios::out | std::ios::trunc);
            // 检查文件是否成功打开/创建
            if (!txtFile.is_open()) {
                std::cerr << "错误：无法创建/覆盖txt文件 " << filename << std::endl;
                exit(1);
                return false;
            }
            return true;
        }

        // 函数：格式化当前日期为 "年_月_日" 格式（如 2025_1_10）
        static std::string getCurrentDateAsString() {
            // 1. 获取当前系统时间戳
            std::time_t now = std::time(nullptr);
            if (now == -1) {
                std::cerr << "错误：无法获取系统当前时间！" << std::endl;
                return "";
            }

            // 2. 转换为本地时间（Ubuntu 下时区适配）
            std::tm* localTime = std::localtime(&now);
            if (localTime == nullptr) {
                std::cerr << "错误：无法转换为本地时间！" << std::endl;
                return "";
            }

            // 3. 格式化日期为 "年_月_日"（tm_year=年份-1900，tm_mon=月份0-11，tm_mday=日期1-31）
            std::ostringstream oss;
            oss << (localTime->tm_year + 1900)    // 年份（如 2025）
                << "_" << (localTime->tm_mon + 1) // 月份（+1 因为tm_mon从0开始）
                << "_" << localTime->tm_mday;     // 日期（1-31）
            std::cout<< "oss.str(): " << oss.str() << std::endl;
            return oss.str();
        }

        // 函数：将 ROS 时间戳转换为可读字符串（格式：2025-01-10 15:30:20.123456789）
        static std::string rosTimeToReadable(const ros::Time& rosTime) {
            // 1. 提取 ROS 时间戳的秒和纳秒
            time_t sec = rosTime.sec;          // 秒级时间戳
            uint32_t nsec = rosTime.nsec;      // 纳秒（0-999999999）

            // 2. 将秒级时间戳转换为本地时间（tm 结构体）
            struct tm* localTime = std::localtime(&sec);
            if (localTime == nullptr) {
                std::cerr << "错误: ROS 时间戳转换为本地时间失败！" << std::endl;
                return "";
            }

            // 3. 格式化时间字符串（年-月-日 时:分:秒.纳秒）
            std::ostringstream oss;
            oss << (localTime->tm_year + 1900) << "-"                  // 年
                << std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "-"  // 月（补零）
                << std::setw(2) << std::setfill('0') << localTime->tm_mday << " "       // 日（补零）
                << std::setw(2) << std::setfill('0') << localTime->tm_hour << ":"       // 时（补零）
                << std::setw(2) << std::setfill('0') << localTime->tm_min << ":"        // 分（补零）
                << std::setw(2) << std::setfill('0') << localTime->tm_sec << "."        // 秒（补零）
                << std::setw(9) << std::setfill('0') << nsec;          // 纳秒（补零到9位）
            
            return oss.str();
        }

    };




}








#endif
