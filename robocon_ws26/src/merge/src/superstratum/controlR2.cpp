#include "controlR2.h"
#include <std_msgs/Int32.h>


namespace Ten
{
    /**
     * @brief 读取txt文件，数字直接读数值，符号转ASCII码，返回int向量，并打印原始文件内容
     * @param filePath txt文件的路径字符串
     * @return std::vector<int> 数字存原值，符号存ASCII码（忽略空格/换行/回车）
     */
    std::vector<int> readFileToAsciiVector(const std::string& filePath)
    {
        // 1. 以只读模式打开文件
        std::ifstream fileStream(filePath, std::ios::in);
        std::vector<int> result;

        // 判断文件是否成功打开
        if (!fileStream.is_open())
        {
            std::cerr << "错误：无法打开文件！路径：" << filePath << std::endl;
            return result;
        }

        // 2. 一次性读取文件全部内容到字符串中
        std::string fileContent((std::istreambuf_iterator<char>(fileStream)),
                                std::istreambuf_iterator<char>());
        fileStream.close(); // 读取完成后关闭文件

        // 3. 核心要求：打印【读取到的原始文件字符串】（不变）
        std::cout << "=== 读取到的文件原始内容 ===" << std::endl;
        std::cout << fileContent << std::endl;
        std::cout << "==========================" << std::endl;

        // 4. 遍历处理：严格按分隔符解析数字，符号转ASCII
        std::string numBuffer;
        for (char ch : fileContent)
        {
            // ===================== 核心分隔规则 =====================
            // 1. 空格/换行/回车：仅作为分隔符，跳过，同时终止当前数字拼接
            if (ch == ' ' || ch == '\n' || ch == '\r')
            {
                // 如果有未保存的数字，先存入结果
                if (!numBuffer.empty())
                {
                    result.push_back(std::stoi(numBuffer));
                    numBuffer.clear();
                }
                continue;
            }

            // 2. 数字字符：连续拼接（多位数合并）
            if (std::isdigit(static_cast<unsigned char>(ch)))
            {
                numBuffer += ch;
            }
            // 3. 符号字符（( ) 等）：作为分隔符，先存数字，再存符号ASCII
            else
            {
                if (!numBuffer.empty())
                {
                    result.push_back(std::stoi(numBuffer));
                    numBuffer.clear();
                }
                result.push_back(static_cast<int>(ch));
            }
        }

        // 处理文件末尾最后一个数字
        if (!numBuffer.empty())
        {
            result.push_back(std::stoi(numBuffer));
        }

        int flag  = 1;
        Ten::_global_path_.clear();
        for(size_t i = 0; i < result.size(); i++)
        {
            if(result[i] == '(')
            {
                flag = 0;
                continue;
            }
            else if(result[i] == ')')
            {
                flag = 1;
                continue;
            }
            else if(result[i] == 0)
            {
                flag = 0;
                continue;
            }

            if(flag)
            {
                Ten::_global_path_.push_back(result[i]);
            }
        }
        return result;
    }


    /**
     * @brief 发布 std_msgs::Int32 消息
     * @param data 输入：需要发布的 int 类型数据
     */
    void response(int data)
    {
        static ros::Publisher int_pub;
        if (!int_pub)
        {
            ros::NodeHandle nh;
            int_pub = nh.advertise<std_msgs::Int32>("/scripts/response", 3);
        }
        std_msgs::Int32 int_msg;
        int_msg.data = data;
        int_pub.publish(int_msg);
    }

    void pathCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        static int receive = 0;
        static int last = 0;
        static uint8_t arr[30];

        receive = msg->data;
        if(receive != last)
        {
            last = receive;
            std::vector<int> path = readFileToAsciiVector(std::string(ROOT_DIR) + std::string("path/map.txt"));
            uint8_t length = path.size();
            _global_path_.resize(length);

            if(length)
            {
                for(size_t i = 0; i < path.size() && i < 30; i++)
                {
                    arr[i] = (uint8_t)path[i];

                    
                }
                Ten::_MAP_FLAG_.set_flag(1);
                while(Ten::_MAP_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    std::cout << "serial.serial_send(arr, 7, length);" << std::endl;
                    serial.serial_send(arr, 7, length);
                    usleep(100*1000);
                }
                response(last);
            }
        }
        else
        {
            response(last);
        }
    }

    // void script_control2()
    // {
    //     ros::NodeHandle nh;
    //     // 订阅int话题
    //     ros::Subscriber send_path = nh.subscribe("/scripts/send_path", 3, pathCallback);
    //     ros::Rate loop_rate(100);  
    //     // 你的线程循环条件
    //     while (Ten::_TREADPOOL_FLAG_.read_flag())  
    //     {
    //         ros::spinOnce();  
    //         loop_rate.sleep();  
    //     }
    // }

    void script_control()
    {
        // 1. 创建当前线程私有独立回调队列，和其他线程完全隔离
        ros::CallbackQueue queue_script;
        // 2. 创建句柄并绑定私有队列，该nh下所有订阅消息都进这个私有队列
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_script);

        // 3. 订阅话题，消息只会投递到 queue_script，不走全局默认队列
        ros::Subscriber send_path = nh.subscribe("/scripts/send_path", 3, pathCallback);

        // 循环控频100Hz
        ros::Rate loop_rate(100);
        while (Ten::_TREADPOOL_FLAG_.read_flag())
        {
            // 只处理当前线程私有队列，不处理全局其他话题回调
            // 超时0.01s，无消息最多阻塞10ms就返回，避免卡死
            queue_script.callAvailable(ros::WallDuration(0.01));
            loop_rate.sleep();
        }
    }

    //位置监控
    void tf_monitor(nav_msgs::Odometry msg)
    {
        static float arr[4] = {0};
        size_t size = sizeof(arr);
        Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        Ten::XYZRPY pose_fastlio = Ten::Nav_Odometrytoxyzrpy(msg);
        if(Ten::_POINT_LIO_RUN_FLAG_.read_flag())
        {
            Ten::XYZRPY pose_point_lio;
            double v = 0;
            ros::Rate sl(Ten::_laser_pub_hz_*2);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.get_latest(odo))
                {
                    sl.sleep();
                    continue;
                }
                pose_point_lio = Ten::Nav_Odometrytoxyzrpy(odo);
                v = std::sqrt(odo.twist.twist.linear.x*odo.twist.twist.linear.x + odo.twist.twist.linear.y*odo.twist.twist.linear.y);
            }
            //距离大于某个阈值
            double distance = pose_fastlio.Eclidean_distance(pose_point_lio);
            if(distance > 1.0)
            {            
                Ten::_POINT_LIO_RUN_FLAG_.set_flag(false);
            }

            if(v < 0.3)
            {
                Ten::_POINT_LIO_CHANGE_FLAG_.set_flag(false);
            }
            else
            {
                Ten::_POINT_LIO_CHANGE_FLAG_.set_flag(true);
            }
        }
        else
        {
            //变化
            Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose_fastlio);
            Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
            arr[0] = result._xyz._x;
            arr[1] = result._xyz._y;
            arr[2] = result._xyz._z;
            arr[3] = result._rpy._yaw;
            serial.serial_send(arr, 1, size);
        }

        if(!Ten::_POINT_LIO_CHANGE_FLAG_.read_flag())
        {
            //变化
            Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose_fastlio);
            Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
            arr[0] = result._xyz._x;
            arr[1] = result._xyz._y;
            arr[2] = result._xyz._z;
            arr[3] = result._rpy._yaw;
            serial.serial_send(arr, 1, size);
        }
       

    }

    // 里程计消息回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        //Ten::_TF_GET2_.write_data(*msg);
        //Ten::_TF_GET2_.push(*msg);
        // Ten::PV pose_and_velocity_now;
        tf_monitor(*msg);

    }

    // 线程1：只处理odom话题，用独立私有队列
    void Loopcallback2()
    {
        urcu_memb_register_thread();
        ros::CallbackQueue queue_odom;   // 线程私有，不与其他线程共享
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_odom); // 当前句柄的所有订阅/服务都绑定到该队列
        ros::Subscriber odom_sub = nh.subscribe("/fast_lio2/odom", 10, odomCallback);

        ros::Rate loop_rate(20);
        while (Ten::_TREADPOOL_FLAG_.read_flag())
        {
            queue_odom.callAvailable(ros::WallDuration(0.05)); // 只消费自己的队列
            loop_rate.sleep();
        }
        urcu_memb_unregister_thread();
    }

}

