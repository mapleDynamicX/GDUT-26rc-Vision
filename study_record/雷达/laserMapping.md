# laserMapping

```cpp
int main(int argc, char** argv)
{
    // 初始化ROS节点，设置节点名称为laserMapping（全局唯一）
    ros::init(argc, argv, "laserMapping");
    // 创建私有命名空间节点句柄，用于参数读取/话题创建（避免命名冲突）
    ros::NodeHandle nh("~");
    // 创建异步自旋器，0表示使用CPU核心数相同的线程处理回调队列
    ros::AsyncSpinner spinner(0);
    // 启动异步自旋器，后台处理ROS消息回调（非阻塞主线程）
    spinner.start();
    // 调用自定义函数，通过节点句柄读取ROS参数服务器中的配置参数
    readParameters(nh);
    // 控制台打印激光雷达类型，用于调试确认参数读取正确性
    cout<<"lidar_type: "<<lidar_type<<endl;
    // 创建体素地图智能指针，传入初始化参数，自动管理内存避免泄漏
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // 设置路径消息的时间戳：将激光帧结束时间（秒）转为ROS标准时间戳
    path.header.stamp    = ros::Time().fromSec(lidar_end_time);
    // 设置路径消息的参考坐标系为camera_init（SLAM世界坐标系）
    path.header.frame_id ="camera_init";

    /*** 计数/计时变量定义 ***/
    // 激光帧计数器，统计已处理的激光帧数
    int frame_num = 0;
    // 各环节平均耗时统计变量：总耗时、ICP匹配、特征匹配、增量估计、优化求解、IMU状态传播
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;

    // 将曲面点选中标记数组全部初始化为true（所有曲面点默认参与特征提取）
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    // 设置曲面特征点体素下采样滤波器的体素尺寸（三维等尺寸），减少点云数量提升效率
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // 设置地图点云体素下采样滤波器的体素尺寸，减少地图内存占用和渲染耗时
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // 将激光-IMU平移外参数组转为Eigen向量（VEC_FROM_ARRAY为自定义宏）
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    // 将激光-IMU旋转外参数组转为Eigen矩阵（MAT_FROM_ARRAY为自定义宏）
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);

    // 如果启用激光-IMU外参在线估计
    if (extrinsic_est_en)
    {
        // 若不将IMU作为输入，将外参初始化到输出端KF的状态量中
        if (!use_imu_as_input)
        {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
        // 若将IMU作为输入，将外参初始化到输入端KF的状态量中
        else
        {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    // 统一设置IMU/前一帧激光数据结构体的雷达类型，保证逻辑适配当前雷达
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    // 设置IMU使能标志（是否启用IMU数据参与状态估计）
    p_imu->imu_en = imu_en;

    // 初始化输入端KF的动态模型：传入状态转移函数、雅可比、观测模型
    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    // 初始化输出端KF的动态模型：额外传入IMU专属观测模型
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    // 定义输入端KF初始协方差矩阵（24维度对应输入端状态量：位姿/速度/外参等）
    Eigen::Matrix<double, 24, 24> P_init;
    // 自定义函数初始化协方差（设置各状态量初始不确定性）
    reset_cov(P_init);
    // 将初始化后的协方差赋值给输入端KF
    kf_input.change_P(P_init);
    // 定义输出端KF初始协方差矩阵（30维度对应输出端状态量）
    Eigen::Matrix<double, 30, 30> P_init_output;
    // 自定义函数初始化输出端协方差
    reset_cov_output(P_init_output);
    // 将初始化后的协方差赋值给输出端KF
    kf_output.change_P(P_init_output);
    // 初始化输入端KF过程噪声协方差（描述状态转移的噪声）
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    // 初始化输出端KF过程噪声协方差
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    /*** 调试日志文件初始化 ***/
    // 文件指针，用于写入位姿调试日志
    FILE *fp;
    // 拼接位姿日志文件路径（root_dir为从参数读取的根目录）
    string pos_log_dir = root_dir + "Log/pos_log.txt";
    // 以写入模式打开日志文件（不存在则创建，存在则清空）
    fp = fopen(pos_log_dir.c_str(),"w");
    // 自定义函数：打开其他调试日志文件（如IMU/激光帧日志）
    open_file();

    /*** ROS订阅器初始化 ***/
    // 根据雷达类型选择回调函数：AVIA雷达用livox_pcl_cbk，其他用standard_pcl_cbk
    // 订阅激光点云话题，队列大小200000（避免点云消息积压）
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    // 订阅IMU数据话题，队列大小200000（处理高频IMU数据）
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    /*** ROS发布器初始化 ***/
    // 发布配准后（世界坐标系）的原始分辨率激光点云，队列大小1000
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 1000);
    // 发布配准后（机器人本体坐标系）的原始分辨率激光点云，队列大小1000
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 1000);
    // 注释：发布参与匹配的有效特征点云（暂未启用）
    // ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/cloud_effected", 1000);
    // 发布全局激光地图点云（RViz可视化建图结果），队列大小1000
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 1000);
    // 发布建图后里程计（世界坐标系下的位姿/速度），队列大小1000
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", 1000);
    // 发布机器人运动轨迹，队列大小1000
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 1000);
    // 注释：发布平面法向量可视化标记（暂未启用）
    // ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
    //         ("/planner_normal", 1000);
    
    //......
}
```

### 指令整体作用

`add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")` 是 **CMake 构建系统** 中的核心指令，作用是**向 C/C++ 编译器注入预处理器宏定义**，将`ROOT_DIR`宏的值设置为当前 CMakeLists.txt 所在目录的路径（带末尾目录分隔符），且宏值被双引号包裹，可直接在 C/C++ 代码中作为字符串使用。

### 逐部分拆解说明

#### 1. `add_definitions(...)`

- CMake 内置命令，核心功能是**向编译器添加预处理器定义**（等价于在编译命令行中给 gcc/clang/MSVC 等编译器传递`-D`参数）。
- 编译器层面的效果：所有通过该 CMakeLists.txt 编译的源码文件，都会在预处理阶段识别这些宏定义，无需在代码中手动`#define`。

#### 2. `-DROOT_DIR`

- `-D`是编译器的预处理器宏定义标志（如 gcc 的`-D`选项），表示要定义一个名为`ROOT_DIR`的宏。
- 这一步的核心是**创建一个全局可见的预处理器宏**，C/C++ 代码中可直接使用`ROOT_DIR`来引用该宏的值。

#### 3. `\"${CMAKE_CURRENT_SOURCE_DIR}/\"`

- 这是`ROOT_DIR`宏的**值**，拆解如下：
  - `${CMAKE_CURRENT_SOURCE_DIR}`：CMake 内置变量，代表**当前 CMakeLists.txt 文件所在的目录路径**（绝对路径 / 相对路径，取决于编译环境）；
  - `/`：手动添加目录分隔符，保证路径以`/`结尾（如`/home/user/lasermapping/`），后续拼接子目录（如`Log/`）时无需额外补分隔符；
  - `\"`：CMake 中的转义双引号（CMakeLists.txt 中直接写`"`会被解析为字符串结束符，需用`\`转义），最终传递给编译器的宏值会被双引号包裹（如`"/home/user/lasermapping/"`）。

```cpp
// Livox激光雷达点云数据的ROS回调函数，处理接收到的自定义雷达消息
// 参数msg: 指向Livox自定义消息的常量智能指针，包含雷达点云数据和时间戳等信息
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    // 注释：原代码中的缓冲区互斥锁加锁操作，暂时注释
    // mtx_buffer.lock();

    // 记录点云预处理的开始时间，使用OpenMP的高精度计时函数
    double preprocess_start_time = omp_get_wtime();

    // 全局扫描计数变量自增，统计接收到的雷达扫描次数
    scan_count ++;

    // 检查当前雷达消息的时间戳是否小于上一次记录的雷达时间戳
    // 若成立说明出现时间回环（lidar loop back），数据异常
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        // 打印ROS错误日志，提示雷达时间回环，需要清空缓冲区
        ROS_ERROR("lidar loop back, clear buffer");

        // 注释：原代码中的互斥锁解锁和条件变量通知，暂时注释
        // mtx_buffer.unlock();
        // sig_buffer.notify_all();

        // 直接返回，不处理本次异常的雷达数据
        return;

        // 注释：原代码中的缓冲区收缩操作，暂时注释
        // lidar_buffer.shrink_to_fit();
    }

    // 更新上一次雷达时间戳为当前消息的时间戳（转换为秒级）
    last_timestamp_lidar = msg->header.stamp.toSec();    

    // 注释：IMU与雷达时间同步的逻辑（暂时注释）
    // 功能：若IMU和雷达时间差超过1秒，且未设置时间偏移、IMU队列非空时，
    // 计算并记录IMU相对雷达的时间偏移量，实现自同步
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    // 判断是否启用"帧切割"模式（cut_frame_init为全局标志位）
    if (cut_frame_init) {
        // 定义点云指针队列，用于存储切割后的点云片段
        deque<PointCloudXYZI::Ptr> ptr;
        // 定义时间戳队列，存储对应点云片段的时间戳
        deque<double> timestamp_lidar;

        // 调用预处理模块的process_cut_frame_livox方法，处理当前雷达消息
        // 将切割后的点云片段和时间戳分别填充到ptr和timestamp_lidar队列中
        // 参数说明：msg-当前雷达消息，ptr-输出的点云队列，timestamp_lidar-输出的时间戳队列
        // cut_frame_num-帧切割数量，scan_count-当前扫描计数
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

        // 循环处理切割后的点云队列和时间戳队列（确保两个队列非空）
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            // 将当前队列头部的点云片段压入全局雷达缓冲区
            lidar_buffer.push_back(ptr.front());
            // 弹出队列头部的点云指针（已存入缓冲区，释放队列空间）
            ptr.pop_front();
            // 将时间戳转换为秒级（原时间戳单位为毫秒），压入全局时间戳缓冲区
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            // 弹出队列头部的时间戳（已存入缓冲区，释放队列空间）
            timestamp_lidar.pop_front();
        }
    }
    // 未启用帧切割模式时的处理逻辑
    else
    {
        // 创建新的点云智能指针，初始化容量为10000个点，预留1个额外空间
        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
        
        // 调用预处理模块的process方法，处理当前雷达消息，填充点云数据到ptr中
        p_pre->process(msg, ptr); 

        // 判断是否启用"帧拼接"模式（con_frame为全局标志位）
        if (con_frame)
        {
            // 若当前拼接帧计数为0，说明是新的拼接周期，记录拼接起始时间
            if (frame_ct == 0)
            {
                time_con = last_timestamp_lidar; // 也可使用msg->header.stamp.toSec()，效果一致
            }

            // 若当前拼接帧计数小于10，继续拼接点云（累计10帧为一个完整拼接帧）
            if (frame_ct < 10)
            {
                // 遍历当前点云的所有点，调整曲率值（融合时间差信息）
                for (int i = 0; i < ptr->size(); i++)
                {
                    // 曲率值加上（当前雷达时间戳 - 拼接起始时间）*1000（转换为毫秒级时间差）
                    // 目的：将时间信息融入曲率特征，便于后续帧匹配
                    ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                    // 将当前点添加到拼接点云容器中
                    ptr_con->push_back(ptr->points[i]);
                }
                // 拼接帧计数自增，记录已拼接的帧数
                frame_ct ++;
            }
            // 拼接帧计数达到10，完成一帧拼接，存入缓冲区
            else
            {
                // 创建新的点云智能指针，存储拼接完成的点云
                PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
                // 注释：调试用打印，输出分割后点云的大小（暂时注释）
                // cout << "ptr div num:" << ptr_div->size() << endl;
                
                // 将拼接完成的点云数据复制到新创建的ptr_con_i中
                *ptr_con_i = *ptr_con;
                // 记录当前拼接帧的起始时间
                double time_con_i = time_con;

                // 将拼接完成的点云压入全局雷达缓冲区
                lidar_buffer.push_back(ptr_con_i);
                // 将拼接帧的起始时间压入全局时间戳缓冲区
                time_buffer.push_back(time_con_i);

                // 清空拼接点云容器，为下一轮拼接做准备
                ptr_con->clear();
                // 重置拼接帧计数，开始新的拼接周期
                frame_ct = 0;
            }
        }
        // 未启用帧拼接模式时的处理逻辑
        else
        {
            // 检查当前点云是否有有效数据（点数大于0）
            if (ptr->points.size() > 0)
            {
                // 将当前点云压入全局雷达缓冲区
                lidar_buffer.emplace_back(ptr);
                // 将当前雷达消息的时间戳（秒级）压入全局时间戳缓冲区
                time_buffer.emplace_back(msg->header.stamp.toSec());
            }
        }
    }

    // 记录本次扫描的预处理耗时（结束时间-开始时间），存入全局数组s_plot11
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;

    // 注释：原代码中的缓冲区互斥锁解锁和条件变量通知，暂时注释
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

```
