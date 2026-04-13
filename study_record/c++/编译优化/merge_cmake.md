# merge cmake

```cmake
# merge 功能包 CMakeLists.txt | 逐行注释版 | 激光雷达+视觉融合ROS项目
# 重点标注：性能优化编译指令（-O3/OpenMP/C++17/多线程调度）

# 注释：备用高版本CMake兼容配置
#cmake_minimum_required(VERSION 3.0.2)
# 指令：指定当前工程支持的最低CMake版本为2.8.3（适配旧版ROS）
cmake_minimum_required(VERSION 2.8.3)
# 指令：定义工程名称为merge，后续可通过${PROJECT_NAME}引用
project(merge)

# ====================== 编译模式与核心性能优化配置 ======================
# 【性能优化】指令：设置编译模式为Debug（调试模式，无代码优化，便于断点调试）
SET(CMAKE_BUILD_TYPE "Debug")
# 【性能优化关键】指令：Release发布模式（编译器自动开启优化，生产环境必用）
# set(CMAKE_BUILD_TYPE "Release")
# 【性能优化关键】指令：Release模式专属参数（最高级优化O3+编译警告+C++17）
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")

# 【性能优化】指令：全局添加编译选项，指定C++17标准（新特性提升执行效率）
ADD_COMPILE_OPTIONS(-std=c++17)
# 指令：重复添加C++17选项，确保所有编译单元生效
ADD_COMPILE_OPTIONS(-std=c++17)
# 【性能优化核心】指令：设置C++基础编译标志（C++17 + O3最高级编译器优化）
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) 

# 指令：定义宏ROOT_DIR，值为当前工程源码目录，用于代码内路径读取
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# 指令：为C语言编译器添加异常支持，保证C/C++异常处理统一
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
# 【性能优化】指令：显式指定C++标准版本为17
set(CMAKE_CXX_STANDARD 17)
# 指令：强制要求C++17标准，不满足则编译失败
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 指令：关闭编译器非标准扩展，保证代码可移植性
set(CMAKE_CXX_EXTENSIONS OFF)
# 【性能优化关键】指令：追加编译参数（C++17+多线程pthread+异常支持）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

# ====================== CPU架构检测 + 多线程性能调度 ======================
# 指令：打印当前CPU架构信息
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
# 【性能优化】指令：判断CPU为x86/amd64架构（桌面平台，支持多线程优化）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  # 指令：引入CPU核心数统计模块
  include(ProcessorCount)
  # 指令：获取CPU核心数并赋值给变量N
  ProcessorCount(N)
  # 指令：打印CPU总核心数
  message("Processer number:  ${N}")
  # 【性能优化】指令：核心数>5，开启多线程并行计算，分配4个核心
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  # 【性能优化】指令：核心数>3，自动分配N-2个核心，避免占用全部CPU
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  # 指令：核心数≤3，单线程运行
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
# 指令：ARM等嵌入式平台，强制单线程运行
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# ====================== 第三方依赖库查找 ======================
# 指令：查找Ceres求解库（SLAM后端优化核心）
find_package(Ceres REQUIRED)
# 指令：查找teaserpp点云配准库（快速全局配准）
find_package(teaserpp REQUIRED)
# 【性能优化关键】指令：查找OpenMP并行计算库（静默模式，未找到不报错）
find_package(OpenMP QUIET)
# 【性能优化关键】指令：将OpenMP编译参数加入C++编译器
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# 【性能优化关键】指令：将OpenMP编译参数加入C编译器
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
# 指令：强制标记realsense2库已找到，跳过依赖检查
set(realsense2_FOUND TRUE)

# 指令：查找Python库（可视化绘图依赖）
find_package(PythonLibs REQUIRED)
# 指令：查找matplotlibcpp绘图头文件路径
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
# 指令：添加libusb-1.0头文件路径
include_directories(/usr/include/libusb-1.0)
# 指令：添加系统库链接目录
link_directories(/usr/lib/x86_64-linux-gnu)
# 指令：强制启用C++17标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指令：查找ROS核心依赖包
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  tf
  tf2
  tf2_msgs
  tf2_ros
  vision_msgs
  rosbag
  livox_ros_driver
  livox_ros_driver2
)

# 指令：查找OpenCV视觉库
find_package(OpenCV REQUIRED)
# 指令：查找OpenVINO AI推理引擎
find_package(OpenVINO REQUIRED)
# 指令：启用pkg-config工具查找依赖
find_package(PkgConfig REQUIRED)
# 指令：查找realsense2深度相机SDK
find_package(realsense2)
# 指令：通过pkg-config查找libserial串口库
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 指令：查找Livox雷达静态库
find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)

# ====================== catkin工程配置 ======================
# 指令：打印OpenVINO版本信息
message(STATUS "Found OpenVINO v${OpenVINO_VERSION}")
message(STATUS "OpenVINO include dir: ${OpenVINO_INCLUDE_DIRS}")
message(STATUS "OpenVINO libraries: ${OpenVINO_LIBRARIES}")
# 指令：catkin工程导出配置（依赖声明）
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge vision_msgs
  DEPENDS OpenVINO
)

# 指令：查找Eigen3矩阵运算库
find_package(Eigen3 REQUIRED)
# 指令：查找PCL点云库（版本≥1.8）
find_package(PCL 1.8 REQUIRED)
# 指令：打印Eigen3头文件路径
message(Eigen: ${EIGEN3_INCLUDE_DIR})

# 指令：配置头文件包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_INCLUDE_DIRS}
  ${LIBSERIAL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TEASERPP_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

# 指令：配置库文件链接目录
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${TEASERPP_LIBRARY_DIRS}
)

# ====================== 编译库文件 ======================
# 指令：编译nanoflann近邻搜索库
add_library(nanoflann src/nano_gicp/src/nanoflann.cc)
# 指令：链接nanoflann依赖库
target_link_libraries(nanoflann ${PCL_LIBRARIES})

# 指令：编译nano_gicp点云配准库
add_library(nano_gicp src/nano_gicp/src/nano_gicp.cc)
# 【性能优化】指令：链接nano_gicp依赖（PCL+Eigen+OpenMP）
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${EIGEN3_LIBS} ${OpenMP_LIBS} nanoflann)

# ====================== 编译可执行文件 ======================
# 指令：生成主节点merge_node
add_executable(merge_node src/merge.cpp)
# 指令：为节点添加预编译宏定义
target_compile_definitions(merge_node PRIVATE
)

# 指令：查找liburcu并发库，未找到则编译失败
find_library(URCU_LIB NAMES urcu urcu-memb)
if(NOT URCU_LIB)
  message(FATAL_ERROR "liburcu not found! Please install liburcu-dev.")
endif()

# 指令：为主节点添加所有源码文件
target_sources(merge_node
  PRIVATE
  src/lidar.cpp
  src/method_math.cpp
  src/openvino.cpp
  src/serial.cpp
  src/camera.cpp
  src/coordinate.cpp
  src/velocity.cpp
  src/calibration.cpp
  src/threadpool.cpp
  src/log/logger.cpp
  src/yolo/yolo_base.cpp
  src/orb/orb_debug.cpp
  src/recognition/world_to_camera.cpp
  src/recognition/camera_calibration.cpp
  src/recognition/occlusion_handing.cpp
  src/livox_ros_driver2/src/driver_node.cpp
  src/livox_ros_driver2/src/lds.cpp
  src/livox_ros_driver2/src/lds_lidar.cpp
  src/livox_ros_driver2/src/lddc.cpp
  src/livox_ros_driver2/src/livox_ros_driver.cpp
  src/livox_ros_driver2/src/comm/comm.cpp
  src/livox_ros_driver2/src/comm/ldq.cpp
  src/livox_ros_driver2/src/comm/semaphore.cpp
  src/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp
  src/livox_ros_driver2/src/comm/cache_index.cpp
  src/livox_ros_driver2/src/comm/pub_handler.cpp
  src/livox_ros_driver2/src/parse_cfg_file/parse_cfg_file.cpp
  src/livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/livox_ros_driver2/src/call_back/lidar_common_callback.cpp
  src/livox_ros_driver2/src/call_back/livox_lidar_callback.cpp
  src/point_lio/src/laserMapping2.cpp
  src/point_lio/src/li_initialization.cpp 
  src/point_lio/src/parameters.cpp 
  src/point_lio/src/preprocess.cpp 
  src/point_lio/src/Estimator.cpp 
  src/point_lio/src/IMU_Processing.cpp
)

# 【性能优化】指令：链接主节点所有依赖库（含加速库/AI/点云配准）
target_link_libraries(merge_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${URCU_LIB}
  ${CERES_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/librealsense2.so
  openvino::runtime
  teaser_registration
  teaser_io
  nano_gicp
  nanoflann
)

# 指令：为节点添加Python头文件路径
target_include_directories(merge_node PRIVATE ${PYTHON_INCLUDE_DIRS})

```


