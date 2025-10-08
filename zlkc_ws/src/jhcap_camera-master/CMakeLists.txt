cmake_minimum_required(VERSION 3.0.2)
project(jhcap_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------
# 寻找依赖包
# ------------------------------
# 寻找ROS核心包和必要组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)

# 寻找OpenCV（需要core、imgproc、highgui、imgcodecs组件）
find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  highgui
  imgcodecs
)

# 寻找JHCap SDK的库文件（假设库名为libJHCap.so）
# find_library(JHCap_LIBRARY
#   NAMES JHCap  # 库文件名（不含前缀lib和后缀.so）
#   PATHS /usr/lib  # 库文件可能的安装路径
#   NO_DEFAULT_PATH  # 不搜索默认路径（可选，根据实际情况调整）
# )

# 验证JHCap库是否找到
# if(NOT JHCap_LIBRARY)
#   message(FATAL_ERROR "JHCap library not found. Please install the JHCap SDK and check the library path.")
# endif()

# # 寻找JHCap SDK的头文件（假设头文件JHCap.h在/usr/include或/usr/local/include）
# find_path(JHCap_INCLUDE_DIR
#   NAMES JHCap.h  # 头文件名
#   PATHS /usr/lib # 头文件可能的安装路径
#   NO_DEFAULT_PATH  # 不搜索默认路径（可选，根据实际情况调整）
# )

# 验证JHCap头文件是否找到
# if(NOT JHCap_INCLUDE_DIR)
#   message(FATAL_ERROR "JHCap header files not found. Please install the JHCap SDK and check the include path.")
# endif()

# ------------------------------
# 配置catkin包信息
# ------------------------------
catkin_package(
  # 依赖的ROS包（其他包依赖本包时需要的信息）
  CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge camera_info_manager
  # 依赖的非ROS包（如OpenCV、JHCap）
  DEPENDS OpenCV 
  # 本包的头文件目录（如果有自定义头文件需要导出，可添加此处）
  # INCLUDE_DIRS include
  # 本包的库文件（如果有自定义库需要导出，可添加此处）
  # LIBRARIES jhcap_camera
)

# ------------------------------
# 包含头文件目录
# ------------------------------
include_directories(
  ${catkin_INCLUDE_DIRS}  # ROS包的头文件目录
  ${OpenCV_INCLUDE_DIRS}  # OpenCV的头文件目录
  #${JHCap_INCLUDE_DIR}    # JHCap SDK的头文件目录
)

# ------------------------------
# 生成可执行文件
# ------------------------------
add_executable(jhcap_camera  # 可执行文件名（对应ROS节点名）
  src/jhcap_camera.cpp       # 源文件路径（假设源文件在src目录下）
)

# ------------------------------
# 链接依赖库
# ------------------------------
target_link_libraries(jhcap_camera
  ${catkin_LIBRARIES}    # ROS包的库文件（如roscpp、sensor_msgs等）
  ${OpenCV_LIBRARIES}    # OpenCV的库文件（如opencv_core、opencv_imgproc等）
  #${JHCap_LIBRARY}       # JHCap SDK的库文件（如libJHCap.so）
  JHCap
  ${CMAKE_THREAD_LIBS_INIT}  # 线程库（支持std::thread）
)

# ------------------------------
# 安装可执行文件
# ------------------------------
install(TARGETS jhcap_camera
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # 安装到ROS的bin目录（如devel/lib/jhcap_camera/）
)