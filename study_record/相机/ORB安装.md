# ORB安装

## 安装Pangolin 0.6 版本

## ⭐⭐⭐ 唯一解决方案：源码编译安装 Pangolin 0.6 版本（⭐强烈推荐，100% 兼容 ORB-SLAM2，一步到位）

### ✅ 版本选择关键说明（重中之重，避坑！）

1. **绝对不要用 `sudo apt install libpangolin-dev` 安装**：Ubuntu 官方源里的 Pangolin 是 **0.5 老版本**，和 ORB-SLAM2 的代码**不兼容**，安装后会出现 `undefined reference` 链接错误，编译必失败；
2. **必须源码编译安装 0.6 版本**：这是**唯一能和 ORB-SLAM2 完美兼容**的版本，无任何编译 / 运行 bug，是所有 ORB-SLAM2 使用者的标配版本，也是本次报错的**唯一正确解法**。

### ✅ 完整安装命令（复制粘贴即可，Ubuntu18.04/20.04 通用，无需手动下载，一步到底）

你的终端当前路径不影响，**直接复制所有命令按顺序执行**，全程自动化，耗时约 3-5 分钟，虚拟机也能顺利完成：

```bash
# ============= 第一步：安装Pangolin编译所需的所有依赖库 =============
sudo apt-get update && sudo apt-get install -y build-essential libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev libpython3-dev python3-dev python3-numpy

# ============= 第二步：下载Pangolin 0.6 源码（固定版本，兼容无bug） =============
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6  # 切换到0.6稳定版，核心！不要用最新版

# ============= 第三步：编译+安装Pangolin（源码编译标准流程） =============
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4  # 内存小/虚拟机 直接用 make 单核编译，避免卡死，稳！
sudo make install  # 安装到系统全局目录 /usr/local，cmake可自动识别

# ============= 第四步：关键！更新系统库缓存，让系统立即识别新装的Pangolin =============
sudo ldconfig
```

## 编译ORB_slam3

```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

### 要修改CMakeLists.txt文件

```yaml
#find_package(OpenCV 4.4)
find_package(OpenCV REQUIRED)
```

# ROS Examples

[](https://github.com/UZ-SLAMLab/ORB_SLAM3#7-ros-examples)

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D

[](https://github.com/UZ-SLAMLab/ORB_SLAM3#building-the-nodes-for-mono-mono-inertial-stereo-stereo-inertial-and-rgb-d)

Tested with ROS Melodic and ubuntu 18.04.

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:

```bash
gedit ~/.bashrc
```

and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
```

2. Execute `build_ros.sh` script:

```bash
chmod +x build_ros.sh
./build_ros.sh
```
