# GTSAM

```bash
#!bash
mkdir build
cd build
cmake ..
make check  # optional, runs all unit tests
make install
```

`nano_gicp` 这个高性能点云配准库安装

```bash
sudo apt update

# 1. ROS1 Noetic（Ubuntu 20.04）
sudo apt install ros-noetic-pcl-ros ros-noetic-libg2o libomp-dev libeigen3-dev cmake gcc g++

# 2. ROS1 Melodic（Ubuntu 18.04）
# sudo apt install ros-melodic-pcl-ros ros-melodic-libg2o libomp-dev libeigen3-dev cmake gcc g++

# 3. ROS2 Humble/Iron（Ubuntu 22.04）
# sudo apt install ros-humble-pcl-ros libg2o-dev libomp-dev libeigen3-dev cmake gcc g++
```

## 编译

```bash
cd ~/your_workspace/src
# 若你不是通过 --recursive 获取本仓库，请先初始化子模块
# git submodule update --init --recursive

# 将本仓库放入 src 后回到工作空间根目录
cd ~/your_workspace

# 先编译依赖模块（建议 Release）
catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON

# 再编译整个工作空间
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
