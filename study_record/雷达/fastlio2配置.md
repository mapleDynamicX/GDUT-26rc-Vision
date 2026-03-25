# fastlio2配置

### 2.3 Sophus

[](https://github.com/hku-mars/FAST-LIVO2?tab=readme-ov-file#23-sophus)

Sophus Installation for the non-templated/double-only version

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.4 Vikit

[](https://github.com/hku-mars/FAST-LIVO2?tab=readme-ov-file#24-vikit)

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git 
```

## 3. Build

[](https://github.com/hku-mars/FAST-LIVO2?tab=readme-ov-file#3-build)

Clone the repository and catkin_make:

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST-LIVO2
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run our examples

[](https://github.com/hku-mars/FAST-LIVO2?tab=readme-ov-file#4-run-our-examples)

Download FAST-LIVO2-Dataset from [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA) Section IV.

```bash
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

# `git checkout a621ff` 命令详解

这是 **Git 版本控制**中最常用的命令之一，核心作用是：

**把你的项目代码，切换到 提交 ID（版本号）为 `a621ff` 的历史版本**

---

## 1. 命令拆解

表格

| 部分             | 含义                                       |
| -------------- | ---------------------------------------- |
| `git checkout` | Git 核心命令：**切换 / 检出**（可以切换分支、切换历史版本、恢复文件） |
| `a621ff`       | **提交哈希值（Commit ID）的缩写**                  |

> Git 每个历史版本都有一个唯一的 40 位哈希码（比如 `a621ffdc23...`），**前 6 位唯一时，直接用缩写即可**。

---

## 2. 执行后会发生什么？

1. **项目文件还原**
   
   你本地文件夹里的所有代码，会瞬间变成 `a621ff` 这个版本当时的样子（新增 / 删除 / 修改的文件都会还原）。

2. **进入「分离头指针」状态 (Detached HEAD)**
   
   ✅ 这是最关键的点：
   
   切换到**具体版本**（不是分支）时，Git 会提示你处于 `detached HEAD` 状态：
   
   - 你可以**查看、调试、运行旧代码**
   - **不要直接在这个状态提交新代码**（否则修改会丢失）

---

## 3. 这个命令的常见用途

1. **查看历史版本**：想看看项目以前的代码长什么样
2. **排查 bug**：新版本出问题，回退到旧版本定位问题
3. **基于旧版本开发**：从这个历史版本新建一个分支（推荐）

这个错误是**旧版 Sophus 库与新版 Eigen 库不兼容**导致的，我来给你讲清楚问题根源和一步到位的解决方法：

---

### 一、错误原因

1. 报错核心：`lvalue required as left operand of assignment`（赋值运算符左侧必须是可修改的左值）
2. 根本原因：
   - 你用的是**新版 Eigen（3.3+）**，其中 `real()`/`imag()` 是**取值函数**（返回右值，不能直接赋值）；
   - 旧版 Sophus 代码里错误地用了 `unit_complex_.real() = 1.` 这种赋值方式，新版 Eigen 不支持。

---

### 二、解决方法（仅需修改 2 行代码）

#### 步骤 1：编辑报错文件

在终端执行命令，打开出错的 `so2.cpp`

#### 步骤 2：修改第 32、33 行代码

找到这**两行错误代码**：

```cpp
unit_complex_.real() = 1.;
unit_complex_.imag() = 0.;
```

替换为**新版 Eigen 支持的赋值写法**：

```cpp
unit_complex_.real(1.);
unit_complex_.imag(0.);

```
