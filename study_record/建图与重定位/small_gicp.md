# small_gicp

[GitHub - koide3/small_gicp: Efficient and parallel algorithms for point cloud registration [C++, Python]](https://github.com/koide3/small_gicp)

```bash
sudo apt-get install libeigen3-dev libomp-dev

cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### 一、核心定位与背景

`small_gicp` 是一个**仅头文件（header-only）** 的轻量级 C++ 库，核心目标是提供**高效、并行化**的点云精细配准算法。它并非全新发明，而是对经典的 `fast_gicp` 库进行了彻底重构和深度优化，解决了原库在性能、并行性和依赖管理上的痛点，专注于 CPU 端的点云配准（明确不支持 GPU 实现）。

### 二、核心特性深度解析

#### 1. 极致的性能优化

- 核心算法（如 GICP/VGICP）在 `fast_gicp` 基础上做了底层优化（比如内存布局、计算流程、数据结构），实测性能提升最高可达 **2 倍**；
- 优化聚焦于 “精细配准” 阶段（粗配准后的精准对齐），在保证配准精度的前提下大幅降低计算耗时。

#### 2. 全流程并行化

`small_gicp` 不仅优化了配准核心算法，还将**预处理阶段也做了并行化**，实现 “端到端” 并行，支持两种主流并行后端：

- **OpenMP**：轻量级、易集成，适合大多数通用场景；
- **Intel TBB**：更灵活的线程池管理，适合复杂的多线程场景。

并行化覆盖的关键步骤包括：

- 点云下采样（Downsampling）；
- KdTree 构建（近邻搜索的核心）；
- 法向量 / 协方差估计（配准算法的关键预处理）；
- 配准核心迭代计算

#### 3. 最小化依赖

`small_gicp` 追求 “轻量”，核心仅依赖 3 个库（无其他强依赖）：

- **Eigen**：线性代数计算（矩阵、向量运算）；
- **nanoflann**（内置）：轻量级近邻搜索库，无需单独安装；
- **Sophus**：李群 / 李代数计算（三维位姿表示与优化）。


