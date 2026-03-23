# OBB原理

### 1.1 Slab 算法核心思想

Slab（平板）算法是检测**射线与轴对齐 / 有向包围盒（OBB）** 相交的经典高效算法，核心逻辑是：

> 将 OBB 看作由**三对相互平行的平面（Slab）** 围成的空间（每对平面对应 OBB 的一个轴向），射线必须**同时穿过所有三对平面**，才会与 OBB 相交。

### 1.2 代码逐行原理拆解

```cpp
bool rayIntersectsCubeOBB(const Ray& ray, const Cube& cube, double& t_out)
{
    // 初始化：t_min是射线进入所有平板的「最晚时刻」（初始为0，射线起点）
    // t_max是射线离开所有平板的「最早时刻」（初始为无穷大）
    double t_min = 0.0;
    double t_max = std::numeric_limits<double>::infinity();

    // 坐标转换：将射线原点从世界坐标系转换到OBB局部坐标系（以OBB中心为原点）
    Eigen::Vector3d origin_local = ray.origin - cube.center;

    // 遍历OBB的3个正交轴（对应三对平行平板）
    for (int i = 0; i < 3; ++i)
    {
        const Eigen::Vector3d& axis = cube.axes[i];  // 当前轴向（单位向量）
        double ext = cube.extents[i];               // OBB在该轴上的半长（平板间距的一半）
        double dir_dot = ray.direction.dot(axis);   // 射线方向在当前轴上的投影
        double orig_dot = origin_local.dot(axis);   // 射线原点在当前轴上的投影

        // 情况1：射线与当前轴垂直（射线方向在该轴上的投影接近0）
        if (std::fabs(dir_dot) < 1e-8)
        {
            // 若射线原点的投影超出平板范围（|orig_dot| > ext），则射线完全在OBB外，无相交
            if (std::fabs(orig_dot) > ext) return false;
            continue;  // 平行但在范围内，不影响t区间，跳过
        }

        // 情况2：射线与当前轴不平行，计算进入/离开当前平板的t值
        // 数学推导：射线方程P = origin + t*direction → 投影到当前轴：P·axis = orig_dot + t*dir_dot
        // 平板的平面方程：P·axis = ±ext → 解t得：t = (±ext - orig_dot)/dir_dot
        double t1 = (ext - orig_dot) / dir_dot;   // 进入平板的t（或离开，取决于dir_dot符号）
        double t2 = (-ext - orig_dot) / dir_dot;  // 离开平板的t（或进入，取决于dir_dot符号）

        // 确保t1是进入时间（较小值），t2是离开时间（较大值）
        if (t1 > t2) std::swap(t1, t2);

        // 更新全局t区间：
        // t_min取「所有t1的最大值」→ 射线必须最晚进入所有平板，才是真正的进入OBB时间
        // t_max取「所有t2的最小值」→ 射线必须最早离开所有平板，才是真正的离开OBB时间
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);

        // 若t_min > t_max → 射线无法同时穿过所有平板，无相交
        if (t_min > t_max) return false;
    }

    // 输出相交点的t参数（取进入时间t_min）
    t_out = t_min;
    // 排除射线起点（相机光心）的误判：t_min > 1e-8 避免射线原点就在OBB表面的情况
    return t_min > 1e-8;
}
```

![](/home/maple/笔记/images/2026-02-10-20-56-42-image.png)

## 二、核心函数 2：isCornerOccluded（检测立方体角点是否被遮挡）

### 2.1 核心思想

判断立方体的某个角点是否能被相机 “看到”，需排除两种遮挡情况：

> - **跨立方体遮挡**：从相机光心到角点的射线，在到达角点前与其他立方体相交；
> - **自身遮挡**：角点位于立方体 “背向相机” 的一侧，被立方体自身遮挡。

### 2.2 代码逐行原理拆解

```cpp
bool isCornerOccluded(const std::vector<Cube>& cube_list, int target_cube_idx, int corner_idx)
{
    // 步骤1：获取目标角点和相机光心（世界坐标系）
    const Cube& target_cube = cube_list[target_cube_idx];
    const Eigen::Vector3d& corner = target_cube.world_corners[corner_idx];

    // 相机光心的世界坐标计算（核心公式）：
    // 相机外参R是「世界→相机」的旋转矩阵，T是「世界→相机」的平移向量；
    // 相机光心在相机坐标系中是(0,0,0)，转换到世界坐标系：0 = R*P_world + T → P_world = -R^T * T
    Eigen::Matrix3d R_cam = camerainfo_.R();
    Eigen::Vector3d T_cam = camerainfo_.T();
    Eigen::Vector3d cam_center = -R_cam.transpose() * T_cam;

    // 步骤2：生成从相机光心到角点的射线（单位化方向）
    Ray ray;
    ray.origin = cam_center;
    Eigen::Vector3d dir = corner - cam_center;
    if (dir.norm() < 1e-8) return true; // 异常：角点与相机光心重合，视为遮挡
    ray.direction = dir.normalized();   // 射线方向单位化（不影响t值的相对大小）
    double corner_dist = dir.norm();    // 相机到角点的真实距离（用于判断遮挡优先级）

    // 步骤3：检测是否被其他立方体遮挡
    for (int i = 0; i < cube_list.size(); ++i)
    {
        if (i == target_cube_idx) continue; // 跳过自身（先检测跨立方体遮挡）
        double t_intersect;
        // 用Slab算法检测射线与当前立方体的相交
        if (rayIntersectsCubeOBB(ray, cube_list[i], t_intersect))
        {
            // 相交点距离（t_intersect） < 角点距离 → 射线先碰到其他立方体，角点被遮挡
            if (t_intersect < corner_dist - 1e-8)
            {
                return true;
            }
        }
    }

    // 步骤4：检测是否被自身遮挡（角点在相机视角的“背面”）
    // corner_rel：角点相对于目标立方体中心的向量
    Eigen::Vector3d corner_rel = corner - target_cube.center;
    // view_dir：相机指向立方体中心的视线方向（单位化）
    Eigen::Vector3d view_dir = (cam_center - target_cube.center).normalized();
    // 点积<0 → 角点在立方体朝向相机的反面（自身遮挡）
    // 原理：点积=|a||b|cosθ，θ>90°时cosθ<0，说明两个向量方向相反
    if (corner_rel.dot(view_dir) < -1e-8)
    {
        return true;
    }

    // 无遮挡
    return false;
}
```

![](/home/maple/笔记/images/2026-02-10-20-58-38-2026-02-10%2020-58-24屏幕截图.png)

# Slab 算法（射线 - 包围盒相交检测）详细解析

Slab 是**射线与 AABB/OBB 相交检测**的工业级标准算法，全程只有**加减乘除**，无开方、无三角函数，极快且鲁棒，是图形学、物理引擎、碰撞检测的基础。

---

## 一、核心思想（一句话吃透）

把**包围盒**看成 **3 组互相垂直的平行平面 “平板（Slab）”** 围成的空间：

- 每一组平行平面对应一个轴向（x/y/z 或 OBB 局部轴）
- 射线必须**同时穿过这 3 个 Slab**，才会和包围盒相交
- 用**射线参数 t** 记录穿过每个 Slab 的进入 / 退出时间，取交集判断是否合法

## 二、前置：射线的参数方程

所有计算都基于这条射线：

![](/home/maple/笔记/images/2026-02-10-22-27-43-image.png)

![](/home/maple/笔记/images/2026-02-10-22-30-39-image.png)

![](/home/maple/笔记/images/2026-02-10-22-31-08-image.png)

![](/home/maple/笔记/images/2026-02-10-22-31-31-image.png)

![](/home/maple/笔记/images/2026-02-10-22-32-15-image.png)

![](/home/maple/笔记/images/2026-02-10-22-32-45-image.png)

相机光心在世界坐标系的坐标公式：Cworld​=−R^T⋅T（R^T 是世界→相机旋转矩阵的转置）；
