# cctagcpp

### 示例代码

```cpp
/**
 * @brief CCTag 3环标记实时检测主函数（相机实时采集+算法检测+结果可视化）
 */
void test1()
{
    // ===================== 1. 初始化相机与ROS控制模块 =====================
    // 获取相机单例实例（单例模式：确保全局只有一个相机对象，避免重复初始化）
    Ten::Ten_camera& camera = Ten::Ten_camera::GetInstance();
    // 设置ROS循环频率为10Hz（控制循环执行速度，防止过快占用资源）
    ros::Rate sl(10);

    // ===================== 2. 初始化CCTag检测核心参数 =====================
    // 创建CCTag算法参数配置对象
    cctag::Parameters params;
    // 设置检测的CCTag为3环类型（与生成的3环标记严格匹配，必须一致）
    params._nCrowns = 3;

    // ===================== 3. 初始化CCTag标记库 =====================
    // 修复：CCTag标记库标准构造方式，传入环数3，加载官方3环标准编码库
    // 作用：存储0~9标准3环ID的编码规则，用于检测时匹配验证
    cctag::CCTagMarkersBank bank(3); 

    // 定义CCTag检测结果容器（存储所有检测到的有效/无效标记对象）
    cctag::CCTag::List markers;

    // ===================== 4. 初始化OpenCV可视化窗口 =====================
    // 创建名为"CCTag Detection"的窗口，支持手动缩放大小
    cv::namedWindow("CCTag Detection", cv::WINDOW_NORMAL);

    // ===================== 5. 主循环：持续检测相机图像 =====================
    // ROS循环条件：节点未关闭时持续执行
    while(ros::ok())
    {
        // 从相机读取一帧原始图像（BGR格式，OpenCV标准格式）
        cv::Mat raw_image = camera.camera_read();

        // 判断：如果读取的图像为空（相机异常/未连接）
        if(raw_image.empty())
        {
            // 打印ROS警告日志：提示无相机图像
            ROS_WARN("无相机图像");
            // 按照设定频率延时，避免空循环占用CPU
            sl.sleep();
            // 跳过本次循环，继续读取下一帧
            continue;
        }

        // ===================== 6. 图像预处理：缩放+转灰度 =====================
        // 定义缩放后的图像变量，存储缩小后的图像
        cv::Mat resize_image = raw_image;
        // 【核心优化】将高清原图缩放到640x360，大幅降低算法计算量，提升检测速度10倍+
        // 缩放不影响圆环比例检测，保证精度的同时极致提速
        cv::resize(raw_image, resize_image, cv::Size(640, 360));
        
        // 定义灰度图像变量，CCTag算法仅支持灰度图检测
        cv::Mat gray_image;
        // 判断：如果图像是3通道彩色图（相机默认输出BGR）
        if(resize_image.channels() == 3)
        {
            // 将彩色图转换为灰度图（BGR TO GRAY，OpenCV标准转换）
            cv::cvtColor(resize_image, gray_image, cv::COLOR_BGR2GRAY);
        }
        else
        {
            // 如果图像已经是灰度图，直接深拷贝赋值
            gray_image = raw_image.clone();
        }

        // ===================== 7. 调用官方CCTag检测算法 =====================
        // 官方标准CCTag检测函数，执行圆环识别、编码验证、ID匹配
        cctag::cctagDetection(
            markers,        // 输出参数：存储检测到的所有标记结果
            0,              // 管道ID：多线程检测时区分线程，单线程填0
            0,              // 帧序号：图像帧编号，用于追踪，默认填0
            gray_image,     // 输入参数：待检测的灰度图像
            params,         // 输入参数：CCTag配置参数（3环配置）
            bank            // 输入参数：3环标准标记编码库
        );

        // ===================== 8. 检测结果可视化绘制 =====================
        // 克隆灰度图，用于绘制检测结果（不修改原始灰度图）
        cv::Mat show_img = gray_image.clone();

        // 遍历所有检测到的CCTag标记对象
        for(auto& marker : markers)
        {
            // 过滤无效标记：仅处理状态为1的有效检测结果
            if(marker.getStatus() != 1)
                continue;

            // 获取CCTag算法输出的标记中心坐标（算法内部坐标格式）
            auto& cctag_pt = marker.centerImg();
            // 转换为OpenCV可识别的二维坐标点
            cv::Point2d center(
                static_cast<double>(cctag_pt.x()),  // 转换X坐标类型
                static_cast<double>(cctag_pt.y())   // 转换Y坐标类型
            );

            // 绘制标记中心点：红色实心圆，半径5像素
            cv::circle(show_img, center, 5, cv::Scalar(0, 0, 255), -1);
            // 在中心点旁绘制标记ID：绿色字体，字号0.8，粗细2
            cv::putText(show_img,
                "ID: " + std::to_string(marker.id()),  // 显示文本：ID编号
                cv::Point(center.x + 10, center.y - 10),// 文本位置（偏移中心点）
                cv::FONT_HERSHEY_SIMPLEX, 0.8,          // 字体样式+字号
                cv::Scalar(0, 255, 0), 2);              // 字体颜色+粗细
        }

        // ===================== 9. 显示结果+窗口刷新 =====================
        // 在创建的窗口中显示绘制后的结果图像
        cv::imshow("CCTag Detection", show_img);
        // 窗口延时1ms，必须调用：用于刷新图像、响应鼠标键盘事件
        cv::waitKey(1);
    }

    // ===================== 10. 资源释放 =====================
    // 销毁所有OpenCV创建的窗口，释放显存资源
    cv::destroyAllWindows();
}

```

### cctag::Parameters

```c
// ===================== CCTag Parameters 官方默认值（3环标准配置） =====================
// 官方默认环数宏定义：3环（你代码使用的标准配置）
constexpr std::size_t kDefaultNCrowns = 3;

struct Parameters
{
    friend class boost::serialization::access;
    static bool OverrideChecked;
    static bool OverrideLoaded;
    static Parameters Override;
    static void LoadOverride();

    explicit Parameters(std::size_t nCrowns = kDefaultNCrowns);
    ~Parameters( );

    // ===================== 1. 边缘检测默认阈值 =====================
    /// Canny低阈值：默认 100.0
    float _cannyThrLow = 100.0f;
    /// Canny高阈值：默认 140.0
    float _cannyThrHigh = 140.0f;

    // ===================== 2. 弧段搜索/投票默认参数 =====================
    /// 边缘点最大搜索距离(像素)：默认 20
    std::size_t _distSearch = 20;
    /// 梯度幅值过滤阈值：默认 20
    int _thrGradientMagInVote = 20;
    /// 梯度方向最大夹角(角度)：默认 40.0
    float _angleVoting = 40.0f;
    /// 梯度距离比例阈值：默认 0.1
    float _ratioVoting = 0.1f;
    /// 最小平均投票数：默认 30.0
    float _averageVoteMin = 30.0f;
    /// 椭圆拟合中值距离阈值：默认 1.5
    float _thrMedianDistanceEllipse = 1.5f;

    // ===================== 3. 性能/候选数默认参数 =====================
    /// 最大候选种子数：默认 250（调小提速）
    std::size_t _maximumNbSeeds = 250;
    /// 第二轮最大候选数：默认 250
    std::size_t _maximumNbCandidatesLoopTwo = 250;

    // ===================== 4. 核心环数默认参数（必匹配） =====================
    /// 默认环数：3（你的代码必须用这个值）
    std::size_t _nCrowns = 3;
    /// 默认圆环数：6（自动计算 2*3，无需修改）
    std::size_t _nCircles = 6;

    // ===================== 5. 过滤阈值默认参数 =====================
    /// 椭圆最小点数：默认 20
    std::size_t _minPointsSegmentCandidate = 20;
    /// 最小投票数：默认 30
    std::size_t _minVotesToSelectCandidate = 30;
    /// 椭圆鲁棒估计阈值：默认 10.0
    float _threshRobustEstimationOfOuterEllipse = 10.0f;
    /// 椭圆生长壳宽度：默认 1.5
    float _ellipseGrowingEllipticHullWidth = 1.5f;
    /// 内椭圆窗口大小：默认 10
    std::size_t _windowSizeOnInnerEllipticSegment = 10;

    // ===================== 6. 多分辨率（提速核心）默认参数 =====================
    /// 总金字塔层数：默认 4
    std::size_t _numberOfMultiresLayers = 4;
    /// 实际处理层数：默认 4（改 2/3 大幅提速）
    std::size_t _numberOfProcessedMultiresLayers = 4;

    // ===================== 7. ID识别默认参数 =====================
    std::size_t _nSamplesOuterEllipse = 300;
    std::size_t _numCutsInIdentStep = 8;
    std::size_t _numSamplesOuterEdgePointsRefinement = 60;
    std::size_t _cutsSelectionTrials = 40;
    std::size_t _sampleCutLength = 20;
    std::size_t _imagedCenterNGridSample = 5;
    float _imagedCenterNeighbourSize = 0.25f;
    /// 最小识别置信度：默认 0.35（值越高，误检越少）
    float _minIdentProba = 0.35f;

    // ===================== 8. 高级开关默认参数 =====================
    bool _useLMDif = true;
    bool _searchForAnotherSegment = false;
    bool _writeOutput = false;
    /// 默认开启ID识别（必须true）
    bool _doIdentification = true;
    uint32_t _maxEdges = 800000;
    /// 默认关闭GPU加速（CPU运行）
    bool _useCuda = false;

    // ===================== 9. CUDA/调试默认参数 =====================
    size_t _pinnedCounters = 4096;
    size_t _pinnedNearbyPoints = 102400;
    std::string _debugDir = "./debug/";
};

```

### 🔥 你必须掌握的 **5 个核心参数**（3 环检测专用）

1. **`_nCrowns`**
   
   作用：设置 CCTag 环数，**必须和你生成的 3 环标签一致 = 3**

2. **`_numberOfProcessedMultiresLayers`**
   
   作用：多分辨率层数，**调小 = 极速提速，不影响精度**（推荐 2~3）

3. **`_maximumNbSeeds`**
   
   作用：最大候选数，**调小 = 提速，调大 = 更精准**

4. **`_minIdentProba`**
   
   作用：识别置信度，过滤误识别（推荐 0.5~0.8）

5. **`_cannyThrLow / _cannyThrHigh`**
   
   作用：边缘检测阈值，光线暗时调低，光线强时调高

### 🔥 最实用默认值速查（你只需要记这 5 个）

1. **环数**：`_nCrowns = 3`（默认就是 3 环，完美匹配你的代码）
2. **Canny 边缘阈值**：`100 / 140`（通用光线环境）
3. **多分辨率层数**：`4` → **改 2/3 直接提速**
4. **识别置信度**：`0.35` → **改 0.5~0.7 减少误检**----<mark>**这个是错误的**</mark>
5. **最大候选数**：`250` → **改 100 提速**

### 一、基础定义

1. **类名**：`CCTagMarkersBank`
   
   直译：**CCTag 标记编码库 / 官方标准字典**

2. **对象名**：`bank`
   
   就是一个**存储了所有标准 CCTag 编码规则**的字典库

3. **构造参数 `(3)`**
   
   **指定加载【3 环标准 CCTag】的编码字典**（和你生成的 3 环标签、`params._nCrowns=3` 必须完全一致）

```cpp
// 核心代码
cctag::CCTagMarkersBank bank(3); 

```

我用**最直白、结合你代码、逐字拆解**的方式，把 `cctag::CCTag::List markers;` 讲得一清二楚，这是 CCTag 检测**结果存储的核心变量**（不是参数，是容器）：

```cpp
// 核心代码
cctag::CCTag::List markers;

```

# 二、完整可获取参数（按功能分类，基于你提供的头文件）

## 🔴 【最常用】基础识别信息（你代码必用）

表格

| 接口函数                 | 返回值类型            | 含义 & 作用                     | 你的代码用法                        |
| -------------------- | ---------------- | --------------------------- | ----------------------------- |
| `marker.id()`        | `MarkerID` (int) | **获取标签 ID**（0~9，3 环标准 ID）   | `std::to_string(marker.id())` |
| `marker.getStatus()` | `int`            | **获取检测状态码**（1 = 有效，负数 = 无效） | `if(marker.getStatus() != 1)` |
| `marker.quality()`   | `float`          | 获取标签质量分数（越高越精准）             | 过滤低质量标签                       |

## 🟡 坐标定位信息（核心）

表格

| 接口函数                 | 返回值类型          | 含义 & 作用           |
| -------------------- | -------------- | ----------------- |
| `marker.centerImg()` | `Point2d<...>` | **获取标签在图像上的中心坐标** |
| `marker.x()`         | `float`        | 中心 X 坐标（简化调用）     |
| `marker.y()`         | `float`        | 中心 Y 坐标（简化调用）     |

## 🟢 几何形状信息（圆环 / 椭圆）

表格

| 接口函数                            | 返回值类型           | 含义 & 作用                        |
| ------------------------------- | --------------- | ------------------------------ |
| `marker.outerEllipse()`         | `Ellipse`       | 获取标签**最外层椭圆**（完整几何参数：长轴、短轴、角度） |
| `marker.rescaledOuterEllipse()` | `Ellipse`       | 获取缩放后的外层椭圆（多分辨率适配）             |
| `marker.nCircles()`             | `size_t`        | 获取标签圆环数量（3 环 = 6 个圆）           |
| `marker.radiusRatios()`         | `vector<float>` | 获取环宽比例（编码核心）                   |

## 🔵 投影 / 位姿信息（视觉定位用）

表格

| 接口函数                  | 返回值类型             | 含义 & 作用                     |
| --------------------- | ----------------- | --------------------------- |
| `marker.homography()` | `Eigen::Matrix3f` | **获取单应矩阵**（用于 3D 位姿估计、坐标变换） |

## 🟣 图像金字塔 / 缩放信息

表格

| 接口函数                    | 返回值类型   | 含义 & 作用               |
| ----------------------- | ------- | --------------------- |
| `marker.pyramidLevel()` | `int`   | 获取检测时所在的金字塔层数（0 = 原图） |
| `marker.scale()`        | `float` | 获取图像缩放比例              |

## 二、标签的坐标系原点，是不是识别到的圆心？

**是！完全正确**

1. CCTag 官方定义：
   - 世界坐标系原点 = **CCTag 标签的几何中心**
   - 也就是你代码里 `marker.centerImg()` 对应的**物理圆心**
2. 坐标系平面：
   - 标签平面为 `Z=0`
   - X/Y 轴在标签平面内，Z 轴垂直标签平面向外

---

# 极简总结（一句话记住）

1. **`rvec、tvec` = 标签在相机眼里的位置和姿态**（标签→相机）
2. **标签坐标系原点 = 识别到的圆心**，完全重合

我用**最通俗的大白话 + 数学本质 + 你的 CCTag 场景**，把**单应矩阵多解问题、解之间的关系、如何选正确 RT**讲透，这是视觉位姿里的经典知识点，看完你彻底懂：

# 一、核心结论先给你

1. **1 个单应矩阵 `H`，固定分解出 4 组 R、t**（数学必然结果）
2. **4 组解里，只有 1 组是物理真实的**（标记在相机前方）
3. **筛选规则超级简单**：看平移向量 `t` 的 **Z 值 > 0**，就是你要的正确姿态

---

# 二、为什么单应矩阵会分解出 4 组 R、t？

## 根本原因：**2D 图像 → 3D 位姿，天生缺少深度信息，存在二义性**

单应矩阵 `H` 只是**平面图像像素**和**标记平面**的投影关系，**没有深度信息**，数学上会产生**2 种旋转二义性 + 2 种深度二义性**，组合起来 = **4 组合法解**。

### 二义性来源（通俗理解）

1. **旋转二义性（镜面翻转）**
   
   算法分不清：标记是**正放**，还是**绕法线镜像翻转**（像照镜子，2D 看起来一模一样）

2. **深度二义性（前后翻转）**
   
   算法分不清：标记在**相机前方**，还是**相机后方**（2D 投影完全一样）

✅ 2 × 2 = **4 组数学上都对的解**❌ 但物理世界里，只有**前方 + 正放**是真实的

# cctag::cctagDetection 函数参数全解析

`cctag::cctagDetection` 是 **CCTag 视觉标记库**的**核心标准检测函数**（官方推荐调用接口），专门用于**从图像中检测圆形编码 CCTag 标记点**（广泛用于相机标定、三维重建、视觉位姿估计、AR/VR 定位等场景）。

函数签名（对应你的调用）：

```cpp
void cctagDetection(
    std::vector<cctag::Marker>& markers,  // 输出：检测结果
    int pipeId,                           // 输入：流水线ID
    std::size_t frame,                    // 输入：帧号
    const cv::Mat& gray_image,            // 输入：灰度图像
    const cctag::Parameters& params_,    // 输入：检测参数
    const cctag::Bank& bank_             // 输入：标记模板库
);

```

## 逐参数详细说明

### 1. `markers`

- **参数类型**：`std::vector<cctag::Marker>&`（引用传递，**输出参数**）
- **核心含义**：存储**检测到的所有 CCTag 标记点**的结果容器
- **作用**：
  1. 函数执行前：传入空的`vector`，无需手动填充；
  2. 函数执行后：库内部会**自动清空并填充**所有有效 CCTag 标记的详细信息；
  3. 存储内容：每个`cctag::Marker`对象包含标记 ID、中心坐标、角点、旋转角度、置信度、位姿等关键数据，是你后续业务逻辑（如标定、定位）的**直接数据源**。

---

### 2. `pipeId`

- **参数类型**：`int`（**输入参数**）

- **核心含义**：**流水线 ID / 线程分组 ID**

- **作用**：
  
  用于**多线程 / 多流水线并行检测**场景，标识当前检测任务所属的流水线编号；
  
  - **单线程 / 单流水线**（绝大多数场景）：**固定传 `0`**；
  - 多任务并行时：传入不同数字（1、2、3...）区分流水线，库内部会根据 ID 管理线程、缓存，避免资源冲突。

---

### 3. `frame`

- **参数类型**：`std::size_t`（**输入参数**）

- **核心含义**：**图像帧编号**

- **作用**：
  
  用于**视频流 / 连续图像序列**检测场景，标识当前处理的是第几帧；
  
  - **单张静态图像**（绝大多数场景）：**固定传 `0`**；
  - 连续帧处理时：传入递增的帧号（1、2、3...），库内部可利用帧号做**缓存优化、时序跟踪、异常帧过滤**。

---

### 4. `gray_image`

- **参数类型**：`const cv::Mat&`（**输入参数**，OpenCV 图像矩阵）

- **核心含义**：**待检测的单通道灰度图像**

- **作用**：
  
  CCTag 算法**仅支持灰度图像输入**，这是算法的**核心输入源**：
  
  1. 格式要求：必须是 **8 位单通道灰度图（CV_8UC1）**，彩色图像必须先转灰度再传入；
  2. 算法行为：基于这张灰度图做**边缘提取、特征检测、编码解码、标记匹配**等核心操作。

---

### 5. `params_`

- **参数类型**：`const cctag::Parameters&`（**输入参数**）

- **核心含义**：CCTag 检测的**全局配置参数对象**

- **作用**：
  
  算法的 **「配置面板」**，控制所有检测行为，是调优检测精度 / 速度的核心：
  
  - 包含关键配置：边缘提取阈值、标记尺寸范围、解码精度、并行度、调试开关、优化选项等；
  - 使用方式：提前初始化`cctag::Parameters`对象，根据场景修改参数（如远距离小标记、高速实时检测），再传入函数。

---

### 6. `bank_`

- **参数类型**：`const cctag::Bank&`（**输入参数**）

- **核心含义**：CCTag**标记模板库（字典）**

- **作用**：
  
  存储 CCTag 的**标准编码模板集合**，相当于算法的 **「字典本」**：
  
  1. CCTag 是**编码标记**，每个标记有唯一 ID 和固定编码规则；
  2. `bank_`预加载了所有合法的 CCTag 模板，检测时算法会将图像特征与模板匹配，判断是否为有效标记并解码 ID；
  3. 必须提前初始化并加载官方 / 自定义的 CCTag 模板库，**无合法 bank 则无法检测**。

---
