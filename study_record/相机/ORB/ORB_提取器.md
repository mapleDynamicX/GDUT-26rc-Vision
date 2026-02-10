# ORB 提取器

```cpp
#include <vector>
#include <opencv2/core/core.hpp>

/**
 * @brief ORB特征提取器类
 * 负责构建图像金字塔、检测ORB特征点、计算ORB描述子，特征点分布采用八叉树策略
 */
class ORBextractor
{
public:
    
    // 特征点评分方式枚举：0-Harris角点评分（更稳定），1-FAST角点评分（更快）
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
     * @brief 构造函数，初始化ORB特征提取器的核心参数
     * @param nfeatures 期望提取的总特征点数量
     * @param scaleFactor 图像金字塔的尺度因子（如1.2）
     * @param nlevels 图像金字塔的层数
     * @param iniThFAST FAST角点检测的初始阈值
     * @param minThFAST FAST角点检测的最小阈值（初始阈值检测不到时使用）
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    /**
     * @brief 析构函数
     * 空实现，无需要手动释放的资源
     */
    ~ORBextractor(){}

    /**
     * @brief 重载()运算符，核心接口：提取图像的ORB特征点和描述子
     * @param _image 输入图像（灰度图）
     * @param _mask 掩膜（当前版本未使用，预留参数）
     * @param _keypoints 输出的特征点向量（包含尺度、方向等信息）
     * @param _descriptors 输出的ORB描述子（每行对应一个特征点，32字节/512位）
     * @param vLappingArea 输出的特征点重叠区域信息向量
     * @return 实际提取到的特征点数量
     */
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    /**
     * @brief 内联函数：获取图像金字塔的层数
     * @return 金字塔层数nlevels
     */
    int inline GetLevels(){
        return nlevels;}

    /**
     * @brief 内联函数：获取金字塔的基础尺度因子
     * @return 尺度因子scaleFactor
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /**
     * @brief 内联函数：获取各层金字塔的尺度因子向量
     * @return 每层对应的尺度因子（如第0层1.0，第1层1.2，第2层1.44...）
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /**
     * @brief 内联函数：获取各层金字塔的逆尺度因子向量
     * @return 每层尺度因子的倒数，用于坐标映射
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /**
     * @brief 内联函数：获取各层金字塔的尺度sigma平方值
     * @return 每层对应的sigma²，用于高斯滤波/尺度归一化
     */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /**
     * @brief 内联函数：获取各层金字塔的逆尺度sigma平方值
     * @return 每层sigma²的倒数，用于梯度计算归一化
     */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    // 图像金字塔存储容器：每层对应一个缩放后的灰度图像
    std::vector<cv::Mat> mvImagePyramid;

protected:

    /**
     * @brief 构建图像金字塔
     * @param image 输入的原始灰度图像
     * @details 根据预设的层数和尺度因子，生成不同尺度的图像并存储到mvImagePyramid
     */
    void ComputePyramid(cv::Mat image);

    /**
     * @brief 基于八叉树算法分配并检测各层金字塔的特征点
     * @param allKeypoints 输出：每层金字塔对应的特征点向量（二维向量）
     * @details 八叉树策略确保特征点在图像上均匀分布，避免局部密集
     */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    
    /**
     * @brief 八叉树核心实现：在指定图像区域内分配特征点
     * @param vToDistributeKeys 待分配的原始特征点集合
     * @param minX 区域左边界（像素坐标）
     * @param maxX 区域右边界（像素坐标）
     * @param minY 区域下边界（像素坐标）
     * @param maxY 区域上边界（像素坐标）
     * @param nFeatures 该区域需要分配的特征点数量
     * @param level 对应的金字塔层数
     * @return 分配后的特征点向量（数量≤nFeatures，保证均匀分布）
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    /**
     * @brief 旧版本的特征点检测方法（非八叉树分配）
     * @param allKeypoints 输出：每层金字塔对应的特征点向量（二维向量）
     * @details 用于兼容或对比测试，特征点分布不如八叉树均匀
     */
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    // ORB描述子的采样模式：存储FAST角点周围用于计算描述子的像素点对坐标
    std::vector<cv::Point> pattern;

    // 总特征点数量（分配到各层）
    int nfeatures;
    // 金字塔尺度因子（如1.2，表示每层比上一层缩小1.2倍）
    double scaleFactor;
    // 金字塔层数
    int nlevels;
    // FAST角点检测初始阈值（阈值越高，检测到的角点越少、越稳定）
    int iniThFAST;
    // FAST角点检测最小阈值（初始阈值检测不到时降级使用）
    int minThFAST;

    // 每层金字塔分配的特征点数量
    std::vector<int> mnFeaturesPerLevel;

    // 用于ORB描述子计算的u方向最大值数组（优化计算速度）
    std::vector<int> umax;

    // 每层金字塔的尺度因子（第i层 = scaleFactor^i）
    std::vector<float> mvScaleFactor;
    // 每层金字塔的逆尺度因子（1/mvScaleFactor[i]）
    std::vector<float> mvInvScaleFactor;    
    // 每层金字塔的尺度sigma平方（用于尺度归一化）
    std::vector<float> mvLevelSigma2;
    // 每层金字塔的逆尺度sigma平方（1/mvLevelSigma2[i]）
    std::vector<float> mvInvLevelSigma2;
};

```

operator()

函数先构建图像金字塔→检测各层特征点→计算描述子→按 "单目区 / 立体重叠区" 分区存储特征点和描述子，最终返回单目区特征点数量。

ComputeKeyPointsOctTree()

函数先将图像分网格→逐网格用双阈值检测 FAST 角点→收集所有原始角点→八叉树均匀分配特征点→修正坐标 / 尺度信息→计算特征点主方向

### 三、直观对比：定制化 ORB（你的代码） vs OpenCV 原生 ORB

| 维度         | OpenCV 原生 ORB           | 定制化 ORB（你的代码）             |
| ---------- | ----------------------- | ------------------------- |
| 特征点分布      | 按 Harris 打分取 Top-N，局部密集 | 八叉树 + 网格，全局均匀分布           |
| 立体 / 鱼眼适配  | 无定制化逻辑                  | 支持重叠区分区、左右目阈值调整           |
| 边缘特征点过滤    | 通用过滤，无精细化控制             | 精准控制`EDGE_THRESHOLD`，源头过滤 |
| 与 SLAM 耦合性 | 低，需额外转换                 | 高，直接对接后续模块                |
| 阈值控制       | 单一 FAST 阈值              | 双阈值检测，保证检出率               |
| 灵活性        | 低（参数少，逻辑封装）             | 高（可修改任意细节）                |

### 总结

1. **核心结论**：不是 “不用 ORB”，而是 “不用 OpenCV 的通用 ORB”—— 你贴的代码本身就是针对工程场景优化的 ORB 实现，核心算法还是 ORB，只是解决了通用 ORB 在 SLAM / 立体匹配中的痛点。
2. **关键差异**：通用库追求 “易用性”，工程定制版追求 “场景适配性 + 鲁棒性 + 性能”。
3. **通俗理解**：OpenCV 的 ORB 是 “现成的成衣”，尺寸通用但不一定贴合；你的代码是 “量身定制的衣服”，虽然还是衣服（ORB），但完全适配 SLAM / 立体相机的 “身材”。
