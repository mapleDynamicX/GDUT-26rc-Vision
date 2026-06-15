#ifndef __CCTAGBASE_H_
#define __CCTAGBASE_H_
#include <cctag/CCTag.hpp>
#include <cctag/Params.hpp>
#include <cctag/Detection.hpp>
#include <cctag/Types.hpp>

namespace Ten
{

    namespace Tencctag
    {

        class cctagbase
        {
        public:
            /**
             * @param nCrowns: 指定环数
             */
            cctagbase(int nCrowns = 3)
            : bank_(nCrowns)
            {
                // ==========================================
                // 1. 基础配置：指定环数
                // ==========================================
                params_._nCrowns = nCrowns;
                // _nCircles 是根据 _nCrowns 自动计算的衍生参数 (2倍环数)，通常不需要手动改
                // 但为了保险，可以显式同步一下

                //params_._nCircles = 2 * nCrowns;

                // ==========================================
                // 2. 【核心修改】资源扩容：解决双标签资源竞争
                // ==========================================
                
                // 【修改1】粗筛种子点上限：从 4 改为 200 (默认是500，这里取折中值)
                // 作用：决定图像中最多允许多少个“边缘点聚集体”作为初始候选
                // 原因：双标签场景下，两个标签各需要至少几十到上百个种子点
                //       设为200足够支持2-3个标签同时检测，同时避免全图噪点生成过多无效种子

                params_._maximumNbSeeds = 4;
                
                // 【修改2】精筛候选目标上限：从 2 改为 10 (默认是40)
                // 作用：决定在第二轮精细筛选中，最多保留多少个椭圆候选进入最终的ID解码
                // 原因：你只需要检测2个标签，但为了防止误检占用名额，设为10可以留出足够的冗余

                params_._maximumNbCandidatesLoopTwo = 2;

                // ==========================================
                // 3. 【重要修改】边缘检测：降低门槛，让弱标签也能被看到
                // ==========================================
                
                // 【修改3】Canny边缘检测低阈值：从默认 0.01 降低到 0.005
                // 作用：控制边缘检测的灵敏度，值越低，越能检测到微弱的边缘
                // 原因：双标签时，可能有一个标签离得远、光照差，边缘对比度低
                //       降低此值可以避免弱标签的边缘直接被Canny滤波器抹掉

                //params_._cannyThrLow = 0.005f;
                
                // 【修改4】Canny边缘检测高阈值：从默认 0.04 降低到 0.02
                // 作用：Canny算法的双阈值之一，高阈值用于确定强边缘
                // 原因：配合低阈值一起降低，保持高低阈值比为 4:1 (推荐比例)
                //       确保弱标签的边缘轮廓能被完整提取

                //params_._cannyThrHigh = 0.02f;

                // ==========================================
                // 4. 【优化修改】投票与聚类：提升弱标签的存活率
                // ==========================================
                
                // 【修改5】最小投票数阈值：从默认 3 降低到 2
                // 作用：一个边缘点必须收到至少这么多票，才能被选为种子点
                // 原因：弱标签的边缘点可能较少，投票积累慢，降低到2可以让弱标签更容易被激活

                //params_._minVotesToSelectCandidate = 2;

                // 【修改6】搜索距离：从默认 30 增加到 50
                // 作用：边缘点投票时，沿着梯度方向寻找另一个边缘点的最大像素距离
                // 原因：如果标签在画面中比较大（占比高），或者有轻微透视畸变，
                //       增大这个值可以让椭圆轮廓的点更容易匹配到一起，避免标签断裂

                //params_._distSearch = 50;

                // ==========================================
                // 5. 打印调试信息，确认参数生效
                // ==========================================
                std::cout << "===== CCTag 参数初始化 (双标签优化版) =====" << std::endl;
                std::cout << "环数 _nCrowns: " << params_._nCrowns << std::endl;
                std::cout << "粗筛种子数 _maximumNbSeeds: " << params_._maximumNbSeeds << std::endl;
                std::cout << "精筛候选数 _maximumNbCandidatesLoopTwo: " << params_._maximumNbCandidatesLoopTwo << std::endl;
                std::cout << "Canny低阈值 _cannyThrLow: " << params_._cannyThrLow << std::endl;
                std::cout << "Canny高阈值 _cannyThrHigh: " << params_._cannyThrHigh << std::endl;
                std::cout << "============================================" << std::endl;
            }

            /**
             * @brief 输入图片返回官方指定结果容器
             * @param image：RGB图像
             * @param frame_num: 第几帧
             * @return cctag::CCTag::List: 检测链表
             */
            cctag::CCTag::List process(cv::Mat image, size_t frame_num = 0)       
            {
                // 转灰度图
                cv::Mat gray_image;
                cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
                //官方指定结果容器
                cctag::CCTag::List markers;
                try
                {
                    // 官方标准检测函数
                    cctag::cctagDetection(
                        markers,
                        0,        // pipeId
                        frame_num,        // frame
                        gray_image,
                        params_,
                        bank_
                    );
                }
                catch(const std::exception& e)
                {
                    // 打印错误信息，跳过当前帧，程序继续运行
                    std::cout << "CCTag 检测异常: " << e.what() << std::endl;
                    return markers;
                }
                return markers;
            }     

        private:
        cctag::Parameters params_; //CCTag参数
        cctag::CCTagMarkersBank bank_;//3环标记库
        };

    }





}








#endif
