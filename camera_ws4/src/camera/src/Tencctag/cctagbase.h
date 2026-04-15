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
            :bank_(nCrowns)
            {
                //指定环数
                params_._nCrowns = nCrowns;
                //粗筛个数
                params_._maximumNbSeeds = 4;
                //精筛个数
                params_._maximumNbCandidatesLoopTwo = 2;
                std::cout << "params._maximumNbSeeds: " << params_._maximumNbSeeds <<std::endl;
                std::cout << "params._maximumNbCandidatesLoopTwo: " << params_._maximumNbCandidatesLoopTwo <<std::endl;
            }

            /**
             * @brief 输入图片返回官方指定结果容器
             * @param image：RGB图像
             */
            cctag::CCTag::List process(cv::Mat image)       
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
                        0,        // frame
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
