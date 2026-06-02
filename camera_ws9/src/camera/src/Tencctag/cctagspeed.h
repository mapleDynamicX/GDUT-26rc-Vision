#ifndef __CCTAGSPEED_H_
#define __CCTAGSPEED_H_
#include "cctagcoordinate.h"
#include <queue>
#include <thread>
#include <mutex>
#include <vector>
#include <tuple>

namespace Ten
{

    namespace Tencctag
    {
        #define _time_for_delay_ 33000

        // ===================== 1. 核心数据结构 =====================
        // 输入任务：待处理的图像
        struct TaskInput {
            TaskInput(cv::Mat image = cv::Mat(), int frame_num = 0)
            :image_(image)
            {
                frame_num_ = frame_num;
            }
            // 图像数据 + 时间戳
            cv::Mat image_;
            size_t frame_num_ = 0;
        };

        // 输出结果：推理完成的检测框
        struct TaskOutput {
            TaskOutput()
            :image_(cv::Mat())
            {
                biasx_ = 0.0;
                biasy_ = 0.0;
                flag_ = -2;
            }
            // 检测结果数据
            cv::Mat image_;
            double biasx_; 
            double biasy_;
            Ten::XYZRPY pose_;
            int flag_;
        };

        // ===================== 2. 线程池核心类 =====================
        class cctagspeed
        {
        public:
            // 构造：初始化线程池 + 启动所有线程
            /**
             * @param thread_num: 线程数量
             * @param queue_size: 队列大小
             * @param real_outer_radius: 圆半径
             * @param scale 缩放比例
             * @param nCrowns: 指定环数
             */
            cctagspeed(int thread_num = 2, int queue_size = 4, double real_outer_radius = 0.1, double scale = 1.0, int nCrowns = 3)
            :running_(true)
            ,num_threads_(thread_num)
            ,real_outer_radius_(real_outer_radius)
            ,scale_(scale)
            ,nCrowns_(nCrowns)
            {


                // 【核心1】创建固定数量的工作线程（推理线程池）
                for (int i = 0; i < num_threads_; ++i) {
                    workers_.emplace_back(&cctagspeed::worker_loop, this, i);
                }

                // 【核心2】独立发布线程（解耦计算与IO）
                publisher_thread_ = std::thread(&cctagspeed::publish_loop, this);
            }

            // 析构：安全关闭线程池
            ~cctagspeed() {
                running_ = false;
                // 等待所有工作线程退出
                for (auto& t : workers_) if (t.joinable()) t.join();
                // 等待发布线程退出
                if (publisher_thread_.joinable()) publisher_thread_.join();
            }

            void input_image(cv::Mat image)
            {
                static size_t frame_num = 0;
                frame_num++;
                TaskInput task(image.clone(), frame_num);
                submit_task(task);
            }
            // 【生产者接口】往线程池提交任务（ROS图像回调调用）
            void submit_task(const TaskInput& task) {
                std::lock_guard<std::mutex> lock(input_mutex_); // 线程安全加锁
                if (input_queue_.size() >= queue_size_) input_queue_.pop(); // 队列满则丢弃旧任务
                input_queue_.push(task);
            }

        private:
            // ===================== 3. 工作线程（线程池核心） =====================
            // 推理线程循环：不断从队列取任务执行（消费者）
            void worker_loop(int thread_id) {
                std::cout << "worker_loop id: " << thread_id << std::endl;
                //cctag位姿解算
                cctagcoordinate cctag_coordinate(real_outer_radius_, scale_, nCrowns_);
                int flag;
                while (running_) 
                {
                    TaskInput task;
                    // 1. 线程安全地从输入队列取任务
                    bool flag = 0;
                    {
                        std::lock_guard<std::mutex> lock(input_mutex_);
                        flag = input_queue_.empty();
                    }

                    if(flag)
                    {
                        usleep(_time_for_delay_);
                        continue; // 无任务则跳过
                    }
                    else
                    {
                        std::lock_guard<std::mutex> lock(input_mutex_);
                        task = input_queue_.front();
                        input_queue_.pop();
                    }
                    
                    // 2. 执行核心计算：预处理 + 模型推理 + 后处理
                    TaskOutput output;
                    output.flag_ = process_task(cctag_coordinate, task, output);
                    if(output.flag_ == -2)
                    {
                        continue;
                    }

                    // 3. 线程安全地把结果放入输出队列
                    {
                        std::lock_guard<std::mutex> lock(output_mutex_);
                        output_queue_.push(output);
                    }
                }
            }

            // 任务处理（原代码：预处理+OpenVINO推理+后处理）
            int process_task(cctagcoordinate& cctag_coordinate,  TaskInput& task, TaskOutput& output) 
            {
                // 视觉推理核心计算逻辑
                output.image_ = task.image_;
                //处理
                return cctag_coordinate.getpose(output.image_, output.biasx_, output.biasy_, output.pose_, task.frame_num_);
            }

            // ===================== 4. 发布线程 =====================
            // 异步发布结果，不阻塞推理线程
            void publish_loop() {
                auto start = std::chrono::high_resolution_clock::now();
                int num = 0;
                double time = 0;
                while (running_) 
                {
                    TaskOutput result;
                    // 线程安全取结果
                    bool flag = 0;
                    {
                        std::lock_guard<std::mutex> lock(output_mutex_);
                        flag = output_queue_.empty();
                    }

                    if (flag)
                    {
                        usleep(_time_for_delay_);
                        continue;
                    } 
                    else
                    {
                        std::lock_guard<std::mutex> lock(output_mutex_);
                        result = output_queue_.front();
                        output_queue_.pop();
                    }

                    cv::imshow("CCTag Detection", result.image_);
                    cv::waitKey(1);

                    
                    num++;
                    if(num >= 30)
                    {
                        auto end = std::chrono::high_resolution_clock::now();
                        time = std::chrono::duration<double, std::milli>(end - start).count();
                        std::cout << "hz: " << 1000.0 / time * 30.0<< std::endl;
                        start = std::chrono::high_resolution_clock::now();
                        num = 0;   
                    }
                }
            }

   
            

            // ===================== 5. 线程池成员变量 =====================
            bool running_;                          // 线程池运行标志
            int num_threads_;                       // 工作线程数量
            int queue_size_;                    // 任务队列长度
            double real_outer_radius_;
            double scale_;
            int nCrowns_;

            // 线程组
            std::vector<std::thread> workers_;      // 【推理线程池】多个并行工作线程
            std::thread publisher_thread_;          // 【独立发布线程】异步发布

            // 线程安全队列（生产者-消费者缓冲）
            std::queue<TaskInput> input_queue_;     // 输入任务队列
            std::queue<TaskOutput> output_queue_;   // 输出结果队列

            // 互斥锁（保证多线程访问队列安全）
            std::mutex input_mutex_;
            std::mutex output_mutex_;
        };





    }



}








#endif
