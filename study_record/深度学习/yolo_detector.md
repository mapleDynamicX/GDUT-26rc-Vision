# yolo_detector

```cpp
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/BoundingBox2D.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <image_transport/image_transport.h>

class YOLOv11DetectorGPU {
public:
    YOLOv11DetectorGPU() : nh_("~") {
        // 加载相机参数
        loadCameraParams();

        // 参数配置
        std::string model_path;
        nh_.param<std::string>("model_path", model_path, 
            "/root/model/basket11s_openvino_model/basket11s");
        nh_.param<float>("conf_threshold", conf_thres_, 0.5f);
        nh_.param<float>("iou_threshold", iou_thres_, 0.45f);
        nh_.param<std::string>("image_topic", img_topic_, "camera/image");
        nh_.param<int>("queue_size", queue_size_, 2);
        nh_.param<int>("num_threads", num_threads_, 8);

        // 初始化OpenVINO
        core_ = ov::Core();
        core_.set_property("GPU", ov::cache_dir("./cache"));

        // 加载模型
        model_ = core_.read_model(model_path + ".xml", model_path + ".bin");
        compiled_model_ = core_.compile_model(model_, "GPU");
        //compiled_model_ = core_.compile_model(model_, "CPU");
        input_shape_ = compiled_model_.input().get_shape();
        output_shape_ = compiled_model_.output().get_shape();

        // 创建推理请求池
        for(int i=0; i<num_threads_; ++i){
            infer_requests_.push_back(compiled_model_.create_infer_request());
        }

        // 初始化发布器
        //vision_msgs::BoundingBox2DArray​​：消息类型，表示一组二维边界框的集合。
        //适用于目标检测场景，每个边界框包含中心坐标、尺寸、旋转角度等信息。
        bbox_pub_ = nh_.advertise<vision_msgs::BoundingBox2DArray>(
            "/yolo/detections", 5);
        image_transport::ImageTransport it(nh_);
        debug_pub_ = nh_.advertise<sensor_msgs::Image>(
            "/yolo/debug_image", 10);
        fps_pub_ = nh_.advertise<std_msgs::Float32>("/yolo/fps", 10);

        // 启动处理线程
        for(int i=0; i<num_threads_; ++i){
            workers_.emplace_back(&YOLOv11DetectorGPU::worker, this, i);
        }

        // 启动发布线程
        //创建一个独立的线程 publisher_thread_，用于异步执行 YOLOv11DetectorGPU::publishResults 方法，
        //定期发布检测结果（如边界框、类别、置信度等）到 ROS 话题，避免阻塞主线程或图像处理流程。
        publisher_thread_ = std::thread(&YOLOv11DetectorGPU::publishResults, this);

        // 订阅图像话题
        //订阅 ROS 中的图像话题，每当有新图像消息到达时，
        //自动调用 YOLOv11DetectorGPU::imageCallback 方法处理图像数据（如解码、推理、结果存储）。
        image_sub_ = nh_.subscribe(img_topic_, 1, 
            &YOLOv11DetectorGPU::imageCallback, this);
    }
//确保所有工作线程在对象销毁前完成资源释放（如释放 GPU 内存、关闭文件句柄等），避免资源泄漏或程序崩溃。
    ~YOLOv11DetectorGPU() {
        running_ = false;
        for(auto& t : workers_){
            if(t.joinable()) t.join();
        }
        //等待发布线程结束​​
        if(publisher_thread_.joinable()) publisher_thread_.join();
    }

private:
    struct QueueItem {
        std_msgs::Header header;
        cv::Mat image;
    };

    struct Detection {
        float x1, y1, x2, y2;
        float conf;
        int cls_id;
    };

    void loadCameraParams() {
        // 硬编码相机内参矩阵
        camera_matrix_ = (cv::Mat_<double>(3,3) << 
        739.847052, 0.0, 345.063816,   // fx, 0, cx
            0.0, 746.857123, 284.527784,   // 0, fy, cy
            0.0, 0.0, 1.0);      // 0, 0, 1

        // 硬编码畸变系数
        dist_coeffs_ = (cv::Mat_<double>(1,5) << 
        -0.184897, 1.463657, -0.003123, -0.003060, -4.367658);  // k1, k2, p1, p2, k3
        //通过 cv::getOptimalNewCameraMatrix 自动计算优化的内参矩阵
        new_camera_matrix_ = cv::getOptimalNewCameraMatrix(
            camera_matrix_, 
            dist_coeffs_, 
            cv::Size(640, 480),  // 图像宽度和高度
            0
        ).clone();

        ROS_INFO("Hardcoded camera parameters loaded");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            //cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            //cv::imshow("img", image);
            //cv::waitKey(30);
            cv::Mat undistorted;
            cv::undistort(image, undistorted, camera_matrix_, 
                dist_coeffs_, new_camera_matrix_);
            std::cout << undistorted.cols <<std::endl;
            std::cout << undistorted.rows <<std::endl;
            std::cout << undistorted.channels() <<std::endl;
            //使用 std::lock_guard 对互斥锁 queue_mutex_ 进行自动加锁，
            //确保后续的队列操作（检查大小、弹出元素、推入新元素）是​​线程安全​​的
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if(input_queue_.size() >= queue_size_){
                input_queue_.pop();
            }
            input_queue_.push({msg->header, undistorted});

        }
        catch(...) {
            ROS_WARN("Image processing failed");
        }
    }

    void worker(int thread_id) {
        while(running_) {
            QueueItem item;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                if(input_queue_.empty()) continue;
                item = input_queue_.front();
                input_queue_.pop();
            }

            // 预处理
            cv::Mat resized;
            cv::resize(item.image, resized, cv::Size(640, 640));
            //图像预处理并转换为 OpenVINO 张量
            ov::Tensor input_tensor = preprocess(resized);

            // 推理
            auto& req = infer_requests_[thread_id];
            req.set_input_tensor(input_tensor);
            req.start_async();
            req.wait();

            // 后处理
            auto output = req.get_output_tensor();
            auto detections = postprocess(output, item.image.cols, item.image.rows);

            // 存入输出队列
            std::lock_guard<std::mutex> lock(output_mutex_);
            output_queue_.push({item.header, item.image, detections});
        }
    }


    cv::Mat shapeimg(cv::Mat img)
    {

    cv::Mat resize_img;
    cv::resize(img, resize_img, cv::Size(640, 360));
    cv::Mat padding_img;

    cv::copyMakeBorder(resize_img, padding_img, 140, 140, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return padding_img;
    }

    ov::Tensor preprocess(cv::Mat& image) {
        // 调整尺寸
        //cv::resize(image, image, cv::Size(640, 640));
        image = shapeimg(image);
        // BGR转RGB
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        // 转换为浮点并归一化
        cv::Mat float_img;
        image.convertTo(float_img, CV_32F, 1.0/255.0);

        // 手动转置维度 (HWC -> CHW)
        std::vector<cv::Mat> channels(3);
        cv::split(float_img, channels);

        // 创建符合OpenVINO要求的形状 (1, 3, 640, 640)
        ov::Shape input_shape = {1, 3, 640, 640};
        ov::Tensor input_tensor(ov::element::f32, input_shape);

        // 将数据复制到张量
        float* tensor_data = input_tensor.data<float>();
        for(int c = 0; c < 3; ++c) {
            memcpy(tensor_data + c*640 * 640,
                channels[c].data,
                640 * 640*sizeof(float));
        }

        return input_tensor;
    }

    std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h) {
        std::vector<Detection> detections;
        const float* data = output.data<const float>();
        auto shape = output.get_shape();

        // 打印形状信息
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < shape.size(); ++i) {
            oss << shape[i] << (i != shape.size()-1 ? ", " : "]");
        }
        //ROS_INFO("Output tensor shape: %s", oss.str().c_str());

        // 验证基础形状
        if (shape.size() != 3 || shape[0] != 1) {
            ROS_ERROR("Unexpected output shape");
            return detections;
        }

        // 新输出结构解析
        const int num_detections = shape[1];  // 25200
        const int features_per_box = shape[2];// 6
        const int num_classes = 1;            // 根据实际情况调整
        std::cout<<"num_detections"<< num_detections<< std::endl;
        std::cout<<"features_per_box"<< features_per_box<< std::endl;

        //ROS_INFO("Processing %d detections, %d features per box", 
        //        num_detections, features_per_box);

        // 特征索引定义（需根据模型文档确认）
        const int CX_IDX = 0;    // 中心x坐标通道
        const int CY_IDX = 1;    // 中心y坐标通道
        const int W_IDX  = 2;    // 宽度通道
        const int H_IDX  = 3;    // 高度通道
        const int CONF_IDX = 4;  // 综合置信度通道

        for (int i = 0; i < num_detections; ++i) {
            //内存布局为 [1][5][8400]，按通道优先访问

            // float cx = data[CX_IDX * num_detections + i];
            // float cy = data[CY_IDX * num_detections + i];
            // float w  = data[W_IDX * num_detections + i];
            // float h  = data[H_IDX * num_detections + i];
            // float confidence = data[CONF_IDX * num_detections + i];
            float cx = data[i * features_per_box + CX_IDX];
            float cy = data[i * features_per_box + CY_IDX];
            float w  = data[i * features_per_box + W_IDX];
            float h  = data[i * features_per_box + H_IDX];
            float confidence = data[i * features_per_box + CONF_IDX];
            //std::cout<<"confidence"<< confidence<< std::endl;
            // 应用Sigmoid（根据模型输出是否需要）
            // confidence = 1.0f / (1.0f + std::exp(-confidence));

            if (confidence < conf_thres_) continue;

            // 坐标反归一化（假设原始输入为640x640）
            float x1 = (cx - w/2) * orig_w / 640;
            float y1 = (cy - h/2) * orig_h / 640;
            float x2 = (cx + w/2) * orig_w / 640;
            float y2 = (cy + h/2) * orig_h / 640;

            detections.push_back({x1, y1, x2, y2, confidence, 0}); // 假设单类别
        }

        return detections;
    }

    void publishResults() {
        ros::Rate rate(80);
        int fps_counter = 0;
        auto fps_start = ros::Time::now();

        while(running_ && ros::ok()) {
            // 获取检测结果
            std::vector<Detection> detections;
            std_msgs::Header header;
            cv::Mat debug_image;

            {
                std::lock_guard<std::mutex> lock(output_mutex_);
                if(output_queue_.empty()) {
                    rate.sleep();
                    continue;
                }
                auto item = output_queue_.front();
                header = std::get<0>(item);
                debug_image = std::get<1>(item).clone();
                detections = std::get<2>(item);
                output_queue_.pop();
            }

            // 创建消息
            vision_msgs::BoundingBox2DArray bbox_array;
            bbox_array.header = header;

            // 筛选上半部分检测结果
            // float img_height = debug_image.rows;
            // auto it = std::remove_if(detections.begin(), detections.end(),
            //     [img_height](const Detection& d){
            //         return (d.y1 + d.y2)/2 > img_height/2;
            //     });
            // detections.erase(it, detections.end());

            // 选择最佳检测
            if(!detections.empty()) {
                auto best = *std::max_element(detections.begin(), detections.end(),
                    [](const Detection& a, const Detection& b){
                        return a.conf < b.conf;
                    }); 

                // 填充消息
                vision_msgs::BoundingBox2D bbox;
                bbox.center.x = (best.x1 + best.x2)/2;
                bbox.center.y = (best.y1 + best.y2)/2;
                bbox.size_x = best.x2 - best.x1;
                bbox.size_y = best.y2 - best.y1;
                bbox_array.boxes.push_back(bbox);
                std::cout<<"confidence"<< best.conf<< std::endl;
                std::cout << bbox.center.x << "@" <<bbox.center.y << "@" << bbox.size_x << "@" << bbox.size_y << std::endl;

                // 绘制调试图像
                // cv::rectangle(debug_image, 
                //     cv::Point(best.x1, best.y1),
                //     cv::Point(best.x2, best.y2),
                //     cv::Scalar(0,255,0), 2);
                //     std::ostringstream conf_stream;
                // conf_stream << std::fixed << std::setprecision(2) << best.conf;
                // std::string conf_text = "conf: " + conf_stream.str();

                // 自动调整文本位置
                // int baseline = 0;
                // cv::Size text_size = cv::getTextSize(conf_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                // cv::Point text_org(best.x1, best.y1 - 5);

                // 如果文本超出图像顶部，调整到框内底部
                // if (text_org.y < text_size.height) {
                //     text_org.y = best.y1 + text_size.height + 5;
                // }

                // cv::putText(debug_image, conf_text, text_org, 
                //     cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }

            // 发布消息
            sensor_msgs::ImagePtr debug_msg = 
                cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
            bbox_pub_.publish(bbox_array);
            //debug_pub_.publish(debug_msg);

            // 更新FPS
            fps_counter++;
            if((ros::Time::now() - fps_start).toSec() >= 1.0) {
                std_msgs::Float32 fps_msg;
                fps_msg.data = fps_counter/(ros::Time::now() - fps_start).toSec();
                fps_pub_.publish(fps_msg);
                fps_counter = 0;
                fps_start = ros::Time::now();
            }

            rate.sleep();
        }
    }

    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher bbox_pub_, debug_pub_, fps_pub_;

    // OpenVINO相关
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    std::vector<ov::InferRequest> infer_requests_;
    ov::Shape input_shape_, output_shape_;

    // 图像处理
    cv::Mat camera_matrix_, new_camera_matrix_, dist_coeffs_;
    float conf_thres_, iou_thres_;

    // 线程相关
    std::queue<QueueItem> input_queue_;
    std::queue<std::tuple<std_msgs::Header, cv::Mat, std::vector<Detection>>> output_queue_;
    std::mutex queue_mutex_, output_mutex_;
    std::vector<std::thread> workers_;
    std::thread publisher_thread_;
    bool running_ = true;
    int queue_size_, num_threads_;

    std::string img_topic_;
    std::shared_ptr<ov::Model> model_; 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_detector_node");
    YOLOv11DetectorGPU detector;
    std::cout << "undistorted.cols" <<std::endl;
    ros::spin();
    return 0;
}
```

`queue_size_`和`num_threads_`是两个关键的配置参数，分别控制​**​输入队列的容量​**​和​**​并行处理的线程数​**​，共同影响目标检测系统的实时性、稳定性和处理效率

### **一、queue_size_：输入队列的容量限制​**​

它表示​**​输入队列（`input_queue_`）的最大允许长度​**​，用于限制同时等待处理的图像数量

### **​二、num_threads_：并行处理的线程数​**​

它表示​**​同时参与推理和处理的线程数量​**​，直接影响系统的并行处理能力

### **​三、总结​**​

| 参数             | 作用                               | 关键影响                                       |
| -------------- | -------------------------------- | ------------------------------------------ |
| `queue_size_`  | 限制输入队列的最大长度，缓冲图像采集与处理的速率差异       | 防止内存溢出；平衡处理延迟与实时性（过小易导致GPU空闲，过大增加内存占用）     |
| `num_threads_` | 控制并行处理的线程数（推理请求池和工作线程），提升整体处理吞吐量 | 提升GPU/CPU利用率；需与`queue_size_`匹配以避免资源浪费或处理延迟 |

​**​典型场景示例​**​：  
假设摄像头帧率为30FPS（每秒30张图像），单张图像处理耗时（预处理+推理+后处理）为50ms：

- 若`num_threads_=8`，则每秒可处理`8 / 0.05 = 160`张图像，远大于30FPS，队列仅需缓冲少量图像（`queue_size_=2`足够）；
- 若单张处理耗时增至200ms（如复杂场景），`num_threads_=8`可处理`8 / 0.2 = 40`张/秒，仍大于30FPS，队列仍无需过大；
- 若处理耗时进一步增至500ms（如低性能GPU），`num_threads_=8`仅能处理`8 / 0.5 = 16`张/秒，此时需增大`queue_size_`（如设为10）以缓冲更多图像，避免因处理延迟导致丢帧。
