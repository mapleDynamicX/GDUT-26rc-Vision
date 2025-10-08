// jhcap_camera_node.cpp - ROS Camera Node for JingHang Camera
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include "JHCap.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <thread>
#include <atomic>
#include <memory>

class JHCapCameraNode
{
public:
    JHCapCameraNode() : 
        nh_("~"), 
        it_(nh_),
        camera_id_(0),
        running_(false),
        frame_id_("camera"),
        camera_name_("jhcap_camera")
    {
        // Load parameters
        loadParameters();
        
        // Initialize camera info manager
        camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(nh_, camera_name_);
        
        // Initialize publishers
        image_pub_ = it_.advertise("image_raw", 1);
        compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("image_raw/compressed", 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
        
        // Initialize camera
        if (!initializeCamera()) {
            ROS_ERROR("Failed to initialize camera");
            ros::shutdown();
            return;
        }
        
        // Start capture thread
        running_ = true;
        capture_thread_ = std::thread(&JHCapCameraNode::captureLoop, this);
        
        ROS_INFO("JHCap Camera Node initialized successfully");
        ROS_INFO("Publishing on topics:");
        ROS_INFO("  - %s", image_pub_.getTopic().c_str());
        ROS_INFO("  - %s/compressed", image_pub_.getTopic().c_str());
        ROS_INFO("  - %s", camera_info_pub_.getTopic().c_str());
    }
    
    ~JHCapCameraNode()
    {
        shutdown();
    }
    
    void shutdown()
    {
        if (running_) {
            running_ = false;
            if (capture_thread_.joinable()) {
                capture_thread_.join();
            }
            CameraFree(camera_id_);
            ROS_INFO("Camera node shutdown completed");
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher compressed_pub_;
    ros::Publisher camera_info_pub_;
    
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    
    int camera_id_;
    std::atomic<bool> running_;
    std::thread capture_thread_;
    
    // Parameters
    std::string frame_id_;
    std::string camera_name_;
    int image_width_;
    int image_height_;
    int gain_;
    int exposure_;
    double frame_rate_;
    bool auto_exposure_;
    bool auto_gain_;
    int jpeg_quality_;
    
    void loadParameters()
    {
        nh_.param<std::string>("frame_id", frame_id_, "camera");
        nh_.param<std::string>("camera_name", camera_name_, "jhcap_camera");
        nh_.param<int>("camera_id", camera_id_, 0);
        nh_.param<int>("image_width", image_width_, 1920);
        nh_.param<int>("image_height", image_height_, 1080);
        nh_.param<int>("gain", gain_, 32);
        nh_.param<int>("exposure", exposure_, 1000);
        nh_.param<double>("frame_rate", frame_rate_, 30.0);
        nh_.param<bool>("auto_exposure", auto_exposure_, false);
        nh_.param<bool>("auto_gain", auto_gain_, false);
        nh_.param<int>("jpeg_quality", jpeg_quality_, 80);
        
        ROS_INFO("Camera parameters:");
        ROS_INFO("  camera_id: %d", camera_id_);
        ROS_INFO("  frame_id: %s", frame_id_.c_str());
        ROS_INFO("  resolution: %dx%d", image_width_, image_height_);
        ROS_INFO("  gain: %d", gain_);
        ROS_INFO("  exposure: %d", exposure_);
        ROS_INFO("  frame_rate: %.1f", frame_rate_);
        ROS_INFO("  auto_exposure: %s", auto_exposure_ ? "true" : "false");
        ROS_INFO("  auto_gain: %s", auto_gain_ ? "true" : "false");
    }
    
    bool initializeCamera()
    {
        int count = 0;
        if (CameraGetCount(&count) != API_OK) {
            ROS_ERROR("Failed to get camera count");
            return false;
        }
        
        ROS_INFO("Found %d camera(s)", count);
        if (count <= camera_id_) {
            ROS_ERROR("Camera ID %d not available. Found %d cameras.", camera_id_, count);
            return false;
        }
        
        // Get camera name and model
        char name[256], model[256];
        if (CameraGetName(camera_id_, name, model) == API_OK) {
            ROS_INFO("Camera %d: %s (Model: %s)", camera_id_, name, model);
        }
        
        // Initialize camera
        if (CameraInit(camera_id_) != API_OK) {
            ROS_ERROR("Failed to initialize camera %d", camera_id_);
            return false;
        }
        
        // Set camera parameters
        if (CameraSetGain(camera_id_, gain_) != API_OK) {
            ROS_WARN("Failed to set gain to %d", gain_);
        }
        
        if (CameraSetExposure(camera_id_, exposure_) != API_OK) {
            ROS_WARN("Failed to set exposure to %d", exposure_);
        }
        
        if (CameraSetAEC(camera_id_, auto_exposure_) != API_OK) {
            ROS_WARN("Failed to set auto exposure to %s", auto_exposure_ ? "true" : "false");
        }
        
        if (CameraSetAGC(camera_id_, auto_gain_) != API_OK) {
            ROS_WARN("Failed to set auto gain to %s", auto_gain_ ? "true" : "false");
        }
        
        if (CameraSetSnapMode(camera_id_, CAMERA_SNAP_CONTINUATION) != API_OK) {
            ROS_WARN("Failed to set snap mode to continuous");
        }
        
        // Set resolution
        if (CameraSetResolution(camera_id_, 0, &image_width_, &image_height_) != API_OK) {
            ROS_ERROR("Failed to set resolution to %dx%d", image_width_, image_height_);
            return false;
        }
        
        ROS_INFO("Camera initialized with resolution %dx%d", image_width_, image_height_);
        return true;
    }
    
    void captureLoop()
    {
        int buffer_size = 0;
        if (CameraGetImageBufferSize(camera_id_, &buffer_size, CAMERA_IMAGE_RGB24) != API_OK) {
            ROS_ERROR("Failed to get image buffer size");
            return;
        }
        
        cv::Mat image(image_height_, image_width_, CV_8UC3);
        ros::Rate rate(frame_rate_);
        
        ROS_INFO("Starting capture loop at %.1f Hz", frame_rate_);
        
        while (running_ && ros::ok()) {
            int length = buffer_size;
            
            // Capture image from camera
            if (CameraQueryImage(camera_id_, (unsigned char*)image.ptr(), &length, CAMERA_IMAGE_RGB24) == API_OK) {
                
                // Convert RGB to BGR for ROS
                cv::Mat bgr_image;
                cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
                
                // Create timestamp
                ros::Time timestamp = ros::Time::now();
                
                // Publish raw image
                publishRawImage(bgr_image, timestamp);
                
                // Publish compressed image
                //publishCompressedImage(bgr_image, timestamp);
                
                // Publish camera info
                publishCameraInfo(timestamp);
                
            } else {
                ROS_WARN_THROTTLE(1.0, "Failed to query image from camera");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            rate.sleep();
        }
        
        ROS_INFO("Capture loop terminated");
    }
    
    void publishRawImage(const cv::Mat& image, const ros::Time& timestamp)
    {
        if (image_pub_.getNumSubscribers() > 0) {
            try {
                cv_bridge::CvImage cv_image;
                cv_image.header.stamp = timestamp;
                cv_image.header.frame_id = frame_id_;
                cv_image.encoding = sensor_msgs::image_encodings::BGR8;
                cv_image.image = image;
                
                image_pub_.publish(cv_image.toImageMsg());
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }
    }
    
    void publishCompressedImage(const cv::Mat& image, const ros::Time& timestamp)
    {
        if (compressed_pub_.getNumSubscribers() > 0) {
            try {
                sensor_msgs::CompressedImage compressed_msg;
                compressed_msg.header.stamp = timestamp;
                compressed_msg.header.frame_id = frame_id_;
                compressed_msg.format = "jpeg";
                
                // Encode image as JPEG
                std::vector<uchar> buffer;
                std::vector<int> encode_params;
                encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                encode_params.push_back(jpeg_quality_);
                
                if (cv::imencode(".jpg", image, buffer, encode_params)) {
                    compressed_msg.data = buffer;
                    compressed_pub_.publish(compressed_msg);
                } else {
                    ROS_ERROR("Failed to encode image as JPEG");
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception in publishCompressedImage: %s", e.what());
            }
        }
    }
    
    void publishCameraInfo(const ros::Time& timestamp)
    {
        if (camera_info_pub_.getNumSubscribers() > 0) {
            sensor_msgs::CameraInfo info_msg = camera_info_manager_->getCameraInfo();
            info_msg.header.stamp = timestamp;
            info_msg.header.frame_id = frame_id_;
            info_msg.width = image_width_;
            info_msg.height = image_height_;
            
            camera_info_pub_.publish(info_msg);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jhcap_camera_node");
    
    try {
        JHCapCameraNode camera_node;
        
        // Handle shutdown gracefully
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in camera node: %s", e.what());
        return -1;
    }
    
    return 0;
}