// Copyright 2024 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <string>
#include <chrono>
#include <iomanip>
#include <iostream>

// opencv
#include <opencv2/opencv.hpp>

// ros
#include "rclcpp/rclcpp.hpp"

#define RBQ_ROS_MAJOR_VERSION    2
#define RBQ_ROS_MINOR_VERSION    5
#define RBQ_ROS_PATCH_VERSION    1

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define RBQ_ROS_VERSION_STR (VAR_ARG_STRING(RBQ_ROS_MAJOR_VERSION.RBQ_ROS_MINOR_VERSION.RBQ_ROS_PATCH_VERSION))

#define ROS_DEBUG(...) RCLCPP_DEBUG(_logger, __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(_logger, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(_logger, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(_logger, __VA_ARGS__)

// Based on: https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(_logger, msg)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(_logger, msg)
#define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(_logger, msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(_logger, msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(_logger, msg)
#define ROS_DEBUG_STREAM_ONCE(msg) RCLCPP_DEBUG_STREAM_ONCE(_logger, msg)
#define ROS_INFO_STREAM_ONCE(msg) RCLCPP_INFO_STREAM_ONCE(_logger, msg)
#define ROS_WARN_STREAM_COND(cond, msg) RCLCPP_WARN_STREAM_EXPRESSION(_logger, cond, msg)

#define ROS_WARN_ONCE(msg) RCLCPP_WARN_ONCE(_logger, msg)
#define ROS_WARN_COND(cond, ...) RCLCPP_WARN_EXPRESSION(_logger, cond, __VA_ARGS__)

#include <sensor_msgs/msg/image.hpp>
// cv_bridge.h last supported version is humble
#if defined(CV_BRDIGE_HAS_HPP)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
namespace enc = sensor_msgs::image_encodings;

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <eigen3/Eigen/Geometry>

#include "ipc.h"

using namespace std::chrono_literals;

class VisionSubscriber : public rclcpp::Node
{
public:
    struct VisionSensor_t
    {
        Vision_IPC::Sensors_e id;
        bool enabled = false;

        std::string name_ir;
        std::string name_rgb;
        std::string name_depth;
        std::string name_rgb_compressed;
        std::string name_ir_compressed;
        std::string name_camera_info;
        std::string name_rgb_camera_info;
        std::string name_ir_camera_info;
        std::string name_depth_camera_info;
        std::string name_depth_compressed;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_rgb_compressed;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_ir_compressed;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_rgb_camera_info;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_ir_camera_info;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_depth_camera_info;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_depth_compressed;

        void setEnabled(const std::string _name, const Vision_IPC::Sensors_e &_id) {
            id = _id;
            enabled = true;
            name_ir = _name + "/ir";
            name_rgb = _name + "/rgb";
            name_depth = _name + "/depth";
            name_rgb_compressed = _name + "/rgb/compressed";
            name_ir_compressed = _name + "/ir/compressed";
            name_camera_info = _name + "/camera_info";
            name_rgb_camera_info = _name + "/rgb/camera_info";
            name_ir_camera_info = _name + "/ir/camera_info";
            name_depth_camera_info = _name + "/depth/camera_info";
            name_depth_compressed = _name + "/depth/compressed";
        }
    };

    VisionSubscriber(const std::chrono::milliseconds &_m_sleepTime = 20ms)
        : Node("rbq_vision_subscriber")
        , m_sleepTime(_m_sleepTime)
        , _logger(this->get_logger())
    {
        m_ipc = new Vision_IPC();

        // Initialize TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::string name_prepend = "rbq/vision/";
        // Bottom 0~3, Front, Rear
        for(int idx = 0; idx < 6; idx++)
        {
            std::string name = (idx == 4) ? "sensor_front" : (idx == 5) ? "sensor_rear" : QString::asprintf("sensor_bottom_%d", idx).toStdString();
            VisionSensor_t sensor_;
            sensor_.setEnabled(name_prepend + name, (Vision_IPC::Sensors_e)idx);

            // 바닥(0~3번) 센서
            if(idx == 0 || idx == 1 || idx == 2 || idx == 3){
                // RGB subscriber
                // sensor_.sub_rgb = this->create_subscription<sensor_msgs::msg::Image>(
                //     sensor_.name_rgb, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::Image::SharedPtr msg) {
                //         this->rgbCallback(msg, sensor_id);
                //     });
                    
                // sensor_.sub_rgb_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                //     sensor_.name_rgb_compressed, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                //         this->rgbCompressedCallback(msg, sensor_id);
                //     });
                
                // sensor_.sub_rgb_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                //     sensor_.name_rgb_camera_info, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                //         this->rgbCameraInfoCallback(msg, sensor_id);
                //     });
                    
                // // IR subscriber
                // sensor_.sub_ir = this->create_subscription<sensor_msgs::msg::Image>(
                //     sensor_.name_ir, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::Image::SharedPtr msg) {
                //         this->irCallback(msg, sensor_id);
                //     });

                // sensor_.sub_ir_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                //     sensor_.name_ir_compressed, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                //         this->irCompressedCallback(msg, sensor_id);
                //     });
                
                // sensor_.sub_ir_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                //     sensor_.name_ir_camera_info, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                //         this->irCameraInfoCallback(msg, sensor_id);
                //     });

                // // Depth subscriber
                // sensor_.sub_depth = this->create_subscription<sensor_msgs::msg::Image>(
                //     sensor_.name_depth, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::Image::SharedPtr msg) {
                //         this->depthCallback(msg, sensor_id);
                //     });
                
                // sensor_.sub_depth_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                //     sensor_.name_depth_compressed, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                //         this->depthCompressedCallback(msg, sensor_id);
                //     });

                // sensor_.sub_depth_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                //     sensor_.name_depth_camera_info, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                //         this->depthCameraInfoCallback(msg, sensor_id);
                //     });

            }

            // 전후방(4,5번) 센서
            if(idx == 4 || idx == 5) {  
                // // RGB subscriber
                // sensor_.sub_rgb = this->create_subscription<sensor_msgs::msg::Image>(
                //     sensor_.name_rgb, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::Image::SharedPtr msg) {
                //         this->rgbCallback(msg, sensor_id);
                //     });
                    
                sensor_.sub_rgb_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                    sensor_.name_rgb_compressed, 10,
                    [this, sensor_id = sensor_.id](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                        this->rgbCompressedCallback(msg, sensor_id);
                    });
                
                sensor_.sub_rgb_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    sensor_.name_rgb_camera_info, 10,
                    [this, sensor_id = sensor_.id](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                        this->rgbCameraInfoCallback(msg, sensor_id);
                    });
                    
                // // IR subscriber
                // sensor_.sub_ir = this->create_subscription<sensor_msgs::msg::Image>(
                //     sensor_.name_ir, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::Image::SharedPtr msg) {
                //         this->irCallback(msg, sensor_id);
                //     });

                // sensor_.sub_ir_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                //     sensor_.name_ir_compressed, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                //         this->irCompressedCallback(msg, sensor_id);
                //     });
                
                // sensor_.sub_ir_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                //     sensor_.name_ir_camera_info, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                //         this->irCameraInfoCallback(msg, sensor_id);
                //     });

                // // Depth subscriber
                // sensor_.sub_depth = this->create_subscription<sensor_msgs::msg::Image>(
                //     sensor_.name_depth, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::Image::SharedPtr msg) {
                //         this->depthCallback(msg, sensor_id);
                //     });
                
                // sensor_.sub_depth_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                //     sensor_.name_depth_compressed, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                //         this->depthCompressedCallback(msg, sensor_id);
                //     });

                // sensor_.sub_depth_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                //     sensor_.name_depth_camera_info, 10,
                //     [this, sensor_id = sensor_.id](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                //         this->depthCameraInfoCallback(msg, sensor_id);
                //     });

            }
     
            
            m_sensors.push_back(sensor_);
        }
  

        // Create timer for periodic tasks
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&VisionSubscriber::timerCallback, this));

        ROS_INFO("RBQ Vision Subscriber initialized with IPC");
        
        // Initialize memory pools for each sensor
        for(int idx = 0; idx < 6; idx++) {
            Vision_IPC::Sensors_e sensor_id = (Vision_IPC::Sensors_e)idx;
            // Pre-allocate cv::Mat with default size (1280x720)
            image_pool_[sensor_id] = cv::Mat(720, 1280, CV_8UC1);
            // Pre-allocate ImageIR_t structure
            ir_pool_[sensor_id] = Vision_IPC::ImageIR_t();
            
            // Initialize FPS measurement variables
            rgb_fps_count_[sensor_id] = 0;
            ir_fps_count_[sensor_id] = 0;
            depth_fps_count_[sensor_id] = 0;
            rgb_last_time_[sensor_id] = std::chrono::steady_clock::now();
            ir_last_time_[sensor_id] = std::chrono::steady_clock::now();
            depth_last_time_[sensor_id] = std::chrono::steady_clock::now();
        }
    }

    ~VisionSubscriber()
    {
        if(m_ipc != nullptr) {
            delete m_ipc;
            m_ipc = nullptr;
        }
    }

private:

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        try {
            cv::Mat rgb_image = cv_bridge::toCvShare(msg, enc::BGR8)->image;
            

            Vision_IPC::ImageColor_t imageColor_;
            imageColor_.time = this->now().seconds();
            imageColor_.width = rgb_image.cols;
            imageColor_.height = rgb_image.rows;
            
            // Copy image data
            size_t data_size = rgb_image.total() * rgb_image.elemSize();
            if (data_size <= sizeof(imageColor_.data)) {
                memcpy(imageColor_.data, rgb_image.data, data_size);
            } else {
                ROS_WARN("RGB image too large for IPC buffer: %zu > %zu", data_size, sizeof(imageColor_.data));
                return;
            }

            // Set camera intrinsics - use stored values if available, otherwise default
            if (camera_info_received_[sensor_id]) {
                // Use stored camera info (fixed values once received)
                auto& info = camera_info_[sensor_id];
                imageColor_.intrinsics[0] = info.k[0];  // fx
                imageColor_.intrinsics[1] = info.k[4];  // fy
                imageColor_.intrinsics[2] = info.k[2];  // ppx
                imageColor_.intrinsics[3] = info.k[5];  // ppy
                
                // Set distortion coefficients
                for (int i = 0; i < 8 && i < info.d.size(); i++) {
                    imageColor_.coeffs[i] = info.d[i];
                }
            } else {
                // Default intrinsics (only used before camera info is received)
                imageColor_.intrinsics[0] = 500.0; // fx
                imageColor_.intrinsics[1] = 500.0; // fy
                imageColor_.intrinsics[2] = msg->width / 2.0;  // ppx
                imageColor_.intrinsics[3] = msg->height / 2.0; // ppy
            }

            // Store to IPC
            Vision_IPC::Error_e result = m_ipc->setImageColor(sensor_id, imageColor_);
            if (result != Vision_IPC::Error_e::noError) {
                ROS_ERROR("Failed to store RGB image to IPC for sensor %d", sensor_id);
            } else {
                ROS_DEBUG("Stored RGB image to IPC for sensor %d: %dx%d", sensor_id, rgb_image.cols, rgb_image.rows);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception in RGB callback: %s", e.what());
        }
    }

    void rgbCompressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        try {
            cv::Mat compressed_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (!compressed_image.empty()) {
                // OAK-D provides BGR format, no conversion needed
                // cv::cvtColor(compressed_image, compressed_image, cv::COLOR_BGR2RGB);  // 제거
                
                Vision_IPC::ImageColor_t imageColor_;
                imageColor_.time = this->now().seconds();
                imageColor_.width = compressed_image.cols;
                imageColor_.height = compressed_image.rows;
                
                // Copy image data
                size_t data_size = compressed_image.total() * compressed_image.elemSize();
                if (data_size <= sizeof(imageColor_.data)) {
                    memcpy(imageColor_.data, compressed_image.data, data_size);
                } else {
                    ROS_WARN("Compressed image too large for IPC buffer: %zu > %zu", data_size, sizeof(imageColor_.data));
                    return;
                }

                // Set camera intrinsics - use stored values if available, otherwise default
                if (camera_info_received_[sensor_id]) {
                    // Use stored camera info (fixed values once received)
                    auto& info = camera_info_[sensor_id];
                    imageColor_.intrinsics[0] = info.k[0];  // fx
                    imageColor_.intrinsics[1] = info.k[4];  // fy
                    imageColor_.intrinsics[2] = info.k[2];  // ppx
                    imageColor_.intrinsics[3] = info.k[5];  // ppy
                    
                    // Set distortion coefficients
                    for (int i = 0; i < 8 && i < info.d.size(); i++) {
                        imageColor_.coeffs[i] = info.d[i];
                    }
                    
                    // Log intrinsic and distortion values
                    // ROS_INFO("RGB Compressed - Sensor%d: Using camera info - fx:%.2f, fy:%.2f, cx:%.2f, cy:%.2f", 
                    //          (int)sensor_id, imageColor_.intrinsics[0], imageColor_.intrinsics[1], 
                    //          imageColor_.intrinsics[2], imageColor_.intrinsics[3]);
                    // ROS_INFO("RGB Compressed - Sensor%d: Distortion coeffs: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", 
                    //          (int)sensor_id, imageColor_.coeffs[0], imageColor_.coeffs[1], imageColor_.coeffs[2], imageColor_.coeffs[3],
                    //          imageColor_.coeffs[4], imageColor_.coeffs[5], imageColor_.coeffs[6], imageColor_.coeffs[7]);
                } else {
                    // Default intrinsics (only used before camera info is received)
                    if (sensor_id == Vision_IPC::Sensors_e::Front) {  // sensor4
                        imageColor_.intrinsics[0] = 1154.58; // fx
                        imageColor_.intrinsics[1] = 1154.15; // fy
                        imageColor_.intrinsics[2] = 972.45; // cx
                        imageColor_.intrinsics[3] = 538.69; // cy
                        // distortion coefficients
                        imageColor_.coeffs[0] = 12.4791; imageColor_.coeffs[1] = 8.9469;
                        imageColor_.coeffs[2] =  0.0000; imageColor_.coeffs[3] = 0.0001;
                        imageColor_.coeffs[4] = -2.5814; imageColor_.coeffs[5] = 12.4305;
                        imageColor_.coeffs[6] = 12.8340; imageColor_.coeffs[7] = -2.4814;
                    } else if (sensor_id == Vision_IPC::Sensors_e::Rear) {  // sensor5
                        imageColor_.intrinsics[0] = 1152.72; // fx
                        imageColor_.intrinsics[1] = 1152.16; // fy
                        imageColor_.intrinsics[2] = 949.22; // cx
                        imageColor_.intrinsics[3] = 552.83; // cy
                        // distortion coefficients
                        imageColor_.coeffs[0] = 12.0450; imageColor_.coeffs[1] = 9.8536;
                        imageColor_.coeffs[2] =  0.0011; imageColor_.coeffs[3] = -0.0001;
                        imageColor_.coeffs[4] = -2.5503; imageColor_.coeffs[5] = 12.0065;
                        imageColor_.coeffs[6] = 13.6372; imageColor_.coeffs[7] = -2.2479;
                    } 

                    ROS_WARN("RGB Compressed - Sensor%d: Using uninitialized camera info - fx:%.2f, fy:%.2f, cx:%.2f, cy:%.2f", 
                            (int)sensor_id, imageColor_.intrinsics[0], imageColor_.intrinsics[1], 
                            imageColor_.intrinsics[2], imageColor_.intrinsics[3]);
                }

                // Store to IPC
                Vision_IPC::Error_e result = m_ipc->setImageColor(sensor_id, imageColor_);
                if (result != Vision_IPC::Error_e::noError) {
                    ROS_ERROR("Failed to store compressed image to IPC for sensor %d", sensor_id);
                } else {
                    ROS_DEBUG("Stored compressed image to IPC for sensor %d: %dx%d", sensor_id, compressed_image.cols, compressed_image.rows);
                }
                
                // FPS 측정
                rgb_fps_count_[sensor_id]++;
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - rgb_last_time_[sensor_id]).count();
                
                if (elapsed >= 1.0) {  // 1초마다 FPS 출력
                    double fps = rgb_fps_count_[sensor_id] / elapsed;
                    std::cout << "RGB Compressed - Sensor " << (int)sensor_id << " FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
                    rgb_fps_count_[sensor_id] = 0;
                    rgb_last_time_[sensor_id] = current_time;
                }
            }
        } catch (cv::Exception& e) {
            ROS_ERROR("OpenCV exception in compressed callback: %s", e.what());
        }
    }

    void rgbCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        // Store camera info for later use
        camera_info_[sensor_id] = *msg;
        camera_info_received_[sensor_id] = true; // Mark camera info as received
        ROS_DEBUG("Received RGB camera info for sensor %d: %dx%d", sensor_id, msg->width, msg->height);
    }

    void irCallback(const sensor_msgs::msg::Image::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        try {
            cv::Mat ir_image = cv_bridge::toCvShare(msg, enc::MONO8)->image;
            
            Vision_IPC::ImageIR_t imageIR_;
            imageIR_.time = this->now().seconds();
            imageIR_.width = ir_image.cols;
            imageIR_.height = ir_image.rows;
            
            // Copy image data
            size_t data_size = ir_image.total() * ir_image.elemSize();
            if (data_size <= sizeof(imageIR_.data)) {
                memcpy(imageIR_.data, ir_image.data, data_size);
            } else {
                ROS_WARN("IR image too large for IPC buffer: %zu > %zu", data_size, sizeof(imageIR_.data));
                return;
            }

            // Set camera intrinsics - use stored values if available, otherwise default
            if (camera_info_received_[sensor_id]) {
                // Use stored camera info (fixed values once received)
                auto& info = camera_info_[sensor_id];
                imageIR_.intrinsics[0] = info.k[0];  // fx
                imageIR_.intrinsics[1] = info.k[4];  // fy
                imageIR_.intrinsics[2] = info.k[2];  // ppx
                imageIR_.intrinsics[3] = info.k[5];  // ppy
                
                // Set distortion coefficients
                for (int i = 0; i < 8 && i < info.d.size(); i++) {
                    imageIR_.coeffs[i] = info.d[i];
                }
            } else {
                // Default intrinsics (only used before camera info is received)
                imageIR_.intrinsics[0] = 500.0; // fx
                imageIR_.intrinsics[1] = 500.0; // fy
                imageIR_.intrinsics[2] = msg->width / 2.0;  // ppx
                imageIR_.intrinsics[3] = msg->height / 2.0; // ppy
            }

            // Store to IPC
            Vision_IPC::Error_e result = m_ipc->setImageIR(sensor_id, imageIR_);
            if (result != Vision_IPC::Error_e::noError) {
                ROS_ERROR("Failed to store IR image to IPC for sensor %d", sensor_id);
            } else {
                ROS_DEBUG("Stored IR image to IPC for sensor %d: %dx%d", sensor_id, ir_image.cols, ir_image.rows);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception in IR callback: %s", e.what());
        }
    }

    void irCompressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        try {
            // Use memory pool for cv::Mat
            cv::Mat& ir_image = image_pool_[sensor_id];
            ir_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
            
            if (!ir_image.empty()) {
                // Use memory pool for ImageIR_t
                Vision_IPC::ImageIR_t& imageIR_ = ir_pool_[sensor_id];
                imageIR_.time = this->now().seconds();
                imageIR_.width = ir_image.cols;
                imageIR_.height = ir_image.rows;
                
                // Copy image data
                size_t data_size = ir_image.total() * ir_image.elemSize();
                if (data_size <= sizeof(imageIR_.data)) {
                    memcpy(imageIR_.data, ir_image.data, data_size);
                } else {
                    ROS_WARN("IR compressed image too large for IPC buffer: %zu > %zu", data_size, sizeof(imageIR_.data));
                    return;
                }

                // Set camera intrinsics - use stored values if available, otherwise default
                if (camera_info_received_[sensor_id]) {
                    // Use stored camera info (fixed values once received)
                    auto& info = camera_info_[sensor_id];
                    imageIR_.intrinsics[0] = info.k[0];  // fx
                    imageIR_.intrinsics[1] = info.k[4];  // fy
                    imageIR_.intrinsics[2] = info.k[2];  // ppx
                    imageIR_.intrinsics[3] = info.k[5];  // ppy
                    
                    // Set distortion coefficients
                    for (int i = 0; i < 8 && i < info.d.size(); i++) {
                        imageIR_.coeffs[i] = info.d[i];
                    }
                    
                    // Log intrinsic and distortion values
                    // ROS_INFO("IR Compressed - Sensor%d: Using camera info - fx:%.2f, fy:%.2f, cx:%.2f, cy:%.2f", 
                    //          (int)sensor_id, imageIR_.intrinsics[0], imageIR_.intrinsics[1], 
                    //          imageIR_.intrinsics[2], imageIR_.intrinsics[3]);
                    // ROS_INFO("IR Compressed - Sensor%d: Distortion coeffs: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", 
                    //          (int)sensor_id, imageIR_.coeffs[0], imageIR_.coeffs[1], imageIR_.coeffs[2], imageIR_.coeffs[3],
                    //          imageIR_.coeffs[4], imageIR_.coeffs[5], imageIR_.coeffs[6], imageIR_.coeffs[7]);
                } else {
                    // Default intrinsics (only used before camera info is received)
                    if (sensor_id == Vision_IPC::Sensors_e::Front) {  // sensor4
                        imageIR_.intrinsics[0] = 570.06; // fx
                        imageIR_.intrinsics[1] = 569.84; // fy
                        imageIR_.intrinsics[2] = 649.29; // cx
                        imageIR_.intrinsics[3] = 338.81; // cy
                        // distortion coefficients [전부 0]
                        for (int i = 0; i < 8; i++) {
                            imageIR_.coeffs[i] = 0.0;
                        }
                    } else if (sensor_id == Vision_IPC::Sensors_e::Rear) {  // sensor5
                        imageIR_.intrinsics[0] = 572.69; // fx
                        imageIR_.intrinsics[1] = 572.59; // fy
                        imageIR_.intrinsics[2] = 646.69; // cx
                        imageIR_.intrinsics[3] = 339.94; // cy

                        for (int i = 0; i < 8; i++) {
                            imageIR_.coeffs[i] = 0.0;
                        }
                        // // distortion coefficients
                        // imageIR_.coeffs[0] = 12.0450; imageIR_.coeffs[1] = 9.8536;
                        // imageIR_.coeffs[2] = 0.0011; imageIR_.coeffs[3] = -0.0001;
                        // imageIR_.coeffs[4] = -2.5503; imageIR_.coeffs[5] = 12.0065;
                        // imageIR_.coeffs[6] = 13.6372; imageIR_.coeffs[7] = -2.2479;
                    } 
                    
                    ROS_WARN("IR Compressed - Sensor%d: Using uninitialized camera info - fx:%.2f, fy:%.2f, cx:%.2f, cy:%.2f", 
                             (int)sensor_id, imageIR_.intrinsics[0], imageIR_.intrinsics[1], 
                             imageIR_.intrinsics[2], imageIR_.intrinsics[3]);
                }

                // Store to IPC
                Vision_IPC::Error_e result = m_ipc->setImageIR(sensor_id, imageIR_);
                if (result != Vision_IPC::Error_e::noError) {
                    ROS_ERROR("Failed to store IR compressed image to IPC for sensor %d", sensor_id);
                } else {
                    ROS_DEBUG("Stored IR compressed image to IPC for sensor %d: %dx%d", sensor_id, ir_image.cols, ir_image.rows);
                }
                
                // FPS 측정
                ir_fps_count_[sensor_id]++;
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - ir_last_time_[sensor_id]).count();
                
                if (elapsed >= 1.0) {  // 1초마다 FPS 출력
                    double fps = ir_fps_count_[sensor_id] / elapsed;
                    std::cout << "IR Compressed - Sensor " << (int)sensor_id << " FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
                    ir_fps_count_[sensor_id] = 0;
                    ir_last_time_[sensor_id] = current_time;
                }
            }
        } catch (cv::Exception& e) {
            ROS_ERROR("OpenCV exception in IR compressed callback: %s", e.what());
        }
    }

    void irCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        // Store camera info for later use
        camera_info_[sensor_id] = *msg;
        camera_info_received_[sensor_id] = true; // Mark camera info as received
        ROS_DEBUG("Received IR camera info for sensor %d: %dx%d", sensor_id, msg->width, msg->height);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        try {
            cv::Mat depth_image = cv_bridge::toCvShare(msg, enc::TYPE_16UC1)->image;
            
            Vision_IPC::Depth_t depth_;
            depth_.time = this->now().seconds();
            depth_.width = depth_image.cols;
            depth_.height = depth_image.rows;
            
            // Copy depth data
            size_t data_size = depth_image.total() * depth_image.elemSize();
            if (data_size <= sizeof(depth_.data)) {
                memcpy(depth_.data, depth_image.data, data_size);
            } else {
                ROS_WARN("Depth image too large for IPC buffer: %zu > %zu", data_size, sizeof(depth_.data));
                return;
            }

            // Set camera intrinsics - use stored values if available, otherwise default
            if (camera_info_received_[sensor_id]) {
                // Use stored camera info (fixed values once received)
                auto& info = camera_info_[sensor_id];
                depth_.intrinsics[0] = info.k[0];  // fx
                depth_.intrinsics[1] = info.k[4];  // fy
                depth_.intrinsics[2] = info.k[2];  // ppx
                depth_.intrinsics[3] = info.k[5];  // ppy
                
                // Set distortion coefficients
                for (int i = 0; i < 8 && i < info.d.size(); i++) {
                    depth_.coeffs[i] = info.d[i];
                }
            } else {
                // Default intrinsics (only used before camera info is received)
                depth_.intrinsics[0] = 500.0; // fx
                depth_.intrinsics[1] = 500.0; // fy
                depth_.intrinsics[2] = msg->width / 2.0;  // ppx
                depth_.intrinsics[3] = msg->height / 2.0; // ppy
            }

            // Store to IPC
            Vision_IPC::Error_e result = m_ipc->setDepth(sensor_id, depth_);
            if (result != Vision_IPC::Error_e::noError) {
                ROS_ERROR("Failed to store depth image to IPC for sensor %d", sensor_id);
            } else {
                ROS_DEBUG("Stored depth image to IPC for sensor %d: %dx%d", sensor_id, depth_image.cols, depth_image.rows);
            }
            
            // FPS 측정
            depth_fps_count_[sensor_id]++;
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - depth_last_time_[sensor_id]).count();
            
            if (elapsed >= 1.0) {  // 1초마다 FPS 출력
                double fps = depth_fps_count_[sensor_id] / elapsed;
                std::cout << "Depth - Sensor " << (int)sensor_id << " FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
                depth_fps_count_[sensor_id] = 0;
                depth_last_time_[sensor_id] = current_time;
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception in depth callback: %s", e.what());
        }
    }
    
    void depthCompressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        try {
            // Decode compressed depth image
            cv::Mat depth_image = cv::imdecode(cv::InputArray(msg->data), cv::IMREAD_UNCHANGED);
            
            if (depth_image.empty()) {
                ROS_ERROR("Failed to decode compressed depth image for sensor %d", sensor_id);
                return;
            }
            
            // Check if it's 16-bit depth image
            if (depth_image.type() != CV_16UC1) {
                ROS_WARN("Compressed depth image is not 16UC1 format for sensor %d, converting...", sensor_id);
                depth_image.convertTo(depth_image, CV_16UC1);
            }
            
            Vision_IPC::Depth_t depth_;
            depth_.time = this->now().seconds();
            depth_.width = depth_image.cols;
            depth_.height = depth_image.rows;
            
            // Copy depth data
            size_t data_size = depth_image.total() * depth_image.elemSize();
            if (data_size <= sizeof(depth_.data)) {
                memcpy(depth_.data, depth_image.data, data_size);
            } else {
                ROS_WARN("Compressed depth image too large for IPC buffer: %zu > %zu", data_size, sizeof(depth_.data));
                return;
            }

            // Set camera intrinsics - use stored values if available, otherwise default
            if (camera_info_received_[sensor_id]) {
                // Use stored camera info (fixed values once received)
                auto& info = camera_info_[sensor_id];
                depth_.intrinsics[0] = info.k[0];  // fx
                depth_.intrinsics[1] = info.k[4];  // fy
                depth_.intrinsics[2] = info.k[2];  // ppx
                depth_.intrinsics[3] = info.k[5];  // ppy
                
                // Set distortion coefficients
                for (int i = 0; i < 8 && i < info.d.size(); i++) {
                    depth_.coeffs[i] = info.d[i];
                }
            } else {
                // Default intrinsics (only used before camera info is received)
                depth_.intrinsics[0] = 500.0; // fx
                depth_.intrinsics[1] = 500.0; // fy
                depth_.intrinsics[2] = depth_image.cols / 2.0;  // ppx
                depth_.intrinsics[3] = depth_image.rows / 2.0; // ppy
            }

            // Store to IPC
            Vision_IPC::Error_e result = m_ipc->setDepth(sensor_id, depth_);
            if (result != Vision_IPC::Error_e::noError) {
                ROS_ERROR("Failed to store compressed depth image to IPC for sensor %d", sensor_id);
            } else {
                ROS_DEBUG("Stored compressed depth image to IPC for sensor %d: %dx%d", sensor_id, depth_image.cols, depth_image.rows);
            }
            
            // FPS 측정
            depth_fps_count_[sensor_id]++;
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - depth_last_time_[sensor_id]).count();
            
            if (elapsed >= 1.0) {  // 1초마다 FPS 출력
                double fps = depth_fps_count_[sensor_id] / elapsed;
                std::cout << "Depth Compressed - Sensor " << (int)sensor_id << " FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
                depth_fps_count_[sensor_id] = 0;
                depth_last_time_[sensor_id] = current_time;
            }
        } catch (cv::Exception& e) {
            ROS_ERROR("OpenCV exception in depth compressed callback: %s", e.what());
        }
    }
 
    void depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, Vision_IPC::Sensors_e sensor_id)
    {
        // Store depth camera info for later use
        camera_info_[sensor_id] = *msg;
        camera_info_received_[sensor_id] = true; // Mark camera info as received
        ROS_DEBUG("Received Depth camera info for sensor %d: %dx%d", sensor_id, msg->width, msg->height);
    }


    void timerCallback()
    {
        // Periodic tasks 
        checkTFTransforms();
    }

    void checkTFTransforms()
    {
        // Check if TF transforms are available for each sensor
        for (auto& sensor : m_sensors) {
            if (sensor.enabled) {
                try {
                    geometry_msgs::msg::TransformStamped transform = 
                        tf_buffer_->lookupTransform("base", sensor.name_depth, tf2::TimePointZero);
                    // TF is available, could be used for additional processing
                } catch (tf2::TransformException& ex) {
                    // TF not available, this is normal for some sensors
                }
            }
        }
    }

    std::chrono::milliseconds m_sleepTime = 100ms;
    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<VisionSensor_t> m_sensors;
    std::map<Vision_IPC::Sensors_e, sensor_msgs::msg::CameraInfo> camera_info_;
    std::map<Vision_IPC::Sensors_e, bool> camera_info_received_;  // Track if camera info was ever received

    // Memory pools for performance optimization
    std::map<Vision_IPC::Sensors_e, cv::Mat> image_pool_;
    std::map<Vision_IPC::Sensors_e, Vision_IPC::ImageIR_t> ir_pool_;
    
    // FPS 측정을 위한 변수들
    std::map<Vision_IPC::Sensors_e, int> rgb_fps_count_;
    std::map<Vision_IPC::Sensors_e, int> ir_fps_count_;
    std::map<Vision_IPC::Sensors_e, int> depth_fps_count_;
    std::map<Vision_IPC::Sensors_e, std::chrono::steady_clock::time_point> rgb_last_time_;
    std::map<Vision_IPC::Sensors_e, std::chrono::steady_clock::time_point> ir_last_time_;
    std::map<Vision_IPC::Sensors_e, std::chrono::steady_clock::time_point> depth_last_time_;

    Vision_IPC* m_ipc = nullptr;

    rclcpp::Logger _logger;
}; 