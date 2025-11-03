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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>

#include "ipc.h"

using namespace std::chrono_literals;

class VisionPublisher : public rclcpp::Node
{
public:
    VisionPublisher(const std::chrono::milliseconds &_m_sleepTime = 20ms)
        : Node("rbq_vision_publisher")
        , m_sleepTime(_m_sleepTime)
        , _logger(this->get_logger())
    {
        m_ipc = new Vision_IPC();

        std::string name_prepend = "rbq/vision/";
        // Bottom 0~3, Front, Rear
        for(int idx = 0; idx < 6; idx++)
        {
            std::string name = (idx == 4) ? "sensor_front" : (idx == 5) ? "sensor_rear" : QString::asprintf("sensor_bottom_%d", idx).toStdString();
            VisionSensor_t sensor_;
            sensor_.setEnabled(name_prepend + name, (Vision_IPC::Sensors_e)idx);

            if(idx == 0 || idx == 1 || idx == 2 || idx == 3) {
                sensor_.setEnabledIR(this);                
                sensor_.setEnabledIRCompressed(this);
                sensor_.setEnabledIRCameraInfo(this);

                sensor_.setEnabledDepth(this);
                sensor_.setEnabledDepthCompressed(this);
                sensor_.setEnabledDepthCameraInfo(this);
            }

            if(idx == 4 || idx == 5) {
                sensor_.setEnabledRGB(this);
                sensor_.setEnabledRGBCompressed(this);
                sensor_.setEnabledRGBCameraInfo(this);

                sensor_.setEnabledIR(this);                
                sensor_.setEnabledIRCompressed(this);
                sensor_.setEnabledIRCameraInfo(this);

                sensor_.setEnabledDepth(this);
                sensor_.setEnabledDepthCompressed(this);
                sensor_.setEnabledDepthCameraInfo(this);
            }

            m_sensors.push_back(sensor_);
        }

        m_timer = this->create_wall_timer(m_sleepTime, std::bind(&VisionPublisher::publishOnce, this));

        restartStaticTransformBroadcaster();
    }

    ~VisionPublisher()
    {
        delete m_ipc;
    }

protected:
    class VisionSensor_t {
    public:
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_rgb_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ir_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb, pub_ir, pub_depth;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_rgb_compressed, pub_ir_compressed, pub_depth_compressed;
        std::string name, name_rgb, name_ir, name_depth, name_camera_info, name_rgb_compressed, name_ir_compressed, name_depth_compressed = "";
        std::string name_rgb_camera_info, name_ir_camera_info, name_depth_camera_info = "";
        bool enabled, enabled_rgb, enabled_ir, enabled_depth, enabled_pcd, enabled_camera_info = false;
        Vision_IPC::Sensors_e id;
        void setEnabled(const std::string _name, const Vision_IPC::Sensors_e &_id) {
            enabled = true;
            name = _name;
            id = _id;
        }
        void setEnabledRGB(rclcpp::Node *_node) {
            enabled_rgb = true;
            name_rgb = name+"/rgb";
            pub_rgb = _node->create_publisher<sensor_msgs::msg::Image>(name_rgb, 10);
        }
        void setEnabledRGBCompressed(rclcpp::Node *_node) {
            name_rgb_compressed = name_rgb + "/compressed";
            pub_rgb_compressed = _node->create_publisher<sensor_msgs::msg::CompressedImage>(name_rgb_compressed, 10);
        }
        void setEnabledIR(rclcpp::Node *_node) {
            enabled_ir = true;
            name_ir = name+"/ir";
            pub_ir = _node->create_publisher<sensor_msgs::msg::Image>(name_ir, 10);
        }
        void setEnabledIRCompressed(rclcpp::Node *_node) {
            name_ir_compressed = name_ir + "/compressed";
            pub_ir_compressed = _node->create_publisher<sensor_msgs::msg::CompressedImage>(name_ir_compressed, 10);
        }
        void setEnabledRGBCameraInfo(rclcpp::Node *_node) {
            name_rgb_camera_info = name_rgb + "/camera_info";
            pub_rgb_camera_info = _node->create_publisher<sensor_msgs::msg::CameraInfo>(name_rgb_camera_info, 10);
        }
        void setEnabledIRCameraInfo(rclcpp::Node *_node) {
            name_ir_camera_info = name_ir + "/camera_info";
            pub_ir_camera_info = _node->create_publisher<sensor_msgs::msg::CameraInfo>(name_ir_camera_info, 10);
        }


        void setEnabledDepth(rclcpp::Node *_node) {
            enabled_depth = true;
            name_depth = name + "/depth";
            pub_depth = _node->create_publisher<sensor_msgs::msg::Image>(name_depth, 10);
        }
        void setEnabledDepthCameraInfo(rclcpp::Node *_node) {
            name_depth_camera_info = name_depth + "/camera_info";
            pub_depth_camera_info = _node->create_publisher<sensor_msgs::msg::CameraInfo>(name_depth_camera_info, 10);
        }
        void setEnabledDepthCompressed(rclcpp::Node *_node) {
            name_depth_compressed = name_depth + "/compressed";
            pub_depth_compressed = _node->create_publisher<sensor_msgs::msg::CompressedImage>(name_depth_compressed, 10);
        }

        bool getSubCountIR() {
            return enabled_ir && pub_ir && pub_ir->get_subscription_count() > 0;
        }
        bool getSubCountRgb() {
            return enabled_rgb && pub_rgb && pub_rgb->get_subscription_count() > 0;
        }
        bool getSubCountDepth() {
            return enabled_depth && pub_depth && pub_depth->get_subscription_count() > 0;
        }
        bool getSubCountCameraInfo() {
            return enabled_camera_info && pub_camera_info && pub_camera_info->get_subscription_count() > 0;
        }
        bool getSubCountRGBCompressed() {
            return pub_rgb_compressed && pub_rgb_compressed->get_subscription_count() > 0;
        }
        bool getSubCountIRCompressed() {
            return pub_ir_compressed && pub_ir_compressed->get_subscription_count() > 0;
        }
        bool getSubCountRGBCameraInfo() {
            return pub_rgb_camera_info && pub_rgb_camera_info->get_subscription_count() > 0;
        }
        bool getSubCountIRCameraInfo() {
            return pub_ir_camera_info && pub_ir_camera_info->get_subscription_count() > 0;
        }
        bool getSubCountDepthCameraInfo() {
            return pub_depth_camera_info && pub_depth_camera_info->get_subscription_count() > 0;
        }
        bool getSubCountDepthCompressed() {
            return pub_depth_compressed && pub_depth_compressed->get_subscription_count() > 0;
        }

    };

private:
    void publishOnce() {
        if(nullptr == m_ipc) {
            return;
        }
        resetStaticTfMsgs();
        std::string parent_frame_id = "base";

        foreach (auto sensor_, m_sensors) {
            if(sensor_.enabled) {
                //필요 카메라 이미지 주석 해제

                if(sensor_.id == Vision_IPC::Sensors_e::Bottom0){
                    // IR 관련
                    if(sensor_.getSubCountIR()) {
                        publishIR(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountIRCompressed()) {
                        publishIRCompressed(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    // Depth 관련
                    if(sensor_.getSubCountDepth()) {
                        publishDepth(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                    if(sensor_.getSubCountDepthCompressed()) {
                        publishDepthCompressed(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                }

                if(sensor_.id == Vision_IPC::Sensors_e::Bottom1){
                    // IR 관련
                    if(sensor_.getSubCountIR()) {
                        publishIR(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountIRCompressed()) {
                        publishIRCompressed(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    // Depth 관련
                    if(sensor_.getSubCountDepth()) {
                        publishDepth(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                    if(sensor_.getSubCountDepthCompressed()) {
                        publishDepthCompressed(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                }

                if(sensor_.id == Vision_IPC::Sensors_e::Bottom2){
                    // IR 관련
                    if(sensor_.getSubCountIR()) {
                        publishIR(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountIRCompressed()) {
                        publishIRCompressed(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    // Depth 관련
                    if(sensor_.getSubCountDepth()) {
                        publishDepth(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                    if(sensor_.getSubCountDepthCompressed()) {
                        publishDepthCompressed(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                }

                if(sensor_.id == Vision_IPC::Sensors_e::Bottom3){
                    // IR 관련
                    if(sensor_.getSubCountIR()) {
                        publishIR(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountIRCompressed()) {
                        publishIRCompressed(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    // Depth 관련
                    if(sensor_.getSubCountDepth()) {
                        publishDepth(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                }

                if(sensor_.id == Vision_IPC::Sensors_e::Front || sensor_.id == Vision_IPC::Sensors_e::Rear) {
                    // // RGB 관련
                    if(sensor_.getSubCountRgb()) {
                        publishRGB(sensor_.id);
                        publishRGBCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountRGBCompressed()) {
                        publishRGBCompressed(sensor_.id);
                        publishRGBCameraInfo(sensor_.id);
                    }
                    
                    // IR 관련
                    if(sensor_.getSubCountIR()) {
                        publishIR(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountIRCompressed()) {
                        publishIRCompressed(sensor_.id);
                        publishIRCameraInfo(sensor_.id);
                    }

                    // Depth 관련
                    if(sensor_.getSubCountDepth()) {
                        publishDepth(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                    
                    if(sensor_.getSubCountDepthCompressed()) {
                        publishDepthCompressed(sensor_.id);
                        publishDepthCameraInfo(sensor_.id);
                    }
                    
                }
            }
        }
        publishDynamicTransforms();
    }

    void publishDynamicTransforms()
    {
        if (!m_dynamic_tf_broadcaster)
        {
            m_dynamic_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        rclcpp::Time t = this->now();
        try
        {
            for(auto& msg : m_static_tf_msgs)
                msg.header.stamp = t;
            m_dynamic_tf_broadcaster->sendTransform(m_static_tf_msgs);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("Error publishing dynamic transforms: " << e.what());
        }
    }

    void restartStaticTransformBroadcaster()
    {
        if (m_static_tf_broadcaster) { m_static_tf_broadcaster.reset(); }
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
        options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this,  tf2_ros::StaticBroadcasterQoS(), std::move(options));
        resetStaticTfMsgs();
        m_static_tf_broadcaster->sendTransform(m_static_tf_msgs);
    }

    void resetStaticTfMsgs()
    {
        m_static_tf_msgs.clear();
    }

    void appendStaticTfMsg(const float TfMat[12], const std::string& from, const std::string& to)
    {
        /*
        rotation[0], rotation[3], rotation[6], translation[9 ],
        rotation[1], rotation[4], rotation[7], translation[10],
        rotation[2], rotation[5], rotation[8], translation[11],
                  0,           0,           0,               1;
        */
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = from;
        msg.child_frame_id = to;

        msg.transform.translation.x = TfMat[9];
        msg.transform.translation.y = TfMat[10];
        msg.transform.translation.z = TfMat[11];

        Eigen::Matrix3f m;
        // m<< TfMat[0], TfMat[3], TfMat[6],
        //     TfMat[1], TfMat[4], TfMat[7],
        //     TfMat[2], TfMat[5], TfMat[8];
        m<< 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        Eigen::Quaternionf q(m);

        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();

        m_static_tf_msgs.push_back(msg);
    }

    void appendStaticTfMsg(const Eigen::Matrix4d TfMat, const std::string& from, const std::string& to)
    {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = from;
        msg.child_frame_id = to;

        msg.transform.translation.x = TfMat(0,3);
        msg.transform.translation.y = TfMat(1,3);
        msg.transform.translation.z = TfMat(2,3);

        Eigen::Quaterniond q(TfMat.block<3,3>(0,0));

        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();

        m_static_tf_msgs.push_back(msg);
    }

    // RGB 
    bool publishRGB(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                if(sensor_.getSubCountRgb()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_color[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_color_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageColor_t imageColor_;
                        m_ipc->getImageColor(sensor_.id, imageColor_);
                        cv::Mat img_(imageColor_.height, imageColor_.width, CV_8UC3, (void*)(imageColor_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_rgb;
                        sensor_.pub_rgb->publish(*(cv_bridge::CvImage(header, enc::BGR8, img_).toImageMsg()));
                        return true;
                    }
                }
                break;
            }
        }
        return false;
    }
    
    bool publishRGBCompressed(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                if(sensor_.getSubCountRGBCompressed()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_color[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_color_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageColor_t imageColor_;
                        m_ipc->getImageColor(sensor_.id, imageColor_);
                        cv::Mat img_(imageColor_.height, imageColor_.width, CV_8UC3, (void*)(imageColor_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_rgb;
                        sensor_msgs::msg::CompressedImage::UniquePtr compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                        compressed_msg->header = header;
                        compressed_msg->format = "jpeg";
                        // Optimize compression for speed (lower quality = faster compression)
                        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 70};
                        cv::imencode(".jpg", img_, compressed_msg->data, compression_params);
                        sensor_.pub_rgb_compressed->publish(std::move(compressed_msg));
                        return true;
                    }
                }
                break;
            }
        }
        return false;
    }
    
    bool publishRGBCameraInfo(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                // RGB CameraInfo는 RGB 데이터와 독립적으로 발행 (tick 체크 없이)
                Vision_IPC::ImageColor_t imageColor_;
                m_ipc->getImageColor(sensor_.id, imageColor_);
                
                sensor_msgs::msg::CameraInfo::UniquePtr msg_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
                msg_camera_info->header.stamp = this->now();
                msg_camera_info->header.frame_id = sensor_.name_rgb;
                msg_camera_info->width = imageColor_.width;
                msg_camera_info->height = imageColor_.height;

                // Set intrinsic parameters from ImageColor_t
                msg_camera_info->k[0] = imageColor_.intrinsics[0];  // fx
                msg_camera_info->k[2] = imageColor_.intrinsics[2];  // cx
                msg_camera_info->k[4] = imageColor_.intrinsics[1];  // fy
                msg_camera_info->k[5] = imageColor_.intrinsics[3];  // cy
                msg_camera_info->k[8] = 1.0;  // Identity

                // Set distortion coefficients
                msg_camera_info->d = std::vector<double>(8);
                for (int i = 0; i < 8; i++) {
                    msg_camera_info->d[i] = imageColor_.coeffs[i];
                }
                sensor_.pub_rgb_camera_info->publish(std::move(msg_camera_info));
                
                // ROS_INFO("RGB CameraInfo PUBLISHED - Sensor %d: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                //     sensor_.id, imageColor_.intrinsics[0], imageColor_.intrinsics[1], 
                //     imageColor_.intrinsics[2], imageColor_.intrinsics[3]);
                return true;
            }
        }
        return false;
    }
    
    // IR 
    bool publishIR(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                if(sensor_.getSubCountIR()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_ir[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_ir_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageIR_t imageIR_;
                        m_ipc->getImageIR(sensor_.id, imageIR_);
                        cv::Mat img_(imageIR_.height, imageIR_.width, CV_8U, (void*)(imageIR_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_ir;
                        sensor_.pub_ir->publish(*(cv_bridge::CvImage(header, enc::MONO8, img_).toImageMsg()));
                        return true;
                    }
                }
                break;
            }
        }
        return false;
    }
    
    bool publishIRCompressed(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                if(sensor_.getSubCountIRCompressed()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_ir[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_ir_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageIR_t imageIR_;
                        m_ipc->getImageIR(sensor_.id, imageIR_);
                        cv::Mat img_(imageIR_.height, imageIR_.width, CV_8U, (void*)(imageIR_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_ir;
                        sensor_msgs::msg::CompressedImage::UniquePtr compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                        compressed_msg->header = header;
                        compressed_msg->format = "jpeg";
                        // Optimize compression for speed (lower quality = faster compression)
                        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 70};
                        cv::imencode(".jpg", img_, compressed_msg->data, compression_params);
                        sensor_.pub_ir_compressed->publish(std::move(compressed_msg));
                        return true;
                    }
                }
                break;
            }
        }
        return false;
    }
    
    bool publishIRCameraInfo(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                // CameraInfo는 IR 데이터와 독립적으로 발행 (tick 체크 없이)
                Vision_IPC::ImageIR_t imageIR_;
                m_ipc->getImageIR(sensor_.id, imageIR_);
                
                sensor_msgs::msg::CameraInfo::UniquePtr msg_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
                msg_camera_info->header.stamp = this->now();
                msg_camera_info->header.frame_id = sensor_.name_ir;
                msg_camera_info->width = imageIR_.width;
                msg_camera_info->height = imageIR_.height;

                        // Set intrinsic parameters from ImageIR_t
                        msg_camera_info->k[0] = imageIR_.intrinsics[0];  // fx
                        msg_camera_info->k[2] = imageIR_.intrinsics[2];  // cx
                        msg_camera_info->k[4] = imageIR_.intrinsics[1];  // fy
                        msg_camera_info->k[5] = imageIR_.intrinsics[3];  // cy
                        msg_camera_info->k[8] = 1.0;  // Identity

                // Set distortion coefficients
                msg_camera_info->d = std::vector<double>(8);
                for (int i = 0; i < 8; i++) {
                    msg_camera_info->d[i] = imageIR_.coeffs[i];
                }
                sensor_.pub_ir_camera_info->publish(std::move(msg_camera_info));
                
                // ROS_INFO("IR CameraInfo PUBLISHED - Sensor %d: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                //     sensor_.id, imageIR_.intrinsics[0], imageIR_.intrinsics[1], 
                //     imageIR_.intrinsics[2], imageIR_.intrinsics[3]);
                return true;
            }
        }
        return false;
    }

    // Depth 
    bool publishDepth(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                if(sensor_.getSubCountDepth()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_depth[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_depth_tickWrite[sensor_.id]) {
                        Vision_IPC::Depth_t depth_;
                        m_ipc->getDepth(sensor_.id, depth_);
                        
                        // Convert depth data to cv::Mat
                        cv::Mat depth_image(depth_.height, depth_.width, CV_16UC1, (void*)(depth_.data), cv::Mat::AUTO_STEP);
                        
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_depth;
                        
                        sensor_.pub_depth->publish(*(cv_bridge::CvImage(header, enc::TYPE_16UC1, depth_image).toImageMsg()));
                        return true;
                    }
                }
                break;
            }
        }
        return false;
    }
    
    bool publishDepthCompressed(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                if(sensor_.getSubCountDepthCompressed()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_depth[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_depth_tickWrite[sensor_.id]) {
                        Vision_IPC::Depth_t depth_;
                        m_ipc->getDepth(sensor_.id, depth_);
                        
                        // Convert depth data to cv::Mat
                        cv::Mat depth_image(depth_.height, depth_.width, CV_16UC1, (void*)(depth_.data), cv::Mat::AUTO_STEP);
                        
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_depth;
                        
                        sensor_msgs::msg::CompressedImage::UniquePtr compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                        compressed_msg->header = header;

                        compressed_msg->format = "png";  
                        std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 6};  // 0-9, 높을수록 압축률 증가
                        cv::imencode(".png", depth_image, compressed_msg->data, compression_params);
                        
                        sensor_.pub_depth_compressed->publish(std::move(compressed_msg));
                        return true;
                    }
                }
                break;
            }
        }
        return false;
    }
    
    bool publishDepthCameraInfo(Vision_IPC::Sensors_e sensor_id) {
        // 해당 센서 찾기
        for(auto& sensor_ : m_sensors) {
            if(sensor_.id == sensor_id && sensor_.enabled) {
                // Depth CameraInfo는 Depth 데이터와 독립적으로 발행 (tick 체크 없이)
                Vision_IPC::Depth_t depth_;
                m_ipc->getDepth(sensor_.id, depth_);
                
                sensor_msgs::msg::CameraInfo::UniquePtr msg_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
                msg_camera_info->header.stamp = this->now();
                msg_camera_info->header.frame_id = sensor_.name_depth;
                msg_camera_info->width = depth_.width;
                msg_camera_info->height = depth_.height;

                // Set intrinsic parameters from Depth_t
                msg_camera_info->k[0] = depth_.intrinsics[0];  // fx
                msg_camera_info->k[2] = depth_.intrinsics[2];  // cx
                msg_camera_info->k[4] = depth_.intrinsics[1];  // fy
                msg_camera_info->k[5] = depth_.intrinsics[3];  // cy
                msg_camera_info->k[8] = 1.0;  // Identity

                // Set distortion coefficients
                msg_camera_info->d = std::vector<double>(8);
                for (int i = 0; i < 8; i++) {
                    msg_camera_info->d[i] = depth_.coeffs[i];
                }
                sensor_.pub_depth_camera_info->publish(std::move(msg_camera_info));
                
                // ROS_INFO("Depth CameraInfo PUBLISHED - Sensor %d: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                //     sensor_.id, depth_.intrinsics[0], depth_.intrinsics[1], 
                //     depth_.intrinsics[2], depth_.intrinsics[3]);
                return true;
            }
        }
        return false;
    }

    std::chrono::milliseconds m_sleepTime = 50ms;  // 15fps (1000ms / 15 = 66.67ms)
    rclcpp::TimerBase::SharedPtr m_timer;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>    m_static_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster>          m_dynamic_tf_broadcaster;
    std::vector<geometry_msgs::msg::TransformStamped>       m_static_tf_msgs;

    std::vector<VisionSensor_t> m_sensors;

    Vision_IPC* m_ipc = nullptr;

    rclcpp::Logger _logger;
};
