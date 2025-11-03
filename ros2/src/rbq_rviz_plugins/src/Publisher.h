// Copyright 2023 Rainbow Robotics Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    Publisher(const std::string &nodeName) : Node("rbq_rviz_panel")
    {
        std::cout << nodeName << std::endl;

        m_pub_autoStart            = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/autoStart",            10);
        m_pub_canCheck             = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/canCheck",             10);
        m_pub_findHome             = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/findHome",             10);
        m_pub_sit                  = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/sit",                  10);
        m_pub_stand                = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/stand",                10);
        m_pub_walk                 = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/walk",                 10);
        m_pub_walkSlow             = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/walkSlow",             10);
        m_pub_run                  = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/run",                  10);
        m_pub_switchGait           = this->create_publisher<std_msgs::msg::Int8>("rbq/motion/switchGait",           10);
        m_pub_setPortState         = this->create_publisher<std_msgs::msg::Int8MultiArray>("rbq/powerControl/setPortState", 10);
        m_pub_switchControlMode         = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/switchControlMode",        10);
        m_pub_dock                 = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/docking",              10);
        m_pub_emergency            = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/emergency",            10);
        m_pub_powerLeg             = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Leg",            10);
        m_pub_powerArm             = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Arm",            10);
        m_pub_powerVisionPC        = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/VisionPC",       10);
        m_pub_powerUsbHub          = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/UsbHub",         10);
        m_pub_powerCctv            = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Cctv",           10);
        m_pub_powerThermal         = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Thermal",        10);
        m_pub_powerLidar           = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Lidar",          10);
        m_pub_powerExt52V          = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Ext52V",         10);
        m_pub_powerIrLEDs          = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/IrLEDs",         10);
        m_pub_powerComm            = this->create_publisher<std_msgs::msg::Bool>("rbq/powerControl/Comm",           10);
        m_pub_setBodyHeight        = this->create_publisher<std_msgs::msg::Char>("rbq/motion/setBodyHeight",        10);
        m_pub_setFootHeight        = this->create_publisher<std_msgs::msg::Char>("rbq/motion/setFootHeight",        10);
        m_pub_setMaxSpeed          = this->create_publisher<std_msgs::msg::Char>("rbq/motion/setMaxSpeed",          10);
        m_pub_comEstimation        = this->create_publisher<std_msgs::msg::Char>("rbq/stateEstimation/comEstimation",        10);
        
        m_pub_stairs               = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/stairs",               10);
        m_pub_rlTrot               = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_trot",              10);
        m_pub_rlBound              = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_bound",             10);
        m_pub_rlPace               = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_pace",              10);
        m_pub_rlPronk              = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_pronk",             10);
        m_pub_rlTrotVision         = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_trot_vision",       10);
        m_pub_rlTrotRun            = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_trot_run",          10);
        m_pub_rlSilent             = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_silent",            10);
        m_pub_rlFrontWalk          = this->create_publisher<std_msgs::msg::Bool>("rbq/motion/rl_front_walk",        10);
    }

    void pub_autoStart()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_autoStart->publish(msg);
    }

    void pub_stand()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_stand->publish(msg);
    }

    void pub_sit()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_sit->publish(msg);
    }

    void pub_canCheck()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_canCheck->publish(msg);
    }

    void pub_findHome()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_findHome->publish(msg);
    }

    void pub_walk()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_walk->publish(msg);
    }

    void pub_walkSlow()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_walkSlow->publish(msg);
    }

    void pub_run()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_run->publish(msg);
    }

    void pub_switchGait(const int &gait_id)
    {
        std_msgs::msg::Int8 msg = std_msgs::msg::Int8();
        msg.data = gait_id;
        m_pub_switchGait->publish(msg);
    }

    void pub_setPortState(const int &port_id, const bool &powerON)
    {
        std_msgs::msg::Int8MultiArray msg = std_msgs::msg::Int8MultiArray();
        msg.data.resize(2);
        msg.data[0] = port_id;
        msg.data[1] = powerON ? 1 : 0;
        m_pub_setPortState->publish(msg);
    }

    void pub_switchControlMode(const bool &extJoy)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = extJoy;
        m_pub_switchControlMode->publish(msg);
    }

    void pub_dock()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_dock->publish(msg);
    }


    void pub_emergency()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_emergency->publish(msg);
    }

    void pub_powerLeg(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerLeg->publish(msg);
    }

    void pub_powerArm(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerArm->publish(msg);
    }

    void pub_powerVisionPC(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerVisionPC->publish(msg);
    }

    void pub_powerUsbHub(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerUsbHub->publish(msg);
    }

    void pub_powerCctv(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerCctv->publish(msg);
    }

    void pub_powerThermal(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerThermal->publish(msg);
    }

    void pub_powerLidar(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerLidar->publish(msg);
    }

    void pub_powerExt52V(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerExt52V->publish(msg);
    }

    void pub_powerIrLEDs(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerIrLEDs->publish(msg);
    }

    void pub_powerComm(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerComm->publish(msg);
    }

    void pub_setBodyHeight(const int &addedBodyHeight)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = addedBodyHeight;
        m_pub_setBodyHeight->publish(msg);
    }

    void pub_setFootHeight(const int &addedFootHeight)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = addedFootHeight;
        m_pub_setFootHeight->publish(msg);
    }

    void pub_setMaxSpeed(const int &maxSpeedPercent)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = maxSpeedPercent;
        m_pub_setMaxSpeed->publish(msg);
    }

    void pub_comEstimation(const int &calibrationStage)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = calibrationStage;
        m_pub_comEstimation->publish(msg);
    }

    // RL 관련 함수들 구현
    void pub_stairs()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_stairs->publish(msg);
    }

    void pub_rlTrot()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlTrot->publish(msg);
    }

    void pub_rlBound()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlBound->publish(msg);
    }

    void pub_rlPace()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlPace->publish(msg);
    }

    void pub_rlPronk()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlPronk->publish(msg);
    }

    void pub_rlTrotVision()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlTrotVision->publish(msg);
    }

    void pub_rlTrotRun()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlTrotRun->publish(msg);
    }

    void pub_rlSilent()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlSilent->publish(msg);
    }

    void pub_rlFrontWalk()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_rlFrontWalk->publish(msg);
    }


private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_autoStart           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_stand               ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_sit                 ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_canCheck            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_findHome            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_walk                ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_walkSlow            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_run                 ;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_pub_switchGait          ;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr m_pub_setPortState ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_switchControlMode        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_dock                ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_emergency           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerLeg            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerArm            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerVisionPC       ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerUsbHub         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerCctv           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerThermal        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerLidar          ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerExt52V         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerIrLEDs         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerComm           ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_setBodyHeight       ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_setFootHeight       ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_setMaxSpeed         ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_comEstimation       ;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_stairs              ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlTrot              ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlBound             ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlPace              ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlPronk             ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlTrotVision        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlTrotRun           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlSilent            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_rlFrontWalk         ;

};

