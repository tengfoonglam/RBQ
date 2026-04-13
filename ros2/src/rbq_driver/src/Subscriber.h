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

#include <chrono>
class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
static Timer cmdJoyEnabledTimer;
static Timer cmdHighLevelTimer;

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rbq_msgs/msg/high_level_command.hpp>

#include "RobotApiHandler.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber(const std::shared_ptr<RobotApiHandler> &robotApiHandler)
        : Node("rbq_cmd_subscriber")
        , m_robotApiHandler(robotApiHandler) 
    {

        m_sub_switchControlMode = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/switchControlMode", 10, std::bind(&Subscriber::callback_switchControlMode, this, _1));

        m_sub_cmd_highLevel = this->create_subscription<rbq_msgs::msg::HighLevelCommand>(
            "rbq/motion/cmd_highLevel", 10, std::bind(&Subscriber::callback_cmd_highLevel, this, _1));

        m_sub_cmd_joystick = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&Subscriber::callback_cmd_joystick, this, _1));

        m_sub_cmd_navigateTo = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "rbq/motion/cmd_navigateTo", 10, std::bind(&Subscriber::callback_cmd_navigateTo, this, _1));

        m_sub_autoStart = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/autoStart", 10, std::bind(&Subscriber::callback_autoStart, this, _1));

        m_sub_canCheck = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/canCheck", 10, std::bind(&Subscriber::callback_canCheck, this, _1));

        m_sub_findHome = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/findHome", 10, std::bind(&Subscriber::callback_findHome, this, _1));

        m_sub_switchGait = this->create_subscription<std_msgs::msg::Int8>(
            "rbq/motion/switchGait", 10, std::bind(&Subscriber::callback_switchGait, this, _1));

        m_sub_staticLock = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/staticLock", 10, std::bind(&Subscriber::callback_staticLock, this, _1));

        m_sub_staticReady = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/staticReady", 10, std::bind(&Subscriber::callback_staticReady, this, _1));

        m_sub_staticGround = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/staticGround", 10, std::bind(&Subscriber::callback_staticGround, this, _1));

        m_sub_recoveryErrorClear = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/recoveryErrorClear", 10, std::bind(&Subscriber::callback_recoveryErrorClear, this, _1));

        m_sub_recoveryFlex = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/recoveryFlex", 10, std::bind(&Subscriber::callback_recoveryFlex, this, _1));

        m_sub_emergency = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/emergency", 10, std::bind(&Subscriber::callback_emergency, this, _1));

        m_sub_setPortState = this->create_subscription<std_msgs::msg::Int8MultiArray>(
            "rbq/powerControl/setPortState", 10, std::bind(&Subscriber::callback_setPortState, this, _1));

        m_sub_comEstimationCompensation = this->create_subscription<std_msgs::msg::Char>(
            "rbq/stateEstimation/comEstimationCompensation", 10, std::bind(&Subscriber::callback_comEstimationCompensation, this, _1));

        m_sub_setBodyHeight = this->create_subscription<std_msgs::msg::Char>(
            "rbq/motion/setBodyHeight", 10, std::bind(&Subscriber::callback_setBodyHeight, this, _1));

        m_sub_setFootHeight = this->create_subscription<std_msgs::msg::Char>(
            "rbq/motion/setFootHeight", 10, std::bind(&Subscriber::callback_setFootHeight, this, _1));

        m_sub_setMaxSpeed = this->create_subscription<std_msgs::msg::Char>(
            "rbq/motion/setMaxSpeed", 10, std::bind(&Subscriber::callback_setMaxSpeed, this, _1));

        m_sub_setPanTiltZoom = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "rbq/ptzCamera/setPanTiltZoom", 10, std::bind(&Subscriber::callback_setPanTiltZoom, this, _1));
        
        m_sub_docking = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion/docking", 10, std::bind(&Subscriber::callback_docking, this, _1));
            
    }

private:
    rclcpp::Subscription<rbq_msgs::msg::HighLevelCommand>::SharedPtr m_sub_cmd_highLevel;
    void callback_cmd_highLevel(const rbq_msgs::msg::HighLevelCommand::SharedPtr _highLevel) const
    {
        if(cmdHighLevelTimer.elapsed() < 0.02f) {
            return;
        }

        RBQ_SDK::HighLevelCmd_t cmd_;

        cmd_.roll           = _highLevel->roll;
        cmd_.pitch          = _highLevel->pitch;
        cmd_.yaw            = _highLevel->yaw;
        cmd_.vel_x          = _highLevel->vel_x;
        cmd_.vel_y          = _highLevel->vel_y;
        cmd_.omega_z        = _highLevel->omega_z;
        cmd_.delta_body_h   = _highLevel->delta_body_h;
        cmd_.delta_foot_h   = _highLevel->delta_foot_h;
        cmd_.gaitID         = _highLevel->gait_state;
        cmd_.gaitTransition = _highLevel->gait_transition;

        if(m_robotApiHandler != nullptr) {
            m_robotApiHandler->setHighLevelCommand(cmd_);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_cmd_navigateTo;
    void callback_cmd_navigateTo(const geometry_msgs::msg::PoseStamped::SharedPtr _pose) const
    {
        m_robotApiHandler->motionDynamicNavigateTo(_pose->pose.position.x, _pose->pose.position.y, _pose->pose.orientation.z);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_autoStart;
    void callback_autoStart(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->autoStart();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_canCheck;
    void callback_canCheck(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->canCheck();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_findHome;
    void callback_findHome(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->findHome();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_switchControlMode;
    void callback_switchControlMode(const std_msgs::msg::Bool::SharedPtr _highLevel) const {
        m_robotApiHandler->switchControlMode(_highLevel.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_staticLock;
    void callback_staticLock(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionStaticLock();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_staticReady;
    void callback_staticReady(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionStaticReady();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_staticGround;
    void callback_staticGround(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionStaticGround();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_recoveryErrorClear;
    void callback_recoveryErrorClear(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->recovery1();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_recoveryFlex;
    void callback_recoveryFlex(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->recovery2();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_emergency;
    void callback_emergency(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->eStop();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr m_sub_setPortState;
    void callback_setPortState(const std_msgs::msg::Int8MultiArray::SharedPtr _data) const {
        if(_data->data.size() >= 2) {
            PDU_PORT_IDs_e pdu_port_id = static_cast<PDU_PORT_IDs_e>(_data->data[0]);
            bool status = static_cast<bool>(_data->data[1]);
            m_robotApiHandler->powerControl(pdu_port_id, status);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_comEstimationCompensation;
    void callback_comEstimationCompensation(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->comEstimationCompensation(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_setBodyHeight;
    void callback_setBodyHeight(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->setBodyHeight(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_setFootHeight;
    void callback_setFootHeight(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->setFootHeight(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_setMaxSpeed;
    void callback_setMaxSpeed(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->setMaxSpeed(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_sub_setPanTiltZoom;
    void callback_setPanTiltZoom(const std_msgs::msg::Float32MultiArray::SharedPtr _ptz) const {
        if(_ptz.get()->data.size() >= 3) {
            m_robotApiHandler->setPanTiltZoom(_ptz.get()->data[0], _ptz.get()->data[1], _ptz.get()->data[2]);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_docking;
    void callback_docking(const std_msgs::msg::Bool::SharedPtr _docking) const {
        if (_docking->data) {
            m_robotApiHandler->docking();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_sub_switchGait;
    void callback_switchGait(const std_msgs::msg::Int8::SharedPtr gait_id) const {
        if(m_robotApiHandler != nullptr) {
            m_robotApiHandler->switchGait(gait_id.get()->data);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub_cmd_joystick;
    void callback_cmd_joystick(const sensor_msgs::msg::Joy::SharedPtr joy) const
    {
        if(m_robotApiHandler != nullptr) {
            m_robotApiHandler->setJoystickCommand(joy);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_sit;
    void callback_sit(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicGround();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_stand;
    void callback_stand(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicReady();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_walk;
    void callback_walk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicWalk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_walkSlow;
    void callback_walkSlow(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicWalkSlow();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_run;
    void callback_run(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicRun();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_stairs;
    void callback_stairs(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicWalkStairs();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_trot;
    void callback_rl_trot(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLTrot();
        }
    }

    // RL 관련 콜백 함수들
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_bound;
    void callback_rl_bound(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLBound();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_pace;
    void callback_rl_pace(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLPace();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_pronk;
    void callback_rl_pronk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLPronk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_3leg_hr;
    void callback_rl_3leg_hr(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRL3LegHR();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_3leg_hl;
    void callback_rl_3leg_hl(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRL3LegHL();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_3leg_fr;
    void callback_rl_3leg_fr(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRL3LegFR();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_3leg_fl;
    void callback_rl_3leg_fl(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRL3LegFL();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_trot_vision;
    void callback_rl_trot_vision(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLTrotVision();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_trot_run;
    void callback_rl_trot_run(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLTrotRun();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_silent;
    void callback_rl_silent(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLSilent();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_front_walk;
    void callback_rl_front_walk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLFrontWalk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_hind_walk;
    void callback_rl_hind_walk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLHindWalk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_left_walk;
    void callback_rl_left_walk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLLeftWalk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_right_walk;
    void callback_rl_right_walk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLRightWalk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_rl_stairs;
    void callback_rl_stairs(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionRLStairs();
        }
    }
    std::shared_ptr<RobotApiHandler> m_robotApiHandler = nullptr;


};