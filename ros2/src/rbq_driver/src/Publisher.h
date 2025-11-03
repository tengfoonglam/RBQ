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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/velocity_stamped.hpp>

#include <rbq_msgs/msg/foot_states.hpp>
#include <rbq_msgs/msg/robot_status.hpp>
#include <rbq_msgs/msg/joint_status.hpp>
#include <rbq_msgs/msg/battery_state.hpp>

#include "RobotApiHandler.h"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    Publisher(const std::shared_ptr<RobotApiHandler> &robotApiHandler,
              const std::chrono::milliseconds &_m_sleepTime = 20ms)
        : Node("rbq_state_publisher")
        , m_sleepTime(_m_sleepTime)
        , m_robotApiHandler(robotApiHandler)
    {
        std::string motion_prefix = "rbq/motion/";


        std::string joint_states_prefix = "rbq/joint/";
        m_publisher_jointStates = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_prefix + "joint_states", 10);
        m_timer_jointStates = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishJointStates, this));
        m_publisher_jointStatus = this->create_publisher<rbq_msgs::msg::JointStatus>(joint_states_prefix + "joint_status", 10);
        m_timer_jointStatus = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishJointStatus, this));

        std::string status_prefix = "rbq/status/";
        m_publisher_status = this->create_publisher<rbq_msgs::msg::RobotStatus>(status_prefix + "robot_status", 10);
        m_timer_status = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishRobotStatus, this));

        m_comm_connected = this->create_publisher<std_msgs::msg::Bool>(status_prefix + "comm_connected", 10);
        m_timer_comm_connected = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishCommConnected, this));

        std::string imu_state_prefix = "rbq/imu/";
        m_publisher_IMUState = this->create_publisher<sensor_msgs::msg::Imu>(imu_state_prefix + "IMU_state", 10);
        m_timer_IMUState = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishIMUState, this));

        std::string powercontrol_prefix = "rbq/powerControl/";
        m_publisher_batteryStatus = this->create_publisher<rbq_msgs::msg::BatteryState>(powercontrol_prefix + "battery_status", 10);
        m_timer_batteryStatus = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishBatteryStatus, this));

        std::string stateEstimation_prefix = "rbq/stateEstimation/";
        m_publisher_odometry = this->create_publisher<nav_msgs::msg::Odometry>(stateEstimation_prefix + "odometry", 10);
        m_timer_odometry = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishOdometry, this));
        m_publisher_robotVelocity = this->create_publisher<geometry_msgs::msg::VelocityStamped>(stateEstimation_prefix + "robotVelocity", 10);
        m_timer_robotVelocity = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishRobotVelocity, this));
        m_publisher_footStates = this->create_publisher<rbq_msgs::msg::FootStates>(stateEstimation_prefix + "footStates", 10);
        m_timer_footStates = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishFootStates, this));
        
    }

private:
    void publishRobotStatus()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            // RCLCPP_ERROR(this->get_logger(), "Shared Data is set to nullptr.");
            return;
        }
        rbq_msgs::msg::RobotStatus status_msg = rbq_msgs::msg::RobotStatus();
        {
            status_msg.con_start = robotState->robotStatus.CON_START;
            status_msg.ready_pos = robotState->robotStatus.READY_POS;
            status_msg.ground_pos = robotState->robotStatus.GROUND_POS;
            status_msg.force_con = robotState->robotStatus.FORCE_CON;
            status_msg.ext_joy = robotState->robotStatus.EXT_JOY;
            status_msg.is_standing = robotState->robotStatus.IS_STANDING;
            status_msg.can_check = robotState->robotStatus.CAN_CHECK;
            status_msg.find_home = robotState->robotStatus.FIND_HOME;
            status_msg.gait_id = robotState->robotStatus.GAIT_ID;
            status_msg.is_fall = robotState->robotStatus.IS_FALL;
            status_msg.docking_state = robotState->robotStatus.DOCKING_STAT;
            status_msg.imu_success = robotState->robotStatus.IMU_SUCCESS;
        }
        m_publisher_status->publish(status_msg);
    }
    
    void publishJointStates()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            // RCLCPP_ERROR(this->get_logger(), "Shared Data is set to nullptr.");
            return;
        }
        sensor_msgs::msg::JointState jointState_msg = sensor_msgs::msg::JointState();
        jointState_msg.header.stamp = this->get_clock()->now();
        for(int i=0; i<12; i++)
        {
            jointState_msg.name.push_back(m_jointNames.at(i));
            jointState_msg.position.push_back(  robotState->jointStates.at(i).position);
            jointState_msg.effort.push_back(    robotState->jointStates.at(i).torque);
        }

        m_publisher_jointStates->publish(jointState_msg);
    }

    void publishJointStatus()
    {
        RBQ_SDK::Motion::JointStatus_t* jointStatus = m_robotApiHandler->jointStatus();
        if(jointStatus == nullptr) {
            // RCLCPP_WARN(this->get_logger(), "JointStatus is nullptr");
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Publishing JointStatus data");
        
        rbq_msgs::msg::JointStatus jointStatus_msg;
        jointStatus_msg.header.stamp = this->get_clock()->now();
        jointStatus_msg.header.frame_id = "base_link";
        
        for(int i = 0; i < 16; i++) {
            jointStatus_msg.connected.push_back(jointStatus->joint_details[i].connected);
            jointStatus_msg.temperature.push_back(jointStatus->joint_details[i].temperature);
            jointStatus_msg.motor_temp.push_back(jointStatus->joint_details[i].motor_temp);
            
            jointStatus_msg.status_fet.push_back(jointStatus->joint_details[i].status.FET);
            jointStatus_msg.status_run.push_back(jointStatus->joint_details[i].status.RUN);
            jointStatus_msg.status_init.push_back(jointStatus->joint_details[i].status.INIT);
            jointStatus_msg.status_mod.push_back(jointStatus->joint_details[i].status.MOD);
            jointStatus_msg.status_non_ctr.push_back(jointStatus->joint_details[i].status.NON_CTR);
            jointStatus_msg.status_bat.push_back(jointStatus->joint_details[i].status.BAT);
            jointStatus_msg.status_calib.push_back(jointStatus->joint_details[i].status.CALIB);
            jointStatus_msg.status_mt_err.push_back(jointStatus->joint_details[i].status.MT_ERR);
            jointStatus_msg.status_jam.push_back(jointStatus->joint_details[i].status.JAM);
            jointStatus_msg.status_cur.push_back(jointStatus->joint_details[i].status.CUR);
            jointStatus_msg.status_big.push_back(jointStatus->joint_details[i].status.BIG);
            jointStatus_msg.status_inp.push_back(jointStatus->joint_details[i].status.INP);
            jointStatus_msg.status_flt.push_back(jointStatus->joint_details[i].status.FLT);
            jointStatus_msg.status_tmp.push_back(jointStatus->joint_details[i].status.TMP);
            jointStatus_msg.status_ps1.push_back(jointStatus->joint_details[i].status.PS1);
            jointStatus_msg.status_ps2.push_back(jointStatus->joint_details[i].status.PS2);
            jointStatus_msg.status_rsvd.push_back(jointStatus->joint_details[i].status.rsvd);
            
            jointStatus_msg.position_ref.push_back(jointStatus->joint_details[i].position_ref);
            jointStatus_msg.position_enc.push_back(jointStatus->joint_details[i].position_enc);
            jointStatus_msg.velocity.push_back(jointStatus->joint_details[i].velocity);
            jointStatus_msg.torque_ref.push_back(jointStatus->joint_details[i].torque_ref);
            jointStatus_msg.current.push_back(jointStatus->joint_details[i].current);
            
            jointStatus_msg.kp.push_back(jointStatus->joint_details[i].kp);
            jointStatus_msg.kd.push_back(jointStatus->joint_details[i].kd);
            jointStatus_msg.owner.push_back(jointStatus->joint_details[i].owner);
        }
        
        m_publisher_jointStatus->publish(jointStatus_msg);
    }

    void publishIMUState()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            // RCLCPP_ERROR(this->get_logger(), "Shared Data is set to nullptr.");
            return;
        }

        sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();

        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "IMU";
        imu_msg.orientation.w = robotState->imuState.quaternion.at(0);
        imu_msg.orientation.x = robotState->imuState.quaternion.at(1);
        imu_msg.orientation.y = robotState->imuState.quaternion.at(2);
        imu_msg.orientation.z = robotState->imuState.quaternion.at(3);
        imu_msg.angular_velocity.x = robotState->imuState.gyroscope.at(0);
        imu_msg.angular_velocity.y = robotState->imuState.gyroscope.at(1);
        imu_msg.angular_velocity.z = robotState->imuState.gyroscope.at(2);
        imu_msg.linear_acceleration.x = robotState->imuState.accelerometer.at(0);
        imu_msg.linear_acceleration.y = robotState->imuState.accelerometer.at(1);
        imu_msg.linear_acceleration.z = robotState->imuState.accelerometer.at(2);
        
        m_publisher_IMUState -> publish(imu_msg);

    }
    std::chrono::milliseconds m_sleepTime = 100ms;

    rclcpp::TimerBase::SharedPtr m_timer_odometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher_odometry;
    rclcpp::TimerBase::SharedPtr m_timer_robotVelocity;
    rclcpp::Publisher<geometry_msgs::msg::VelocityStamped>::SharedPtr m_publisher_robotVelocity;

    rclcpp::TimerBase::SharedPtr m_timer_jointStates;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher_jointStates;
    rclcpp::TimerBase::SharedPtr m_timer_jointStatus;
    rclcpp::Publisher<rbq_msgs::msg::JointStatus>::SharedPtr m_publisher_jointStatus;

    rclcpp::TimerBase::SharedPtr m_timer_IMUState;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher_IMUState;

    rclcpp::TimerBase::SharedPtr m_timer_footStates;
    rclcpp::Publisher<rbq_msgs::msg::FootStates>::SharedPtr m_publisher_footStates;
    rclcpp::TimerBase::SharedPtr m_timer_status;
    rclcpp::Publisher<rbq_msgs::msg::RobotStatus>::SharedPtr m_publisher_status;
    rclcpp::TimerBase::SharedPtr m_timer_batteryStatus;
    rclcpp::Publisher<rbq_msgs::msg::BatteryState>::SharedPtr m_publisher_batteryStatus;
    rclcpp::TimerBase::SharedPtr m_timer_comm_connected;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_comm_connected;


    std::vector<std::string> m_jointNames{"joint0_HRR", "joint1_HRP", "joint2_HRK",
                                          "joint3_HLR", "joint4_HLP", "joint5_HLK",
                                          "joint6_FRR", "joint7_FRP", "joint8_FRK",
                                          "joint9_FLR", "joint10_FLP", "joint11_FLK"};

    std::shared_ptr<RobotApiHandler> m_robotApiHandler = nullptr;
    
    void publishOdometry()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            return;
        }
        
        nav_msgs::msg::Odometry odometry_msg = nav_msgs::msg::Odometry();
        odometry_msg.header.stamp = this->get_clock()->now();
        odometry_msg.header.frame_id = "local_world";
        odometry_msg.child_frame_id = "body";
        odometry_msg.pose.pose.position.x = robotState->odometries.odomWrtWorld.pose.position.at(0);
        odometry_msg.pose.pose.position.y = robotState->odometries.odomWrtWorld.pose.position.at(1);
        odometry_msg.pose.pose.position.z = robotState->odometries.odomWrtWorld.pose.position.at(2);
        odometry_msg.pose.pose.orientation.x = robotState->odometries.odomWrtWorld.pose.quaternion.at(1);
        odometry_msg.pose.pose.orientation.y = robotState->odometries.odomWrtWorld.pose.quaternion.at(2);
        odometry_msg.pose.pose.orientation.z = robotState->odometries.odomWrtWorld.pose.quaternion.at(3);
        odometry_msg.pose.pose.orientation.w = robotState->odometries.odomWrtWorld.pose.quaternion.at(0);

        m_publisher_odometry->publish(odometry_msg);
    }

    void publishRobotVelocity()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            return;
        }
        
        geometry_msgs::msg::VelocityStamped body_velocity_msg = geometry_msgs::msg::VelocityStamped();
        body_velocity_msg.header.stamp = this->get_clock()->now();
        body_velocity_msg.header.frame_id = "local_world";
        body_velocity_msg.body_frame_id = "body";
        body_velocity_msg.velocity.linear.x = robotState->odometries.odomWrtWorld.velocity.linear.at(0);
        body_velocity_msg.velocity.linear.y = robotState->odometries.odomWrtWorld.velocity.linear.at(1);
        body_velocity_msg.velocity.linear.z = robotState->odometries.odomWrtWorld.velocity.linear.at(2);
        body_velocity_msg.velocity.angular.x = robotState->odometries.odomWrtWorld.velocity.angular.at(0);
        body_velocity_msg.velocity.angular.y = robotState->odometries.odomWrtWorld.velocity.angular.at(1);
        body_velocity_msg.velocity.angular.z = robotState->odometries.odomWrtWorld.velocity.angular.at(2);
        m_publisher_robotVelocity->publish(body_velocity_msg);
    }

    void publishBatteryStatus()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            return;
        }
        
        rbq_msgs::msg::BatteryState battery_msg;
        battery_msg.header.stamp = this->get_clock()->now();
        battery_msg.header.frame_id = "battery";
        
        battery_msg.identifier = "battery_0";
        battery_msg.charge_percentage = robotState->batteryState.percentage.at(0);
        battery_msg.current = robotState->batteryState.current.at(0);
        battery_msg.voltage = robotState->batteryState.voltage.at(0);
        battery_msg.temperatures.push_back(robotState->batteryState.temperature.at(0));
        
        if(robotState->batteryState.charging.at(0)) {
            battery_msg.status = rbq_msgs::msg::BatteryState::STATUS_CHARGING;
        } else {
            battery_msg.status = rbq_msgs::msg::BatteryState::STATUS_DISCHARGING;
        }
        
        m_publisher_batteryStatus->publish(battery_msg);
    }

    void publishCommConnected()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            return;
        }
        
        std_msgs::msg::Bool comm_msg;
        comm_msg.data = robotState->accepted && (robotState->senderIp4Addr != 0);
        m_comm_connected->publish(comm_msg);
    }

     void publishFootStates()
     {
         RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
         if(robotState == nullptr) {
             return;
         }
         
         rbq_msgs::msg::FootStates footStates_msg;
         footStates_msg.header.stamp = this->get_clock()->now();
         footStates_msg.header.frame_id = "body";
         
         RBQ_SDK::Motion::LegStateArray_t* legStateArray_ = m_robotApiHandler->legStateArray();
         if(legStateArray_ != nullptr) {
             for(int i = 0; i < 4; i++) {
                 geometry_msgs::msg::Point pos, vel;
                 pos.x = legStateArray_->leg_states[i].foot_position_rt_body.at(0);
                 pos.y = legStateArray_->leg_states[i].foot_position_rt_body.at(1);
                 pos.z = legStateArray_->leg_states[i].foot_position_rt_body.at(2);
                 vel.x = legStateArray_->leg_states[i].foot_velocity_rt_body.at(0);
                 vel.y = legStateArray_->leg_states[i].foot_velocity_rt_body.at(1);
                 vel.z = legStateArray_->leg_states[i].foot_velocity_rt_body.at(2);
                 
                 footStates_msg.foot_position_rt_body.push_back(pos);
                 footStates_msg.foot_velocity_rt_body.push_back(vel);
                 footStates_msg.contact.push_back(legStateArray_->leg_states[i].contact);
             }
         }
         m_publisher_footStates->publish(footStates_msg);
     }
};