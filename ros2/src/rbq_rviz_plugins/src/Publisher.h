// Copyright 2026 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/int8.hpp>

#include <rbq_msgs/msg/power_control.hpp>

// Publishes RBQ control commands as ROS 2 topics that map onto the CycloneDDS
// `rt/rbq/cmd/*` wire convention used by Network. Build with
// RMW_IMPLEMENTATION=rmw_cyclonedds_cpp at runtime so publications land on the
// expected DDS topics.
//
// Note: canCheck and findHome no longer have a counterpart on Network; the
// equivalent steps are run by autoStart. The corresponding methods are kept as
// no-ops so existing button slots still link.
class Publisher : public rclcpp::Node
{
public:
    explicit Publisher(const std::string &nodeName) : Node(nodeName)
    {
        m_pub_autoStart         = create_publisher<std_msgs::msg::Bool>("rbq/cmd/autoStart",                 10);
        m_pub_emergency         = create_publisher<std_msgs::msg::Bool>("rbq/cmd/emergency",                 10);
        m_pub_dock              = create_publisher<std_msgs::msg::Bool>("rbq/cmd/docking",                   10);
        m_pub_switchControlMode = create_publisher<std_msgs::msg::Bool>("rbq/cmd/switchControlMode",         10);
        m_pub_switchGait        = create_publisher<std_msgs::msg::Int8>("rbq/cmd/switchGait",                10);
        m_pub_setBodyHeight     = create_publisher<std_msgs::msg::Char>("rbq/cmd/bodyHeightPerc",            10);
        m_pub_setFootHeight     = create_publisher<std_msgs::msg::Char>("rbq/cmd/footHeightPerc",            10);
        m_pub_setMaxSpeed       = create_publisher<std_msgs::msg::Char>("rbq/cmd/maxSpeedPerc",              10);
        m_pub_comEstimation     = create_publisher<std_msgs::msg::Char>("rbq/cmd/comEstimationCompensation", 10);
        m_pub_setPortState      = create_publisher<rbq_msgs::msg::PowerControl>("rbq/cmd/setPortState",      10);
    }

    void pub_autoStart()                                          { publishBool(m_pub_autoStart,         true); }
    void pub_emergency()                                          { publishBool(m_pub_emergency,         true); }
    void pub_dock()                                               { publishBool(m_pub_dock,              true); }
    void pub_switchControlMode(const bool &extJoy)                { publishBool(m_pub_switchControlMode, extJoy); }
    void pub_switchGait(const int &gait_id)
    {
        std_msgs::msg::Int8 msg; msg.data = gait_id; m_pub_switchGait->publish(msg);
    }
    void pub_setBodyHeight(const int &val) { publishChar(m_pub_setBodyHeight, val); }
    void pub_setFootHeight(const int &val) { publishChar(m_pub_setFootHeight, val); }
    void pub_setMaxSpeed(const int &val)   { publishChar(m_pub_setMaxSpeed,   val); }
    void pub_comEstimation(const int &val) { publishChar(m_pub_comEstimation, val); }
    void pub_setPortState(const int &port_id, const bool &on)
    {
        rbq_msgs::msg::PowerControl msg;
        msg.port_id = static_cast<uint8_t>(port_id);
        msg.state   = on;
        m_pub_setPortState->publish(msg);
    }

    // Deprecated — kept as no-ops so panel buttons still link.
    // autoStart triggers the same init sequence on the robot.
    void pub_canCheck() {}
    void pub_findHome() {}

private:
    static void publishBool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &p, bool v)
    {
        std_msgs::msg::Bool msg; msg.data = v; p->publish(msg);
    }
    static void publishChar(const rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr &p, int v)
    {
        std_msgs::msg::Char msg; msg.data = v; p->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         m_pub_autoStart;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         m_pub_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         m_pub_dock;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr         m_pub_switchControlMode;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr         m_pub_switchGait;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr         m_pub_setBodyHeight;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr         m_pub_setFootHeight;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr         m_pub_setMaxSpeed;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr         m_pub_comEstimation;
    rclcpp::Publisher<rbq_msgs::msg::PowerControl>::SharedPtr m_pub_setPortState;
};
