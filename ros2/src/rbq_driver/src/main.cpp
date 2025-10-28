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

#include <unistd.h>
#include <QProcess>
#include <QDebug>
#include <iostream>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static inline int command_terminal_ping_check(const std::string &ip)
{
    int ret=0;
    QProcess *proc;
    proc = new QProcess();
    proc->start("ping", QStringList() << "-c 2" << ip.data());
    proc->waitForFinished(-1);
    QByteArray output = proc->readAllStandardOutput();
    if (!output.isEmpty()){
        if (-1 != QString(output).indexOf("ttl", 0, Qt::CaseInsensitive)){
            std::cout << ip.data() << ", PING OK\n";
            ret = 1;
        }
        else{
            std::cout << ip.data() << ", PING Fail\n";
        }
    }
    delete proc;
    return ret;
}

static inline bool isMotionPC()
{
    struct ifaddrs *ifaddr, *ifa;
    char ip[INET_ADDRSTRLEN];
    bool isMotion = false;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return false;
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
            continue;

        void *addr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
        inet_ntop(AF_INET, addr, ip, INET_ADDRSTRLEN);

        if (strcmp(ip, "192.168.0.10") == 0) {
            isMotion = true;
            break;
        }
    }

    freeifaddrs(ifaddr);
    return isMotion;
}

#include "RobotApiHandler.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "VisionPublisher.h"
#include "VisionSubscriber.h"

bool IS_SIM = false;

void spin(const rclcpp::Node::SharedPtr &_node) {
    rclcpp::spin(_node);
}

bool GLOBAL_KILL_SIGNAL = false;

int main(int argc, char * argv[])
{
    std::string host;
    for (int i = 1; i < argc; ++i) {
        QString arg = argv[i];
        if (arg == "-s") {
            IS_SIM = true;
        }
    }
    if (IS_SIM) {
        host = "127.0.0.1";
        std::cout<<"ROS2 Simulation Mode (127.0.0.1)"<<std::endl;
    } else {
        if (isMotionPC()) {
            host = "192.168.0.10";
            std::cout<<"ROS2 Robot Mode (192.168.0.10)"<<std::endl;
        } else {
            host = "192.168.0.12";
            std::cout<<"ROS2 Robot Mode (192.168.0.12)"<<std::endl;
        }
    }

    bool flag = !IS_SIM;
    while(flag) {
        if(command_terminal_ping_check(host)) {
            flag = false;
        } else {
            qDebug() << "command_terminal_ping_check()" << host.data() << " failed !!! ";
            if(!rclcpp::ok()) {
                return -1;
            }
            sleep(1);
        }
    }

    std::shared_ptr<RobotApiHandler> apiHandler = nullptr;
    std::thread thread_publisher;
    std::thread thread_subscriber;
    std::thread thread_vision_publisher;
    std::thread thread_vision_subscriber;

    rclcpp::init(argc, argv);

    if (IS_SIM || isMotionPC()) {
        std::cout << "Starting RobotApiHandler, Publisher, Subscriber, VisionPublisher" << std::endl;
        apiHandler = std::make_shared<RobotApiHandler>(host, 200);
        rclcpp::Node::SharedPtr publisher = std::make_shared<Publisher>(apiHandler, 5ms);
        thread_publisher = std::thread(spin, publisher);
        rclcpp::Node::SharedPtr subscriber = std::make_shared<Subscriber>(apiHandler);
        thread_subscriber = std::thread(spin, subscriber);
        rclcpp::Node::SharedPtr vision_publisher = std::make_shared<VisionPublisher>(20ms);
        thread_vision_publisher = std::thread(spin, vision_publisher);

        while(rclcpp::ok()) {
            rclcpp::sleep_for(1000ms);
        }
        GLOBAL_KILL_SIGNAL = true;
        rclcpp::shutdown();
        if (thread_publisher.joinable()) thread_publisher.join();
        if (thread_subscriber.joinable()) thread_subscriber.join();
        if (thread_vision_publisher.joinable()) thread_vision_publisher.join();
    } else if (!isMotionPC()) {
        std::cout << "Starting VisionSubscriber only" << std::endl;
        rclcpp::Node::SharedPtr vision_subscriber = std::make_shared<VisionSubscriber>(20ms);
        thread_vision_subscriber = std::thread(spin, vision_subscriber);
        while(rclcpp::ok()) {
            rclcpp::sleep_for(1000ms);
        }
        GLOBAL_KILL_SIGNAL = true;
        rclcpp::shutdown();
        if (thread_vision_subscriber.joinable()) thread_vision_subscriber.join();
    }
 

    return 0;
}
