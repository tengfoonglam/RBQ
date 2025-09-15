#ifndef POLICYRUNNER_H
#define POLICYRUNNER_H

#include <iostream>
#include <vector>
#include <array>
#include <memory>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <algorithm> 
#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>

#define NUM_JOINTS 12

class Utils {
public:
    Eigen::Vector3f quat_rotate_inverse(const Eigen::Quaternionf& quat, const Eigen::Vector3f& vec) {
        return quat.inverse() * vec;
    }
};

struct PolicyParams {
    std::string path;
    float action_scale;
    float policy_dt;

    std::array<float, 12> stiffness;
    std::array<float, 12> damping;
    std::array<std::array<float, 2>, 3> command_ranges;
    std::array<float, 12> default_joint_angles;
    float obs_lin_vel_scale;
    float obs_ang_vel_scale;
    float obs_dof_pos_scale;
    float obs_dof_vel_scale;
    float clip_observations;
    float clip_actions;

    int num_observations;
    int num_actions;

    PolicyParams() {
        std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe").parent_path();
        path = (exe_path / "../Simple-RL/policy.onnx").lexically_normal().string();
        action_scale = 0.2;
        policy_dt = 0.02;

        stiffness = {80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0};
        damping = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

        command_ranges[0] = {-1.0, 1.0};
        command_ranges[1] = {-1.0, 1.0};
        command_ranges[2] = {-1.0, 1.0};

        default_joint_angles = {-0.1, 1.0, -1.5, 
                               0.1, 1.0, -1.5, 
                               -0.1, 0.8, -1.5,
                               0.1, 0.8, -1.5};
        
        obs_lin_vel_scale = 2.0;
        obs_ang_vel_scale = 0.25;
        obs_dof_pos_scale = 1.0;
        obs_dof_vel_scale = 0.05;

        clip_observations = 100;
        clip_actions = 100;

        num_observations = 45;
        num_actions = 12;        
    }
};


class PolicyRunner {
public:
    PolicyRunner(); 
    ~PolicyRunner(); 

    static Ort::Env env;

    PolicyParams params;
    Utils utils;

    Eigen::VectorXf delta_dof_pos = Eigen::VectorXf::Zero(NUM_JOINTS);
    Eigen::VectorXf dof_vel = Eigen::VectorXf::Zero(NUM_JOINTS);
    Eigen::VectorXf default_dof_pos = Eigen::VectorXf::Zero(NUM_JOINTS); 
    Eigen::VectorXf joint_vel = Eigen::VectorXf::Zero(NUM_JOINTS);  
    Eigen::VectorXf actions = Eigen::VectorXf::Zero(NUM_JOINTS);  
    Eigen::VectorXf scaled_actions = Eigen::VectorXf::Zero(NUM_JOINTS); 
    Eigen::Vector3f base_ang_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f projected_gravity = Eigen::Vector3f::Zero();
    
    std::array<float, NUM_JOINTS> m_jointPositions = {};  
    std::array<float, NUM_JOINTS> m_jointVelocities = {}; 
    
    Eigen::Vector3f commands = Eigen::Vector3f::Zero();
    Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
    Eigen::Vector3f gravity_vec = Eigen::Vector3f(0.0f, 0.0f, -1.0f);

    void Initialize_ONNX();
    void initialize_observation(Eigen::VectorXf joint_pos_);
    void set_base_ang_vel(Eigen::Vector3f base_ang_vel_);
    void set_projected_gravity(Eigen::Quaternion<float> quat_);
    void set_commands(float forward, float lateral, float yaw);  
    void set_joint_pos(Eigen::VectorXf joint_pos_);
    void set_joint_vel(Eigen::VectorXf joint_vel_);
    void set_actions(Eigen::VectorXf actions_);
    
    // Helper functions for conversion between float arrays and Eigen vectors
    void initialize_observation(const float joint_pos[NUM_JOINTS]) {
        Eigen::VectorXf eigen_pos = Eigen::Map<const Eigen::VectorXf>(joint_pos, NUM_JOINTS);
        initialize_observation(eigen_pos);
    }
    
    void set_joint_pos(const float joint_pos[NUM_JOINTS]) {
        Eigen::VectorXf eigen_pos = Eigen::Map<const Eigen::VectorXf>(joint_pos, NUM_JOINTS);
        set_joint_pos(eigen_pos);
    }
    
    void set_joint_vel(const float joint_vel[NUM_JOINTS]) {
        Eigen::VectorXf eigen_vel = Eigen::Map<const Eigen::VectorXf>(joint_vel, NUM_JOINTS);
        set_joint_vel(eigen_vel);
    }
    
    std::vector<float> infer(const std::vector<float>& obs);
    std::vector<float> compute_observation();
    std::vector<float> compute_policy();  



private:
    Ort::SessionOptions sessionOptions;
    std::unique_ptr<Ort::Session> session;
    
    std::vector<const char*> input_names; 
    std::vector<const char*> output_names;
    std::vector<std::string> input_names_str;
    std::vector<std::string> output_names_str;

    Ort::AllocatorWithDefaultOptions allocator;   
};

#endif