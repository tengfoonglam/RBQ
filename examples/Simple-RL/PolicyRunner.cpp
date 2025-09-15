#include "PolicyRunner.h"

Ort::Env PolicyRunner::env(ORT_LOGGING_LEVEL_ERROR, "Example-RL");

PolicyRunner::PolicyRunner() {
    Initialize_ONNX();
    std::cout << "Initialize_ONNX successfully" << std::endl;
    
}

PolicyRunner::~PolicyRunner() {
}

void PolicyRunner::Initialize_ONNX() {
    sessionOptions = Ort::SessionOptions();
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    try {
        session = std::make_unique<Ort::Session>(env, params.path.c_str(), sessionOptions);

        input_names.clear();
        input_names_str.clear();

        size_t num_inputs = session->GetInputCount();
        for (size_t i = 0; i < num_inputs; ++i) {
            auto name_ptr = session->GetInputNameAllocated(i, allocator);
            std::string name(name_ptr.get()); 
            input_names_str.push_back(name);  
            input_names.push_back(input_names_str.back().c_str()); 
        }

        output_names.clear();
        output_names_str.clear();

        size_t num_outputs = session->GetOutputCount();
        for (size_t i = 0; i < num_outputs; ++i) {
            auto name_ptr = session->GetOutputNameAllocated(i, allocator);
            std::string name(name_ptr.get()); 
            output_names_str.push_back(name);
            output_names.push_back(output_names_str.back().c_str());
        }
    }
    catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
}

std::vector<float> PolicyRunner::infer(const std::vector<float>& obs) { 
    std::vector<int64_t> input_shape{1,static_cast<int64_t>(obs.size())};
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memInfo, const_cast<float*>(obs.data()), obs.size(), input_shape.data(), input_shape.size()
    );

    auto output_tensors = session->Run(
        Ort::RunOptions{nullptr},
        input_names.data(), &input_tensor, 1,
        output_names.data(), 1
    );

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    return std::vector<float>(output_data, output_data + output_size);
}

void PolicyRunner::initialize_observation(Eigen::VectorXf joint_pos_) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        m_jointPositions[i] = joint_pos_[i];
        m_jointVelocities[i] = 0.0f;
    }
    base_ang_vel.setZero();
    projected_gravity << 0.0f, 0.0f, -1.0f;
    commands.setZero();
    actions.setZero();  // Initialize actions
    scaled_actions.setZero();  // Initialize scaled_actions
    std::cout << "initialize_observation successfully" << std::endl;
}

void PolicyRunner::set_base_ang_vel(Eigen::Vector3f base_ang_vel_) {
    base_ang_vel = base_ang_vel_;
}

void PolicyRunner::set_projected_gravity(Eigen::Quaternion<float> quat_) {
    quat = quat_;
}

void PolicyRunner::set_commands(float forward, float lateral, float yaw) {
    commands << forward, lateral, yaw;
}

void PolicyRunner::set_joint_pos(Eigen::VectorXf joint_pos_) {
    // Update delta_dof_pos based on current position and default angles
    for (int i = 0; i < NUM_JOINTS; i++) {
        delta_dof_pos[i] = joint_pos_[i] - params.default_joint_angles[i];
    }
}

void PolicyRunner::set_joint_vel(Eigen::VectorXf joint_vel_) {
    dof_vel = joint_vel_;
}

void PolicyRunner::set_actions(Eigen::VectorXf actions_) {
    actions = actions_;
}

std::vector<float> PolicyRunner::compute_observation() {
    std::vector<float> obs;

    // base_ang_vel_tensor * base_ang_vel_scale
    for (int i = 0; i < 3; i++)
        obs.push_back(base_ang_vel[i] * params.obs_ang_vel_scale);

    // projected_gravity_tensor
    Eigen::Vector3f proj_grav = utils.quat_rotate_inverse(quat, gravity_vec);
    for (int i = 0; i < 3; i++)
        obs.push_back(proj_grav[i]);

    // commands_tensor * commands_scale
    static float commands_scale[] = {params.obs_lin_vel_scale, params.obs_lin_vel_scale, params.obs_ang_vel_scale};
    for (int i = 0; i < 3; i++)
        obs.push_back(commands[i] * commands_scale[i]);

    // delta_dof_pos_tensor * dof_pos_scale
    for (int i = 0; i < NUM_JOINTS; i++)
        obs.push_back(delta_dof_pos[i] * params.obs_dof_pos_scale);

    // dof_vel_tensor * dof_vel_scale
    for (int i = 0; i < NUM_JOINTS; i++)
        obs.push_back(dof_vel[i] * params.obs_dof_vel_scale);

    // actions_tensor.squeeze(0) → actions[]
    for (int i = 0; i < NUM_JOINTS; i++)
        obs.push_back(actions[i]);
    
    // Fix: Apply clamp to individual elements, not the whole vector
    for (size_t i = 0; i < obs.size(); i++) {
        obs[i] = std::clamp(obs[i], -params.clip_observations, params.clip_observations);
    }

    return obs;
}

std::vector<float> PolicyRunner::compute_policy() {
    // Safety check: if no session, return safe default actions
    if (!session) {
        std::vector<float> default_actions(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; i++) {
            default_actions[i] = params.default_joint_angles[i];
            actions[i] = default_actions[i];
            scaled_actions[i] = default_actions[i];
        }
        return default_actions;
    }

    try {
        std::vector<float> obs = compute_observation();

        // Additional safety check for observation size
        if (obs.size() != static_cast<size_t>(params.num_observations)) {
            std::cerr << "Observation size mismatch. Expected: " << params.num_observations 
                      << ", Got: " << obs.size() << std::endl;
            
            // Return safe default actions
            std::vector<float> default_actions(NUM_JOINTS);
            for (int i = 0; i < NUM_JOINTS; i++) {
                default_actions[i] = params.default_joint_angles[i];
                actions[i] = default_actions[i];
                scaled_actions[i] = default_actions[i];
            }
            return default_actions;
        }

        std::vector<int64_t> input_shape{1, static_cast<int64_t>(obs.size())};
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, obs.data(), obs.size(), input_shape.data(), input_shape.size()
        );
        
        auto output_tensors = session->Run(
            Ort::RunOptions{nullptr},
            input_names.data(), &input_tensor, 1,
            output_names.data(), 1
        );

        // Safety check for output tensor
        if (output_tensors.empty()) {
            std::cerr << "Empty output from ONNX model" << std::endl;
            return std::vector<float>(NUM_JOINTS, 0.0f);
        }

        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

        // Safety check for output size
        if (output_size < NUM_JOINTS) {
            std::cerr << "Output size too small. Expected at least: " << NUM_JOINTS 
                      << ", Got: " << output_size << std::endl;
            return std::vector<float>(NUM_JOINTS, 0.0f);
        }

        bool is_nan = false;
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (std::isnan(output_data[i]) || std::isinf(output_data[i])) {
                is_nan = true;
                break;
            }
        }       
        if (is_nan) {
            std::cerr << "NaN or Inf detected in output data" << std::endl;
            return std::vector<float>(NUM_JOINTS, 0.0f);  // Return zero actions if error
        }

        // Update actions member variable
        for (int i = 0; i < NUM_JOINTS; i++) {
            actions[i] = output_data[i];
        }   

        // Compute scaled actions
        for (int i = 0; i < NUM_JOINTS; i++) {
            scaled_actions[i] = actions[i] * params.action_scale;
            scaled_actions[i] = std::clamp(scaled_actions[i], -params.clip_actions, params.clip_actions);
        }

        // Return scaled actions as vector
        std::vector<float> result(NUM_JOINTS);
        for (int i = 0; i < NUM_JOINTS; i++) {
            result[i] = scaled_actions[i];
        }
        
        return result;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error during inference: " << e.what() << std::endl;
        return std::vector<float>(NUM_JOINTS, 0.0f);
    } catch (const std::exception& e) {
        std::cerr << "Standard exception during policy computation: " << e.what() << std::endl;
        return std::vector<float>(NUM_JOINTS, 0.0f);
    } catch (...) {
        std::cerr << "Unknown exception during policy computation" << std::endl;
        return std::vector<float>(NUM_JOINTS, 0.0f);
    }
}




