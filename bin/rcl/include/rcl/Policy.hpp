#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

#include <onnxruntime_cxx_api.h>

#define R2Df 57.295779513f
#define D2Rf 0.0174532925f

class Policy
{
public:
    explicit Policy() : m_loaded(false) {}
    ~Policy() { release(); }

    enum ERROR {
        ERROR_NONE,
        ERROR_POLICY_NOT_FOUND,
        ERROR_POLICY_NOT_LOADED,
        ERROR_INVALID_INPUT_SIZE,
        ERROR_INVALID_OUTPUT_SIZE,
        ERROR_INPUT_SIZE_MISMATCH,
        ERROR_OUTPUT_DATA_HAS_NAN_OR_INF,
    };

    void reload(const std::string &path)
    {
        try {
            release();
            const std::string policy = path + "/policy.onnx";
            if (!std::filesystem::exists(policy)) {
                throw(std::string("Policy file path is wrong: " + policy));
                m_error = ERROR_POLICY_NOT_LOADED;
            }
            try {
                Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Policy");
                Ort::SessionOptions session_options = Ort::SessionOptions();
                session_options.SetIntraOpNumThreads(1);
                session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
                Ort::Session* session = new Ort::Session(env, policy.c_str(), session_options);
                Ort::AllocatorWithDefaultOptions allocator;
                for (size_t i = 0; i < session->GetInputCount(); i++) {
                    input_names_str.push_back(std::string(session->GetInputNameAllocated(i, allocator).get()));
                    input_names.push_back(input_names_str.back().c_str());
                }
                for (size_t i = 0; i < session->GetOutputCount(); i++) {
                    output_names_str.push_back(std::string(session->GetOutputNameAllocated(i, allocator).get()));
                    output_names.push_back(output_names_str.back().c_str());
                }
                if (session == nullptr) {
                    throw(std::string("Policy file load failed: " + policy + "session is nullptr after load"));
                    m_error = ERROR_POLICY_NOT_LOADED;
                }
                this->session = (int *)session;
            }  catch (...) {
                throw(std::string("Policy file load failed: " + policy));
                m_error = ERROR_POLICY_NOT_LOADED;
            }
            std::cout << "Policy file parse complete: " << policy << std::endl;
            m_loaded = true;
        } catch (const std::string &error) {
            std::cerr << error << std::endl;
        }
    }

    void release()
    {
        m_loaded = false;
        input_names.clear();
        input_names_str.clear();
        output_names.clear();
        output_names_str.clear();
        m_error = ERROR_NONE;
        if (session) {
            ((Ort::Session*)session)->release();
            delete (Ort::Session*)session;
            session = nullptr;
        }
    }

    ERROR error() const { return m_error; }

    bool loaded() const { return m_loaded; }

    /**
     * @brief inference with given input data
     * @param inputData
     * @param inputSize
     * @param outputSize
     * @param out_error
     * @return
     */
    std::vector<float> inference(const std::vector<float> &inputData, const int &inputSize, const int &outputSize, int &out_error) {
        std::vector<float> result(outputSize, 0.0f);
        out_error = ERROR_NONE;
        try {
            if (!m_loaded) {
                out_error = ERROR_POLICY_NOT_LOADED;
                throw(std::string("[ERROR] Policy not loaded"));
            }
            if (!session) {
                out_error = ERROR_POLICY_NOT_LOADED;
                throw(std::string("[ERROR] Policy invalid session"));
            }
            if (input_names.empty()) {
                out_error = ERROR_INVALID_INPUT_SIZE;
                throw(std::string("[ERROR] Policy invalid input names"));
            }
            if (output_names.empty()) {
                out_error = ERROR_INVALID_OUTPUT_SIZE;
                throw(std::string("[ERROR] Policy invalid output names"));
            }
            if (inputData.size() != static_cast<size_t>(inputSize)) {
                out_error = ERROR_INPUT_SIZE_MISMATCH;
                throw(std::string("[ERROR] Policy observation size mismatch"));
            }
            std::vector<int64_t> input_shape{1, static_cast<int64_t>(inputData.size())};
            Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            std::vector<float> input_ = inputData;
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info,
                input_.data(), input_.size(),
                input_shape.data(), input_shape.size());
            auto output_tensors = ((Ort::Session*)session)->Run(
                Ort::RunOptions{nullptr},
                input_names.data(), &input_tensor, 1,
                output_names.data(), 1
                );
            float* output_data = output_tensors[0].GetTensorMutableData<float>();
            for (int i = 0; i < outputSize; i++) {
                if (std::isnan(output_data[i]) || std::isinf(output_data[i])) {
                    out_error = ERROR_OUTPUT_DATA_HAS_NAN_OR_INF;
                    throw(std::string("[ERROR] NaN or Inf detected in output data") );
                }
            }
            for (int i = 0; i < outputSize; i++) {
                result[i] = output_data[i];
            }
        } catch (const std::string &error) {
            std::cerr << error << std::endl;
        }
        m_error = (ERROR)out_error;
        return result;
    }

private:
    bool m_loaded = false;
    ERROR m_error = ERROR_NONE;
    int* session = nullptr;
    std::vector<const char*> input_names;
    std::vector<std::string> input_names_str;
    std::vector<const char*> output_names;
    std::vector<std::string> output_names_str;
};
