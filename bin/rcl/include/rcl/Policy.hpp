#pragma once

#include <string>
#include <vector>

#define R2Df 57.295779513f
#define D2Rf 0.0174532925f

class Policy
{
public:
    explicit Policy(const std::string &path);
    ~Policy();

    enum ERROR {
        ERROR_NONE,
        ERROR_POLICY_NOT_FOUND,
        ERROR_POLICY_NOT_LOADED,
        ERROR_INVALID_INPUT_SIZE,
        ERROR_INVALID_OUTPUT_SIZE,
        ERROR_INPUT_SIZE_MISMATCH,
        ERROR_OUTPUT_DATA_HAS_NAN_OR_INF,
    };

    std::vector<float> compute(const std::vector<float> &inputData, const int &inputSize, const int &outputSize, int &out_error);

    ERROR error() const { return m_error; }

private:
    bool m_loaded = false;
    ERROR m_error = ERROR_NONE;
    int* session = nullptr;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
};
