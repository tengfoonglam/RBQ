#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#define R2Df 57.295779513f
#define D2Rf 0.0174532925f

class Policy
{
public:
    explicit Policy(const std::string &path);
    ~Policy();

    enum ERROR {
        ERROR_NONE,
        ERROR_CONFIG_NOT_FOUND,
        ERROR_CONFIG_READ_FAIL,
        ERROR_CONFIG_PARSE_FAIL,
        ERROR_POLICY_NOT_FOUND,
        ERROR_POLICY_NOT_LOADED,
        ERROR_INVALID_INPUT_SIZE,
        ERROR_INVALID_OUTPUT_SIZE,
        ERROR_INPUT_SIZE_MISMATCH,
    };

    ///
    /// \brief compute
    /// \param imu_gyro
    /// \param imu_quat
    /// \param joint_pos
    /// \param joint_vel
    /// \param command
    /// \return action matrix of size (num_joints x 3). Each column represents: Position, Kp, Kd
    /// // Computes the action matrix based on IMU data, joint positions, velocities, and command inputs.
    ///
    Eigen::MatrixXf compute(const Eigen::Vector3f       &imu_gyro,
                            const Eigen::Quaternionf    &imu_quat,
                            const Eigen::VectorXf       &joint_pos,
                            const Eigen::VectorXf       &joint_vel,
                            const Eigen::Vector3f       &command,
                            const bool                  &reset);

    std::vector<float> compute(const std::vector<float> &input, const int &size, int &out_error);

    ERROR error() const { return m_error; }

private:
    bool m_loaded = false;
    ERROR m_error = ERROR_NONE;
    int* session = nullptr;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    struct Params;
    std::unique_ptr<Params> params;
};
