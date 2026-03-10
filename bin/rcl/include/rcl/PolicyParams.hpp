#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>

#include "nlohmann/json.hpp"

struct PolicyParams {
    std::string name;
    float   dt;
    float   action_scale;
    float   KP[12];
    float   KD[12];
    int     num_observations;
    int     num_actions;
    float   command_ranges[3][2];
    std::vector<float> default_joint_angles;
    float   obs_lin_vel_scale;
    float   obs_ang_vel_scale;
    float   obs_dof_pos_scale;
    float   obs_dof_vel_scale;
    float   clip_observations;
    float   clip_actions;

    enum ERROR {
        ERROR_NONE,
        ERROR_CONFIG_NOT_FOUND,
        ERROR_CONFIG_READ_FAIL,
        ERROR_CONFIG_PARSE_FAIL,
    };
    ERROR error() {return m_error;}
    bool loaded() {return m_loaded;}

    void loadFromPath(const std::string& path) {
        m_loaded = false;
        const std::string config = path + "/info.json";
        if (!std::filesystem::exists(config)) {
            throw(std::string("Config file path is wrong: " + config));
            m_error = ERROR_CONFIG_NOT_FOUND;
        }
        std::ifstream file(config, std::ios::in);
        if (!file.is_open()) {
            throw(std::string("Config file read failed: " + config));
            m_error = ERROR_CONFIG_READ_FAIL;
        }
        try {
            nlohmann::json j;
            file >> j;
            name = j["config_info"]["run_name"];
            dt   = j["config_info"]["control"]["policy_dt"];
            action_scale = j["config_info"]["control"]["action_scale"];
            for (int lnum = 0; lnum < 4; lnum++) {
                KP[lnum*3 + 0]    = j["config_info"]["control"]["stiffness"]["R"];
                KP[lnum*3 + 1]    = j["config_info"]["control"]["stiffness"]["P"];
                KP[lnum*3 + 2]    = j["config_info"]["control"]["stiffness"]["K"];
                KD[lnum*3 + 0]      = j["config_info"]["control"]["damping"]["R"];
                KD[lnum*3 + 1]      = j["config_info"]["control"]["damping"]["P"];
                KD[lnum*3 + 2]      = j["config_info"]["control"]["damping"]["K"];
            }
            num_observations = j["config_info"]["env"]["num_observations"];
            num_actions      = j["config_info"]["env"]["num_actions"];
            for (int i = 0; i < 2; i++) {
                command_ranges[0][i] = j["config_info"]["commands"]["ranges"]["lin_vel_x"][i];
                command_ranges[1][i] = j["config_info"]["commands"]["ranges"]["lin_vel_y"][i];
                command_ranges[2][i] = j["config_info"]["commands"]["ranges"]["ang_vel_yaw"][i];
            }
            obs_lin_vel_scale        = j["config_info"]["normalization"]["obs_scales"]["lin_vel"];
            obs_ang_vel_scale        = j["config_info"]["normalization"]["obs_scales"]["ang_vel"];
            obs_dof_pos_scale        = j["config_info"]["normalization"]["obs_scales"]["dof_pos"];
            obs_dof_vel_scale        = j["config_info"]["normalization"]["obs_scales"]["dof_vel"];
            clip_observations        = j["config_info"]["normalization"]["clip_observations"];
            clip_actions             = j["config_info"]["normalization"]["clip_actions"];
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint0_HRR"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint1_HRP"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint2_HRK"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint3_HLR"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint4_HLP"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint5_HLK"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint6_FRR"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint7_FRP"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint8_FRK"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint9_FLR"] );
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint10_FLP"]);
            default_joint_angles.push_back(j["config_info"]["init_state"]["default_joint_angles"]["joint11_FLK"]);
            m_error = ERROR_NONE;
            m_loaded = true;
            printf("Config file parse complete: %s\n", config.c_str());
        } catch (const std::exception &e) {
            throw(std::string("Config file parse failed: " + config + ". Error: " + e.what()));
            m_error = ERROR_CONFIG_PARSE_FAIL;
        }
    }

private:
    ERROR m_error = ERROR_NONE;
    bool m_loaded = false;
};
