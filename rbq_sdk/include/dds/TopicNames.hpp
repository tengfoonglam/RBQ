#pragma once

#include <string>

namespace rbq_sdk {

const static std::string TOPIC_NAME_PREFIX      = "rbq/";
const static std::string TOPIC_NAME_INFO_IMU    = TOPIC_NAME_PREFIX+"info/imu";
const static std::string TOPIC_NAME_INFO_JOY    = TOPIC_NAME_PREFIX+"info/joy";
const static std::string TOPIC_NAME_INFO_SIM    = TOPIC_NAME_PREFIX+"info/sim";
const static std::string TOPIC_NAME_INFO_LEG    = TOPIC_NAME_PREFIX+"info/leg";
const static std::string TOPIC_NAME_INFO_WHL    = TOPIC_NAME_PREFIX+"info/whl";
const static std::string TOPIC_NAME_INFO_ARM    = TOPIC_NAME_PREFIX+"info/arm";

const static std::string TOPIC_NAME_PARAMETERS  = TOPIC_NAME_PREFIX+"parameters";

const static std::string TOPIC_NAME_REF_MOTION  = TOPIC_NAME_PREFIX+"ref/motion_";
const static std::string TOPIC_NAME_CMD_MOTION_OWN = TOPIC_NAME_PREFIX+"cmd/motion_own_";

const static std::string TOPIC_NAME_CMD_JOY     = TOPIC_NAME_PREFIX+"cmd/joy";

} // namespace rbq_sdk
