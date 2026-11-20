#pragma once

#include <string>
#include "robot_hwi.hpp"

// 统一获取机器人型号：直接走 RobotConfig（硬件描述）。
inline std::string GetRobotModel() {
    RobotConfig hwi_config;
    return hwi_config.getModel();
}
