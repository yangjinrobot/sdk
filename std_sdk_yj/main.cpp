#include "humanoid/humanoid_state_machine.hpp"
#include "humanoid_std/humanoid_std_state_machine.hpp"

#include "dr_logger.hpp"
#include "drdds/core/drdds_core.h"
#include "runtime_params.hpp"
#include "toml/toml.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace types;
MotionStateFeedback StateBase::msfb_{};

int main() {
    std::cout << "State Machine Start Running" << std::endl;
    RobotName robot_name = RobotName::CR1STANDARD;
    std::string model_name = "CR1_STANDARD";
    RemoteCommandType remote_cmd_type = RemoteCommandType::kDDS;
    std::string ctrl_mode_str = "kDDS";
    if (const char* value = std::getenv("MOTION_MASTER_CTRL_MODE")) {
        std::string mode(value);
        std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (mode == "keyboard") {
            remote_cmd_type = RemoteCommandType::kKeyboard;
            ctrl_mode_str = "kKeyboard";
        } else if (mode == "retroid") {
            remote_cmd_type = RemoteCommandType::kRetroid;
            ctrl_mode_str = "kRetroid";
        } else if (mode == "skydroid") {
            remote_cmd_type = RemoteCommandType::kSkydroid;
            ctrl_mode_str = "kSkydroid";
        }
    }
#ifndef USE_SIMULATION
    DrDDSManager::Init(0, "eth0");
    DrLogger::initialize("motion_master", "/var/opt/robot/log/", "/var/opt/robot/conf/log.properties");
    try {
        const auto config = toml::parse("/var/opt/robot/conf/robot_hardware_info.toml");
        model_name = toml::find<std::string>(config, "hardware", "model");
        if (model_name == "CR1_PRO") {
            robot_name = RobotName::CR1PRO;
        }
    } catch (const toml::syntax_error& error) {
        throw std::runtime_error("Error parsing TOML config: " + std::string(error.what()));
    }
#endif
    const char* model = robot_name == RobotName::CR1PRO ? "CR1_PRO" : "CR1_STANDARD";
    std::cout << "Robot model: " << model << ", command interface: " << ctrl_mode_str << std::endl;

    auto& runtime_params = RuntimeParams::Instance();
    runtime_params.SetString("robot", "model", model);
    runtime_params.SetString("robot", "hwi_name", model_name);
    runtime_params.SetString("robot", "ctrl_mode", ctrl_mode_str);
    runtime_params.Start(1000);

    std::unique_ptr<StateMachineBase> state_machine;
    if (robot_name == RobotName::CR1STANDARD) {
        state_machine = std::make_unique<humanoid_std::HumanoidStateMachine>(remote_cmd_type);
    } else {
        state_machine = std::make_unique<humanoid::HumanoidStateMachine>(robot_name, remote_cmd_type);
    }
    state_machine->Start();
    state_machine->Run();
    state_machine->Stop();
    runtime_params.Stop();
    return 0;
}
