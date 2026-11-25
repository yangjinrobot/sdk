
#include "humanoid_pro/humanoid_pro_state_machine.hpp"

#include "dr_logger.hpp"
#include "drdds/core/drdds_core.h"
#include "toml/toml.hpp"

#include "runtime_params.hpp"

#include <cstring>
#include <cstdlib>
#include <cctype>

using namespace types;

int main(){
    std::cout << "State Machine Start Running" << std::endl;
    const RobotName robot_name = RobotName::CR1PRO;
    RemoteCommandType remote_cmd_type = RemoteCommandType::kDDS;
    std::string ctrl_mode_str = "kDDS";
    if (const char* value = std::getenv("MOTION_MASTER_CTRL_MODE")) {
        std::string mode(value);
        std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (mode == "keyboard") {
            remote_cmd_type = RemoteCommandType::kKeyboard; ctrl_mode_str = "kKeyboard";
        } else if (mode == "retroid") {
            remote_cmd_type = RemoteCommandType::kRetroid; ctrl_mode_str = "kRetroid";
        } else if (mode == "skydroid") {
            remote_cmd_type = RemoteCommandType::kSkydroid; ctrl_mode_str = "kSkydroid";
        } else if (!mode.empty() && mode != "dds") {
            std::cerr << "Unknown MOTION_MASTER_CTRL_MODE: " << value << std::endl;
            return 1;
        }
    }
    std::cout << "Command interface: " << ctrl_mode_str << std::endl;
    std::string model_str;
    DrDDSManager::Init(0, "eth0");
    DrLogger::initialize("motion_master", "/var/opt/robot/log/", "/var/opt/robot/conf/log.properties");
    try {
        auto value = toml::parse("/var/opt/robot/conf/robot_hardware_info.toml");
        model_str = toml::find<std::string>(value, "hardware", "model");
        std::cout << "Configured hardware model: " << model_str << std::endl;
        if (model_str != "CR1_PRO") {
            std::cerr << "This migrated controller requires hardware.model = CR1_PRO" << std::endl;
            return 1;
        }
    } catch (const toml::syntax_error& e) {
        throw std::runtime_error("Error parsing TOML config: " + std::string(e.what()));
    }

    // ---- 运行期参数实时快照：写入已知真实生效值并启动后台回写 ----
    {
        auto& rp = RuntimeParams::Instance();
        rp.SetString("robot", "model",
                     robot_name == RobotName::CR1PRO ? "CR1_PRO" : "CR1_STANDARD");
        rp.SetString("robot", "hwi_name",  model_str);
        rp.SetString("robot", "ctrl_mode", ctrl_mode_str);
        rp.SetString("robot", "sdk_mode", "whole_body");
        rp.Start(1000);                              // 自动探测路径，1Hz 回写
    }

    std::shared_ptr<StateMachineBase> fsm =
        std::make_shared<humanoid::HumanoidStateMachine>(robot_name, remote_cmd_type);

    std::signal(SIGINT, StateMachineBase::RequestStop);
    std::signal(SIGTERM, StateMachineBase::RequestStop);
    try {
        fsm->Start();
        fsm->Run();
        fsm->Stop();
    } catch (const std::exception& error) {
        std::cerr << "CR1PRO startup/control failed: " << error.what() << std::endl;
        fsm->Stop();
        RuntimeParams::Instance().Stop();
        return 1;
    }
    RuntimeParams::Instance().Stop();
    return 0;
}
