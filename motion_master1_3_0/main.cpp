
#include "humanoid/humanoid_state_machine.hpp"

#include "dr_logger.hpp"
#include "drdds/core/drdds_core.h"
#include "toml/toml.hpp"

#include "runtime_params.hpp"

#include <cstring>

using namespace types;
MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
    std::cout << "State Machine Start Running" << std::endl;
    RobotName hwi_name;
    std::string model_str = "CR1_STANDARD";
    RemoteCommandType remote_cmd_type = RemoteCommandType::kDDS;
    std::string ctrl_mode_str = "kDDS";
    const char* ctrl_mode_env = std::getenv("MOTION_MASTER_CTRL_MODE");
    if (ctrl_mode_env != nullptr) {
        std::string mode(ctrl_mode_env);
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
        DrDDSManager::Init(0,"eth0");
        DrLogger::initialize("motion_master", "/var/opt/robot/log/", "/var/opt/robot/conf/log.properties");
        try {
            auto value = toml::parse("/var/opt/robot/conf/robot_hardware_info.toml");
            std::string model_ = toml::find<std::string>(value, "hardware", "model");
            if(model_=="CR1_PRO") {hwi_name = RobotName::CR1PRO; std::cout<<"Robot model is CR1_PRO!"<<std::endl;}
            else {hwi_name = RobotName::CR1STANDARD; std::cout<<"Robot model is CR1_STANDARD!"<<std::endl;}
            model_str = model_;
        } catch (const toml::syntax_error& e) {
            throw std::runtime_error("Error parsing TOML config: " + std::string(e.what()));
        }  
    #endif

    // ---- 运行期参数实时快照：写入已知真实生效值并启动后台回写 ----
    {
        auto& rp = RuntimeParams::Instance();
        rp.SetString("robot", "model",
                     hwi_name == RobotName::CR1PRO ? "CR1_PRO" : "CR1_STANDARD");
        rp.SetString("robot", "hwi_name",  model_str);
        rp.SetString("robot", "ctrl_mode", ctrl_mode_str);  // 与下方 HumanoidStateMachine 入参保持一致
        rp.Start(1000);                              // 自动探测路径，1Hz 回写
    }

    std::shared_ptr<StateMachineBase> fsm = std::make_shared<humanoid::HumanoidStateMachine>(RobotName::CR1STANDARD, RemoteCommandType::kKeyboard);
    // std::shared_ptr<StateMachineBase> fsm =
    //     std::make_shared<humanoid::HumanoidStateMachine>(hwi_name, remote_cmd_type);

    fsm->Start();
    fsm->Run();
    fsm->Stop();

    RuntimeParams::Instance().Stop();
    return 0;
}
