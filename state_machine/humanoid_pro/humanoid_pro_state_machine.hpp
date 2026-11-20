#ifndef HUMANOID_PRO_STATE_MACHINE_HPP_
#define HUMANOID_PRO_STATE_MACHINE_HPP_


#include "state_machine_base.hpp"
#include "humanoid_pro/pro_controllers.hpp"
#include "amp_moe_15dof_policy_runner.hpp"
#include "dds_interface.hpp"
#include "hardware/cr1pro_hardware_interface.hpp"
#include "keyboard_interface.hpp"
#include "pro_sdk_config.hpp"
#include "retroid_gamepad_interface.hpp"
#include "rl_stand_policy_runner.hpp"
#include "skydroid_gamepad_interface.hpp"
#include "udp_node.hpp"

namespace humanoid_pro {
class HumanoidStateMachine : public StateMachineBase {
public:
    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;
    const ControlMode control_mode_;

    HumanoidStateMachine(RobotName robot_name, RemoteCommandType remote_cmd_type,
                         ControlMode mode = ControlMode::kWholeBody)
        : StateMachineBase(RobotType::HumanoidPro), robot_name_(robot_name),
          remote_cmd_type_(remote_cmd_type), control_mode_(mode) {
        if (robot_name_ != RobotName::CR1PRO)
            throw std::invalid_argument("This state machine requires CR1PRO / 31 DOF");
    }

    ~HumanoidStateMachine() override { Stop(); }

    void Start() override {
        if (started_) throw std::logic_error("State machine is already started");
        switch (remote_cmd_type_) {
            case RemoteCommandType::kKeyboard:
                uc_ptr_ = std::make_shared<KeyboardInterface>(robot_name_, control_mode_); break;
            case RemoteCommandType::kDDS:
                uc_ptr_ = std::make_shared<DdsInterface>(robot_name_, control_mode_); break;
            case RemoteCommandType::kRetroid:
                uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121, robot_name_); break;
            case RemoteCommandType::kSkydroid:
                uc_ptr_ = std::make_shared<SkydroidGamepadInterface>(12121, robot_name_); break;
            default: throw std::invalid_argument("Unsupported command interface");
        }
        started_ = true;
        motion_state_store_ = std::make_shared<MotionStateStore>(&StateBase::msfb_);
        uc_ptr_->SetMotionStateFeedback(&StateBase::msfb_);
        uc_ptr_->SetMotionStateStore(motion_state_store_);
        uc_ptr_->SetControlMode(control_mode_);
        ConfigureUdp();
        uc_ptr_->SetUdpRemote(shared_udp_node_.get());

        ri_ptr_ = std::make_shared<CR1ProHardwareInterface>("CR1PRO");
        cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
        auto data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        data_ptr->motion_state_store = motion_state_store_;
        data_ptr->control_mode = control_mode_;
        data_ptr->udp_node = shared_udp_node_.get();

        ds_ptr_ = std::make_shared<DataStreaming>();
        ds_ptr_->SetRobotDataSource(ri_ptr_);
        ds_ptr_->SetUserCommandDataSource(uc_ptr_);
        data_ptr->ds_ptr = ds_ptr_;
        sc_ptr_ = std::make_shared<SafeController>(HumanoidPro, "");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);
        sc_ptr_->SetControlParamInfaceSource(cp_ptr_);
        sc_ptr_->SetMotionStateSource(motion_state_store_);
        data_ptr->sc_ptr = sc_ptr_;
        auto* ecan = dynamic_cast<EcanHardwareInterface*>(ri_ptr_.get());
        ecan->initJointProtection(cp_ptr_.get(), sc_ptr_->GetFaultClient());
        ecan->SetSafeControllerSource(sc_ptr_);

        controllers_ = std::make_unique<ProControllers>(robot_name_, data_ptr);
        current_controller_ = GetStateControllerPtr(StateName::kIdle);
        current_state_name_ = next_state_name_ = kIdle;
        du_ptr_ = std::make_shared<MotionDataUpload>();
        du_ptr_->SetMotionStateFeedbackSource(&StateBase::msfb_);
        du_ptr_->SetRobotInterfaceSource(ri_ptr_);
        du_ptr_->SetUserCommandInfaceSource(uc_ptr_);
        dp_ptr_ = std::make_shared<MotionDataPublish>();
        dp_ptr_->SetRobotInterfaceSource(ri_ptr_);
        dp_ptr_->SetMotionStateFeedbackSource(&StateBase::msfb_);
        dp_ptr_->SetSafeControllerSource(sc_ptr_);

        std::cout << "CR1PRO controllers start in 3 seconds" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        if (stop_requested_) return;
        ri_ptr_->Start();
        if (stop_requested_) return;
        // DDS replaces only the source ROS transport; keyboard and UDP use the
        // same source mode-key mapping, even when Keyboard is selected.
        uc_ptr_->EnableDdsInput();
        uc_ptr_->Start();
        ds_ptr_->Start();
        du_ptr_->Start();
        dp_ptr_->Start();
        sc_ptr_->Start();
        ri_ptr_->LoadNeckTakeoverConfig();
        current_controller_->OnEnter();
        controller_entered_ = true;
    }

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) override {
        return controllers_ ? controllers_->GetStateControllerPtr(state_name) : nullptr;
    }

    void Stop() override {
        if (!started_) return;
        started_ = false;
        if (controller_entered_ && current_controller_) {
            current_controller_->OnExit();
            controller_entered_ = false;
        }
        if (uc_ptr_) {
            uc_ptr_->Stop();
            uc_ptr_->EnableDdsInput(false);
            uc_ptr_->SetUdpRemote(nullptr);
        }
        if (sc_ptr_) sc_ptr_->Stop();
        if (du_ptr_) du_ptr_->Stop();
        if (dp_ptr_) dp_ptr_->Stop();
        if (ds_ptr_) ds_ptr_->Stop();
        if (ri_ptr_) {
            ri_ptr_->SetJointCommand(MatXf::Zero(31, 5));
            ri_ptr_->Stop();
        }
        current_controller_.reset();
        controllers_.reset();
        SetSharedUdp(nullptr);
        if (shared_udp_node_) shared_udp_node_->stop();
        shared_udp_node_.reset();
    }

private:
    std::unique_ptr<ProControllers> controllers_;
    std::shared_ptr<MotionStateStore> motion_state_store_;
    std::unique_ptr<UdpNode> shared_udp_node_;
    bool started_ = false;
    bool controller_entered_ = false;

    static void SetSharedUdp(UdpNode* node) {
        RLStandPolicyRunner::SetUdpNode(node);
        AmpMoe15DofPolicyRunner::SetUdpNode(node);
    }

    void ConfigureUdp() {
        const auto config = LoadProSdkConfig();
        if (config.udp_enabled) {
            shared_udp_node_ = std::make_unique<UdpNode>(config.udp_local_ip, config.udp_local_port,
                                                       config.udp_remote_ip, config.udp_remote_port);
            if (!shared_udp_node_->start()) {
                std::cerr << "PRO UDP unavailable; local keyboard and DDS remain active" << std::endl;
                shared_udp_node_.reset();
            }
        }
        SetSharedUdp(shared_udp_node_.get());
    }
};
}  // namespace humanoid_pro

#endif
