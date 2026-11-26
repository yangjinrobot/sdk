/**
 * @file humanoid_state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */
#ifndef HUMANOID_STATE_MACHINE_HPP_
#define HUMANOID_STATE_MACHINE_HPP_

#include "humanoid/idle_state.hpp"
#include "humanoid/joint_damping_state.hpp"
#include "humanoid/suspended_standup_state.hpp"
#include "humanoid/zero_calibration_state.hpp"
#include "humanoid/zeropos_state.hpp"

#include "data_streaming.hpp"
#include "dds_interface.hpp"
#include "keyboard_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "safe_controller/safe_controller.hpp"
#include "skydroid_gamepad_interface.hpp"
#include "state_machine_base.h"

#include <cstdlib>
#include <iostream>
#include <memory>

#ifdef USE_SIMULATION
#include "simulation/pybullet_interface.hpp"
#else
#include "hardware/cr1pro_hardware_interface.hpp"
#include "hardware/cr1std_hardware_interface.hpp"
#endif

namespace humanoid {
class HumanoidStateMachine : public StateMachineBase {
private:
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> zeropos_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> suspended_standup_controller_;
    std::shared_ptr<StateBase> zero_calibration_controller_;

public:
    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;

    HumanoidStateMachine(RobotName robot_name, RemoteCommandType remote_cmd_type)
        : StateMachineBase(RobotType::HumanoidLeg), robot_name_(robot_name),
          remote_cmd_type_(remote_cmd_type) {}

    void Start() override {
        switch (remote_cmd_type_) {
            case RemoteCommandType::kRetroid:
                uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121, robot_name_);
                break;
            case RemoteCommandType::kSkydroid:
                uc_ptr_ = std::make_shared<SkydroidGamepadInterface>(12121, robot_name_);
                break;
            case RemoteCommandType::kDDS:
                uc_ptr_ = std::make_shared<DdsInterface>(robot_name_);
                break;
            case RemoteCommandType::kKeyboard:
                uc_ptr_ = std::make_shared<KeyboardInterface>(robot_name_);
                break;
            default:
                std::cerr << "error user command interface! " << std::endl;
                std::exit(0);
        }
        uc_ptr_->SetMotionStateFeedback(&StateBase::msfb_);

        if (robot_name_ == RobotName::CR1STANDARD) {
#ifdef USE_SIMULATION
            ri_ptr_ = std::make_shared<PybulletInterface>("CR1STANDARD");
#else
            ri_ptr_ = std::make_shared<CR1StdHardwareInterface>("CR1STANDARD");
#endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
        } else if (robot_name_ == RobotName::CR1PRO) {
#ifdef USE_SIMULATION
            ri_ptr_ = std::make_shared<PybulletInterface>("CR1PRO");
#else
            ri_ptr_ = std::make_shared<CR1ProHardwareInterface>("CR1PRO");
#endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
        }

        auto data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;

        ds_ptr_ = std::make_shared<DataStreaming>();
        ds_ptr_->SetRobotDataSource(ri_ptr_);
        ds_ptr_->SetUserCommandDataSource(uc_ptr_);
        data_ptr->ds_ptr = ds_ptr_;

        sc_ptr_ = std::make_shared<SafeController>(HumanoidStandard, "");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);
        sc_ptr_->SetControlParamInfaceSource(cp_ptr_);
        data_ptr->sc_ptr = sc_ptr_;

        // 初始化关节保护检查器（复用 SafeController 的 fault_client）
#ifndef USE_SIMULATION
        if (auto* ecan_ri = dynamic_cast<EcanHardwareInterface*>(ri_ptr_.get())) {
            ecan_ri->initJointProtection(cp_ptr_.get(), sc_ptr_->GetFaultClient());
            ecan_ri->SetSafeControllerSource(sc_ptr_);
        }
#endif

        du_ptr_ = std::make_shared<MotionDataUpload>();
        du_ptr_->SetMotionStateFeedbackSource(&StateBase::msfb_);
        du_ptr_->SetRobotInterfaceSource(ri_ptr_);
        du_ptr_->SetUserCommandInfaceSource(uc_ptr_);
#ifndef USE_SIMULATION
        dp_ptr_ = std::make_shared<MotionDataPublish>();
        dp_ptr_->SetRobotInterfaceSource(ri_ptr_);
        dp_ptr_->SetMotionStateFeedbackSource(&StateBase::msfb_);
        dp_ptr_->SetSafeControllerSource(sc_ptr_);
#endif

        idle_controller_ = std::make_shared<IdleState>(robot_name_, "idle_state", data_ptr);
        zeropos_controller_ = std::make_shared<ZeroPosState>(robot_name_, "zeropos_state", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name_, "joint_damping", data_ptr);
        suspended_standup_controller_ = std::make_shared<SuspendedStandUpState>(robot_name_, "suspended_standup", data_ptr);
        CreateAdditionalControllers(data_ptr);
        zero_calibration_controller_ = std::make_shared<ZeroCalibrationState>(robot_name_, "zero_calibration_state", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = next_state_name_ = StateName::kIdle;

        std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety

        ri_ptr_->Start();
        std::cout << "1" << std::endl;
        uc_ptr_->Start();
        std::cout << "2" << std::endl;
        ds_ptr_->Start();
        std::cout << "3" << std::endl;
        du_ptr_->Start();
        std::cout << "4" << std::endl;
#ifndef USE_SIMULATION
        dp_ptr_->Start();
        std::cout << "5" << std::endl;
#endif

#ifndef USE_RAISIM
        sc_ptr_->Start();
#endif

        // 脖子接管配置加载（代码下沉至 utils::NeckTakeover）
        ri_ptr_->LoadNeckTakeoverConfig();

        current_controller_->OnEnter();
        std::cout << "start finish" << std::endl;
    }

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) override {
        switch (state_name) {
            case StateName::kInvalid:
                return nullptr;
            case StateName::kIdle:
                return idle_controller_;
            case StateName::kSuspendedStand:
                return suspended_standup_controller_;
            case StateName::kJointDamping:
                return joint_damping_controller_;
            case StateName::kZeroPos:
                return zeropos_controller_;
            case StateName::kZeroCalibration:
                return zero_calibration_controller_;
            case StateName::kAmpControlWalk2RunMode:
            case StateName::kMimicControlKneelMode:
            case StateName::kMimicControlKneelStandUpMode:
            case StateName::kRLControlStandMode:
            case StateName::kMimicControlChairDownMode:
            case StateName::kMimicControlChairUpMode:
                return joint_damping_controller_;
            default:
                std::cerr << "error state name" << std::endl;
                return joint_damping_controller_;
        }
    }

    bool IsStateTransitionAllowed(StateName next_state_name) const override {
        (void)next_state_name;
        return true;
    }
    void Stop() override {
        uc_ptr_->Stop();
        ri_ptr_->Stop();
        ds_ptr_->Stop();
    }

protected:
    virtual void CreateAdditionalControllers(const std::shared_ptr<ControllerData>&) {}
};
}  // namespace humanoid
#endif
