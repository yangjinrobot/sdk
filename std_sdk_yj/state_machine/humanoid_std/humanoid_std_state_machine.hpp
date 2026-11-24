#ifndef HUMANOID_STD_STATE_MACHINE_HPP_
#define HUMANOID_STD_STATE_MACHINE_HPP_

#include "humanoid/humanoid_state_machine.hpp"

#include "chiqiangwalk_policy_runner.h"
#include "humanoid_std/dunxia_state.hpp"
#include "humanoid_std/mjrun_state.hpp"
#include "humanoid_std/restand_state.hpp"
#include "humanoid_std/run_state.hpp"
#include "humanoid_std/sit2stand_mimic_state.hpp"
#include "humanoid_std/squat_preparation_state.hpp"
#include "humanoid_std/stand2sit_mimic_state.hpp"
#include "humanoid_std/zhanli_state.hpp"

namespace humanoid_std {

class HumanoidStateMachine final : public humanoid::HumanoidStateMachine {
public:
    explicit HumanoidStateMachine(RemoteCommandType remote_cmd_type)
        : humanoid::HumanoidStateMachine(RobotName::CR1STANDARD, remote_cmd_type) {}

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) override {
        switch (state_name) {
            case StateName::kSquatPreparation:
                return squat_preparation_controller_;
            case StateName::kRun:
                return run_controller_;
            case StateName::kMjRun:
                return mjrun_controller_;
            case StateName::kDunXia:
                return dunxia_controller_;
            case StateName::kZhanLi:
                return zhanli_controller_;
            case StateName::kRestand:
                return restand_controller_;
            case StateName::kStand2SitMimic:
                return stand2sit_mimic_controller_;
            case StateName::kSit2StandMimic:
                return sit2stand_mimic_controller_;
            default:
                return humanoid::HumanoidStateMachine::GetStateControllerPtr(state_name);
        }
    }

private:
    std::shared_ptr<StateBase> CreateRLController(
        const std::shared_ptr<ControllerData>& data_ptr) override {
        auto walk_policy = std::make_shared<ChiqiangWalkPolicyRunner>(
            "rl_control", robot_name_,
            "/opt/robot/share/motion_master/policy/amp/std_amp_2.onnx");
        return std::make_shared<humanoid::RLControlState>(
            robot_name_, "rl_control", data_ptr, std::move(walk_policy),
            RobotMotionState::RLControlMode);
    }

    void CreateAdditionalControllers(const std::shared_ptr<ControllerData>& data_ptr) override {
        squat_preparation_controller_ = std::make_shared<SquatPreparationState>(robot_name_, "squat_preparation", data_ptr);
        stand2sit_mimic_controller_ = std::make_shared<Stand2SitMimicState>(robot_name_, "stand2sit_mimic", data_ptr);
        sit2stand_mimic_controller_ = std::make_shared<Sit2StandMimicState>(robot_name_, "sit2stand_mimic", data_ptr);
        run_controller_ = std::make_shared<RunState>(robot_name_, "run", data_ptr);
        mjrun_controller_ = std::make_shared<MjRunState>(robot_name_, "mjrun", data_ptr);
        restand_controller_ = std::make_shared<RestandState>(robot_name_, "restand", data_ptr);
        dunxia_controller_ = std::make_shared<DunXiaState>(robot_name_, "dunxia", data_ptr);
        zhanli_controller_ = std::make_shared<ZhanLiState>(robot_name_, "zhanli", data_ptr);
    }

    std::shared_ptr<StateBase> squat_preparation_controller_;
    std::shared_ptr<StateBase> run_controller_;
    std::shared_ptr<StateBase> mjrun_controller_;
    std::shared_ptr<StateBase> restand_controller_;
    std::shared_ptr<StateBase> dunxia_controller_, zhanli_controller_;
    std::shared_ptr<StateBase> stand2sit_mimic_controller_, sit2stand_mimic_controller_;
};

}  // namespace humanoid_std

#endif
