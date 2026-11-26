#ifndef HUMANOID_STD_STATE_MACHINE_HPP_
#define HUMANOID_STD_STATE_MACHINE_HPP_

#include "humanoid/humanoid_state_machine.hpp"

#include "amp_control_ni_ren_walk_mode_policy_runner.h"
#include "amp_control_gun_shoot_walk_mode_policy_runner.h"
#include "humanoid_std/amp_control_ni_ren_walk_mode_state.hpp"
#include "humanoid_std/amp_control_gun_shoot_walk_mode_state.hpp"
#include "humanoid_std/mimic_control_kneel_mode_state.hpp"
#include "humanoid_std/mjlab_control_walk2run_mode_state.hpp"
#include "humanoid_std/rl_control_stand_mode_state.hpp"
#include "humanoid_std/amp_control_walk2run_mode_state.hpp"
#include "humanoid_std/mimic_control_chair_up_mode_state.hpp"
#include "humanoid_std/squat_preparation_state.hpp"
#include "humanoid_std/mimic_control_chair_down_mode_state.hpp"
#include "humanoid_std/mimic_control_kneel_stand_up_mode_state.hpp"

namespace humanoid_std {

class HumanoidStateMachine final : public humanoid::HumanoidStateMachine {
public:
    explicit HumanoidStateMachine(RemoteCommandType remote_cmd_type)
        : humanoid::HumanoidStateMachine(RobotName::CR1STANDARD, remote_cmd_type) {}

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) override {
        switch (state_name) {
            case StateName::kAmpControlNiRenWalkMode:
                return amp_control_ni_ren_walk_mode_controller_;
            case StateName::kAmpControlGunShootWalkMode:
                return amp_control_gun_shoot_walk_mode_controller_;
            case StateName::kSquatPreparation:
                return squat_preparation_controller_;
            case StateName::kAmpControlWalk2RunMode:
                return amp_control_walk2run_mode_controller_;
            case StateName::kMjlabControlWalk2RunMode:
                return mjlab_control_walk2run_mode_controller_;
            case StateName::kMimicControlKneelMode:
                return mimic_control_kneel_mode_controller_;
            case StateName::kMimicControlKneelStandUpMode:
                return mimic_control_kneel_stand_up_mode_controller_;
            case StateName::kRLControlStandMode:
                return rl_control_stand_mode_controller_;
            case StateName::kMimicControlChairDownMode:
                return mimic_control_chair_down_mode_controller_;
            case StateName::kMimicControlChairUpMode:
                return mimic_control_chair_up_mode_controller_;
            default:
                return humanoid::HumanoidStateMachine::GetStateControllerPtr(state_name);
        }
    }

private:
    void CreateAdditionalControllers(const std::shared_ptr<ControllerData>& data_ptr) override {
        auto amp_control_ni_ren_walk_policy =
            std::make_shared<AmpControlNiRenWalkModePolicyRunner>(
                "amp_control_ni_ren_walk_mode", robot_name_,
                AmpControlNiRenWalkModePolicyRunner::kDefaultPolicyPath);
        amp_control_ni_ren_walk_mode_controller_ =
            std::make_shared<AmpControlNiRenWalkModeState>(
                robot_name_, "amp_control_ni_ren_walk_mode", data_ptr,
                std::move(amp_control_ni_ren_walk_policy),
                RobotMotionState::AmpControl_NiRenWalk_Mode);

        auto amp_control_gun_shoot_walk_policy =
            std::make_shared<AmpControlGunShootWalkModePolicyRunner>(
                "amp_control_gun_shoot_walk_mode", robot_name_,
                AmpControlGunShootWalkModePolicyRunner::kDefaultPolicyPath);
        amp_control_gun_shoot_walk_mode_controller_ =
            std::make_shared<AmpControlGunShootWalkModeState>(
                robot_name_, "amp_control_gun_shoot_walk_mode", data_ptr,
                std::move(amp_control_gun_shoot_walk_policy),
                RobotMotionState::AmpControl_GunShootWalk_Mode);

        squat_preparation_controller_ = std::make_shared<SquatPreparationState>(robot_name_, "squat_preparation", data_ptr);
        mimic_control_chair_down_mode_controller_ = std::make_shared<MimicControlChairDownModeState>(
            robot_name_, "mimic_control_chair_down_mode", data_ptr);
        mimic_control_chair_up_mode_controller_ = std::make_shared<MimicControlChairUpModeState>(
            robot_name_, "mimic_control_chair_up_mode", data_ptr);
        amp_control_walk2run_mode_controller_ = std::make_shared<AmpControlWalk2RunModeState>(
            robot_name_, "amp_control_walk2run_mode", data_ptr);
        mjlab_control_walk2run_mode_controller_ = std::make_shared<MjlabControlWalk2RunModeState>(
            robot_name_, "mjlab_control_walk2run_mode", data_ptr);
        rl_control_stand_mode_controller_ = std::make_shared<RLControlStandModeState>(
            robot_name_, "rl_control_stand_mode", data_ptr);
        mimic_control_kneel_mode_controller_ = std::make_shared<MimicControlKneelModeState>(
            robot_name_, "mimic_control_kneel_mode", data_ptr);
        mimic_control_kneel_stand_up_mode_controller_ = std::make_shared<MimicControlKneelStandUpModeState>(
            robot_name_, "mimic_control_kneel_stand_up_mode", data_ptr);
    }

    std::shared_ptr<StateBase> amp_control_ni_ren_walk_mode_controller_;
    std::shared_ptr<StateBase> amp_control_gun_shoot_walk_mode_controller_;
    std::shared_ptr<StateBase> squat_preparation_controller_;
    std::shared_ptr<StateBase> amp_control_walk2run_mode_controller_;
    std::shared_ptr<StateBase> mjlab_control_walk2run_mode_controller_;
    std::shared_ptr<StateBase> rl_control_stand_mode_controller_;
    std::shared_ptr<StateBase> mimic_control_kneel_mode_controller_;
    std::shared_ptr<StateBase> mimic_control_kneel_stand_up_mode_controller_;
    std::shared_ptr<StateBase> mimic_control_chair_down_mode_controller_, mimic_control_chair_up_mode_controller_;
};

}  // namespace humanoid_std

#endif
