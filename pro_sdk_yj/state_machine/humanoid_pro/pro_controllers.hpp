#ifndef HUMANOID_PRO_CONTROLLERS_HPP_
#define HUMANOID_PRO_CONTROLLERS_HPP_

#include "state_base.hpp"
#include "humanoid_pro/idle_state.hpp"
#include "humanoid_pro/joint_damping_state.hpp"
#include "humanoid_pro/zeropos_state.hpp"
#include "humanoid_pro/rl_stand_control_state.hpp"
#include "humanoid_pro/rl_stand_salute_state.hpp"
#include "humanoid_pro/rl_stand_salute_left_arm_state.hpp"
#include "humanoid_pro/rl_stand_please_state.hpp"
#include "humanoid_pro/rl_stand_please2_state.hpp"
#include "humanoid_pro/rl_stand_talking_state.hpp"
#include "humanoid_pro/upper_body_control_state.hpp"
#include "humanoid_pro/amp_zeropos_state.hpp"
#include "humanoid_pro/amp_whole_body_control_state.hpp"
#include "humanoid_pro/amp_walking_zeropos_state.hpp"
#include "humanoid_pro/amp_walking_state.hpp"
#include "humanoid_pro/rl_control_state.hpp"
#include "humanoid_pro/rl_mimic12_control_state.hpp"
#include "humanoid_pro/rl_mimic13_control_state.hpp"
#include "humanoid_pro/squat_preparation_state.hpp"
#include "humanoid_pro/rl_mimic2_control_state.hpp"
#include <stdexcept>

namespace humanoid_pro {
class ProControllers {
private:
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> zeropos_controller_;
    std::shared_ptr<StateBase> rl_stand_controller_;
    std::shared_ptr<StateBase> rl_stand_salute_controller_;
    std::shared_ptr<StateBase> rl_stand_salute_left_arm_controller_;
    std::shared_ptr<StateBase> rl_stand_please_controller_;
    std::shared_ptr<StateBase> rl_stand_talking_controller_;
    std::shared_ptr<StateBase> rl_stand_please2_controller_;
    std::shared_ptr<StateBase> upper_body_controller_;
    std::shared_ptr<StateBase> amp_zeropos_controller_;
    std::shared_ptr<StateBase> squat_preparation_controller_;
    std::shared_ptr<StateBase> amp_whole_body_controller_;
    std::shared_ptr<StateBase> amp_walking_zeropos_controller_;
    std::shared_ptr<StateBase> amp_walking_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> rl_mimic12_controller_;
    std::shared_ptr<StateBase> rl_mimic13_controller_;
    std::shared_ptr<StateBase> rl_mimic2_controller_;

public:
    ProControllers(const RobotName& robot_name, const std::shared_ptr<ControllerData>& data_ptr) {
        if (robot_name != RobotName::CR1PRO || !data_ptr || !data_ptr->cp_ptr || data_ptr->cp_ptr->dof_num_ != 31)
            throw std::invalid_argument("The imported PRO state machine requires CR1PRO / 31 DOF");

        idle_controller_ = std::make_shared<IdleState>(robot_name, "idle_state", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name, "joint_damping_state", data_ptr);
        zeropos_controller_ = std::make_shared<ZeroPosState>(robot_name, "zeropos_state", data_ptr);
        rl_stand_controller_ = std::make_shared<RLStandControlState>(robot_name, "rl_stand_control_state", data_ptr);
        auto salute_policy = std::make_shared<RLStandSalutePolicyRunner>("rl_stand_salute", robot_name);
        rl_stand_salute_controller_ = std::make_shared<RLStandSaluteState>(robot_name, "rl_stand_salute_state", data_ptr, salute_policy);
        rl_stand_salute_left_arm_controller_ = std::make_shared<RLStandSaluteLeftArmState>(
            robot_name, "rl_stand_salute_left_arm_state", data_ptr, salute_policy);
        auto please_policy = std::make_shared<RLStandPleasePolicyRunner>("rl_stand_please", robot_name);
        rl_stand_please_controller_ = std::make_shared<RLStandPleaseState>(robot_name, "rl_stand_please_state", data_ptr, please_policy);
        auto talking_policy = std::make_shared<RLStandTalkingPolicyRunner>("rl_stand_talking", robot_name);
        rl_stand_talking_controller_ = std::make_shared<RLStandTalkingState>(robot_name, "rl_stand_talking_state", data_ptr, talking_policy);
        auto please2_policy = std::make_shared<RLStandPlease2PolicyRunner>("rl_stand_please2", robot_name);
        rl_stand_please2_controller_ = std::make_shared<RLStandPlease2State>(robot_name, "rl_stand_please2_state", data_ptr, please2_policy);
        upper_body_controller_ = std::make_shared<UpperBodyControlState>(robot_name, "upper_body_control_state", data_ptr);
        amp_zeropos_controller_ = std::make_shared<AmpZeroPosState>(robot_name, "amp_zeropos_state", data_ptr);
        squat_preparation_controller_ = std::make_shared<SquatPreparationState>(robot_name, "squat_preparation_state", data_ptr);
        amp_whole_body_controller_ = std::make_shared<AmpWholeBodyControlState>(robot_name, "amp_whole_body_control_state", data_ptr);
        amp_walking_zeropos_controller_ = std::make_shared<AmpWalkingZeroPosState>(robot_name, "amp_walking_zeropos_state", data_ptr);
        amp_walking_controller_ = std::make_shared<AmpWalkingState>(robot_name, "amp_walking_state", data_ptr);
        rl_controller_ = std::make_shared<RLControlState>(robot_name, "rl_control_state", data_ptr);
        rl_mimic12_controller_ = std::make_shared<RLMimic12ControlState>(robot_name, "rl_mimic12_control_state", data_ptr);
        rl_mimic13_controller_ = std::make_shared<RLMimic13ControlState>(robot_name, "rl_mimic13_control_state", data_ptr);
        rl_mimic2_controller_ = std::make_shared<RLMimic2ControlState>(robot_name, "rl_mimic2_control_state", data_ptr);
    }

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) const {
        switch (state_name) {
            case StateName::kIdle:
                return idle_controller_;
            case StateName::kJointDamping:
                return joint_damping_controller_;
            case StateName::kZeroPos:
                return zeropos_controller_;
            case StateName::kRLStandControl:
                return rl_stand_controller_;
            case StateName::kRLStandSalute:
                return rl_stand_salute_controller_;
            case StateName::kRLStandSaluteLeftArm:
                return rl_stand_salute_left_arm_controller_;
            case StateName::kRLStandPlease:
                return rl_stand_please_controller_;
            case StateName::kRLStandTalking:
                return rl_stand_talking_controller_;
            case StateName::kRLStandPlease2:
                return rl_stand_please2_controller_;
            case StateName::kUpperBodyControl:
                return upper_body_controller_;
            case StateName::kAmpZeroPos:
                return amp_zeropos_controller_;
            case StateName::kSquatPreparation:
                return squat_preparation_controller_;
            case StateName::kAmpWholeBodyControl:
                return amp_whole_body_controller_;
            case StateName::kAmpWalkingZeroPos:
                return amp_walking_zeropos_controller_;
            case StateName::kAmpWalkingControl:
                return amp_walking_controller_;
            case StateName::kRLControl:
                return rl_controller_;
            case StateName::kRLMimic12Control:
                return rl_mimic12_controller_;
            case StateName::kRLMimic13Control:
                return rl_mimic13_controller_;
            case StateName::kRLMimic2Control:
                return rl_mimic2_controller_;
            case StateName::kInvalid:
            default:
                return nullptr;
        }
    }
};
}  // namespace humanoid_pro

#endif
