#include "humanoid_pro/pro_controllers.hpp"

#include <stdexcept>
#include <utility>
#include "humanoid/idle_state.hpp"
#include "humanoid/joint_damping_state.hpp"
#include "humanoid/zeropos_state.hpp"
#include "humanoid_pro/rl_stand_control_state.hpp"
#include "humanoid_pro/rl_stand_salute_state.hpp"
#include "humanoid_pro/rl_stand_salute_left_arm_state.hpp"
#include "rl_stand_salute_policy_runner.hpp"
#include "humanoid_pro/upper_body_control_state.hpp"
#include "humanoid_pro/amp_zeropos_state.hpp"
#include "humanoid_pro/amp_whole_body_control_state.hpp"
#include "humanoid_pro/amp_walking_zeropos_state.hpp"
#include "humanoid_pro/amp_walking_state.hpp"
#include "humanoid_pro/amp_moe_15dof_state.hpp"
#include "humanoid_pro/amp_walking_waq_state.hpp"

#include "humanoid_pro/rl_control_state.hpp"
#include "humanoid_pro/rl_mimic12_control_state.hpp"
#include "humanoid_pro/rl_mimic13_control_state.hpp"
#include "humanoid_pro/stand2knee_mimic_state.hpp"
#include "humanoid_pro/knee2stand_mimic_state.hpp"

namespace humanoid {
namespace {

template <typename Controller, typename... Args>
void RegisterEnabledProController(ProControllers& controllers, RobotMotionState state, RobotName robot_name,
                                  const char* name, const std::shared_ptr<ControllerData>& data_ptr,
                                  Args&&... args) {
    // Check before construction: disabled policies must not load their models.
    if (IsKnownRobotMotionState(state))
        controllers.emplace(
            static_cast<StateName>(state),
            std::make_shared<Controller>(robot_name, name, data_ptr, std::forward<Args>(args)...));
}

} // namespace

ProControllers MakeProControllers(RobotName robot_name, const std::shared_ptr<ControllerData>& data_ptr) {
    if (robot_name != RobotName::CR1PRO || !data_ptr || !data_ptr->cp_ptr || data_ptr->cp_ptr->dof_num_ != 31)
        throw std::invalid_argument("The imported PRO state machine requires CR1PRO / 31 DOF");
    ProControllers result;
    RegisterEnabledProController<IdleState>(result, Idle, robot_name, "idle_state", data_ptr);
    RegisterEnabledProController<JointDampingState>(result, JointDamping, robot_name, "joint_damping_state",
                                                    data_ptr);
    RegisterEnabledProController<ZeroPosState>(result, ZeroPos, robot_name, "zeropos_state", data_ptr);
    RegisterEnabledProController<RLStandControlState>(result, RLStandControlMode, robot_name,
                                                      "rl_stand_control_state", data_ptr);
    if (IsKnownRobotMotionState(RLStandSaluteMode) || IsKnownRobotMotionState(RLStandSaluteLeftArmMode)) {
        // Carry RL policy history across transitions between the two arm states.
        auto salute_policy = std::make_shared<RLStandSalutePolicyRunner>("rl_stand_salute", robot_name);
        RegisterEnabledProController<RLStandSaluteState>(result, RLStandSaluteMode, robot_name,
                                                         "rl_stand_salute_state", data_ptr, salute_policy);
        RegisterEnabledProController<RLStandSaluteLeftArmState>(result, RLStandSaluteLeftArmMode, robot_name,
                                                                "rl_stand_salute_left_arm_state", data_ptr,
                                                                salute_policy);
    }
    RegisterEnabledProController<UpperBodyControlState>(result, UpperBodyControlMode, robot_name,
                                                        "upper_body_control_state", data_ptr);
    RegisterEnabledProController<AmpZeroPosState>(result, AmpZeroPos, robot_name, "amp_zeropos_state",
                                                  data_ptr);
    RegisterEnabledProController<AmpWholeBodyControlState>(result, AmpWholeBodyControlMode, robot_name,
                                                           "amp_whole_body_control_state", data_ptr);
    RegisterEnabledProController<AmpWalkingZeroPosState>(result, AmpWalkingZeroPos, robot_name,
                                                         "amp_walking_zeropos_state", data_ptr);
    RegisterEnabledProController<AmpWalkingState>(result, AmpWalkingControl, robot_name, "amp_walking_state",
                                                  data_ptr);
    RegisterEnabledProController<AmpMoe15DofState>(result, AmpMoe15Dof, robot_name, "amp_moe_15dof_state",
                                                   data_ptr);
    RegisterEnabledProController<AmpWalkingWAQState>(result, AmpWalkingWAQ, robot_name,
                                                     "amp_walking_waq_state", data_ptr);
    RegisterEnabledProController<RLControlState>(result, RLControlMode, robot_name, "rl_control_state",
                                                 data_ptr);
    RegisterEnabledProController<RLMimic12ControlState>(result, RLControlMimic12Mode, robot_name,
                                                        "rl_mimic12_control_state", data_ptr);
    RegisterEnabledProController<RLMimic13ControlState>(result, RLControlMimic13Mode, robot_name,
                                                        "rl_mimic13_control_state", data_ptr);
    RegisterEnabledProController<Stand2KneeMimicState>(result, Stand2KneeMimicMode, robot_name,
                                                       "stand2knee_mimic_state", data_ptr);
    RegisterEnabledProController<Knee2StandMimicState>(result, Knee2StandMimicMode, robot_name,
                                                       "knee2stand_mimic_state", data_ptr);
    return result;
}
} // namespace humanoid
