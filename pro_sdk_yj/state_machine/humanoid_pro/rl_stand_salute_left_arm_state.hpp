#ifndef HUMANOID_PRO_RL_STAND_SALUTE_LEFT_ARM_STATE_HPP_
#define HUMANOID_PRO_RL_STAND_SALUTE_LEFT_ARM_STATE_HPP_

#include "humanoid_pro/rl_stand_salute_state.hpp"

namespace humanoid_pro {
class RLStandSaluteLeftArmState : public RLStandSaluteState {
public:
    RLStandSaluteLeftArmState(const RobotName& robot_name, const std::string& state_name,
                             std::shared_ptr<ControllerData> data_ptr,
                             std::shared_ptr<RLStandSalutePolicyRunner> policy_ptr)
        : RLStandSaluteState(robot_name, state_name, std::move(data_ptr),
                            std::move(policy_ptr), RLStandSaluteLeftArmMode) {}
};
}  // namespace humanoid_pro

#endif
