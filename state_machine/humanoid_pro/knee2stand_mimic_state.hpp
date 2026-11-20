#ifndef HUMANOID_PRO_KNEE2STAND_MIMIC_STATE_HPP_
#define HUMANOID_PRO_KNEE2STAND_MIMIC_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "knee_mimic_policy_runner.h"

namespace humanoid_pro {
class Knee2StandMimicState : public CopiedPolicyControlState {
public:
    Knee2StandMimicState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr,
              std::make_shared<Knee2StandPolicyRunner>("knee2stand", robot_name),
              Knee2StandMimicMode, {AmpWalkingControl}) {}
};
}  // namespace humanoid_pro

#endif
