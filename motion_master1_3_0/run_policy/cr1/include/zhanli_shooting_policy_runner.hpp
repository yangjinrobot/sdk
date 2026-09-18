#ifndef ZHANLI_SHOOTING_POLICY_RUNNER_HPP_
#define ZHANLI_SHOOTING_POLICY_RUNNER_HPP_

#include "common_policy_runner.h"

// WAQ runner for policy_shoot_walk.onnx. Its [5 history, current] 468-D input
// and 21-D output follow sim2sim_dr02_21dof_waq(4).py.
// training default pose and PD gains differ from the common policy.
class ZhanliShootingPolicyRunner final : public CommonPolicyRunner {
public:
    ZhanliShootingPolicyRunner(const std::string& policy_name, RobotName robot_name,
                               const std::string& onnx_path);
};

#endif
