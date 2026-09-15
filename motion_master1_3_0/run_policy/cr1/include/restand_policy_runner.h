#ifndef RESTAND_POLICY_RUNNER_HPP_
#define RESTAND_POLICY_RUNNER_HPP_

#include "common_policy_runner.h"

// Dedicated runner for restand.onnx. Its observation and action layouts are
// compatible with CommonPolicyRunner, while its training PD gains are kept
// local to this policy.
class ReStandPolicyRunner : public CommonPolicyRunner {
public:
    ReStandPolicyRunner(const std::string& policy_name, RobotName robot_name,
                        const std::string& onnx_path);
    ~ReStandPolicyRunner() override = default;
};

#endif
