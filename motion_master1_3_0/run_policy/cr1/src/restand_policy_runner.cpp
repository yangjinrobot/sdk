#include "restand_policy_runner.h"

ReStandPolicyRunner::ReStandPolicyRunner(const std::string& policy_name,
                                         RobotName robot_name,
                                         const std::string& onnx_path)
    : CommonPolicyRunner(policy_name, robot_name, onnx_path) {
    // env(1).yaml actuator gains, in the 21-D policy joint order.
    VecXf kp(21), kd(21);
    kp << 300, 300, 200, 300, 300, 100, 100, 200, 200, 100, 100,
          300, 300, 100, 100, 80, 80, 100, 100, 30, 30;
    kd << 10, 10, 5, 10, 10, 5, 5, 5, 5, 5, 5,
          10, 10, 5, 5, 3, 3, 5, 5, 1, 1;
    SetPdGains(kp, kd);
}
