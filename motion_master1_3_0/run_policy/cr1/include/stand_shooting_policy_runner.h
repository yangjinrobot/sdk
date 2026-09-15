#ifndef STAND_SHOOTING_POLICY_RUNNER_HPP_
#define STAND_SHOOTING_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"

class StandShootingPolicyRunner : public PolicyRunnerBase {
public:
    StandShootingPolicyRunner(const std::string& policy_name, RobotName robot_name, const std::string& onnx_path);
    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override {}
    RobotAction GetRobotAction(const RobotBasicState& ro, const UserCommand& uc) override;

private:
    static constexpr int kDof = 21, kActionDim = 13, kObsDim = 66, kHistoryFrames = 6;
    static constexpr float kActionScale = 0.25f;
    std::vector<int> policy_to_robot_{9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 0};
    VecXf default_q_, kp_, kd_, action_, last_action_, observation_, observation_history_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    std::string onnx_path_;
    const char* input_names_[1] = {"observations"};
    const char* output_names_[1] = {"actions"};
};

#endif
