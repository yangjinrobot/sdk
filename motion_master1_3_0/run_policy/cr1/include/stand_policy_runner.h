/** Runner for the 21-DOF HoST ground stand-up policy. */
#ifndef STAND_POLICY_RUNNER_HPP_
#define STAND_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"

class StandPolicyRunner : public PolicyRunnerBase {
public:
    StandPolicyRunner(const std::string& policy_name, RobotName robot_name,
                      const std::string& onnx_path);
    ~StandPolicyRunner() override = default;

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override {}
    RobotAction GetRobotAction(const RobotBasicState& ro, const UserCommand& uc) override;

private:
    std::vector<int> robot2policy_idx_;
    std::vector<int> policy2robot_idx_;
    VecXf kp_, kd_, q_policy_, dq_policy_, action_, last_action_;
    VecXf observation_history_, observation_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    std::string onnx_path_;

    const char* input_names_[1] = {"observations"};
    const char* output_names_[1] = {"actions"};

    static constexpr int kActionDim = 21;
    static constexpr int kObservationDim = 70;
    static constexpr int kHistoryFrames = 6;
    static constexpr int kUnactuatedSteps = 30;
    static constexpr float kActionScale = 0.3f;
};

#endif
