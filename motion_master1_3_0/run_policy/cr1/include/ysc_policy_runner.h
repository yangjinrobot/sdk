#ifndef YSC_POLICY_RUNNER_HPP_
#define YSC_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"

// Dedicated runner for policy_CR1_8.onnx.  Its 432-D `state` input is six
// consecutive 72-D observations, unlike the 468-D WAQ common policy input.
class YscPolicyRunner : public PolicyRunnerBase {
public:
    YscPolicyRunner(const std::string& policy_name, RobotName robot_name, const std::string& onnx_path);
    ~YscPolicyRunner() override;

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override;
    RobotAction GetRobotAction(const RobotBasicState& ro, const UserCommand& uc) override;

private:
    std::vector<int> GeneratePermutation(const std::vector<std::string>& from,
                                         const std::vector<std::string>& to) const;

    VecXf kp_, kd_, dof_pos_default_;
    Vec3f max_cmd_vel_, cmd_vel_scale_;
    VecXf observation_, observation_history_, model_input_, action_, last_action_;
    VecXf joint_pos_, joint_vel_;
    std::vector<int> robot_to_policy_, policy_to_robot_, policy_to_arm_;
    float action_scale_ = 0.25f;
    float omega_scale_ = 0.25f;
    float dof_vel_scale_ = 0.05f;
    int obs_dim_ = 72;
    int history_frames_ = 6;
    int action_dim_ = 21;

    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    std::string onnx_path_;
    const char* input_names_[1] = {"state"};
    const char* output_names_[1] = {"output"};
};

#endif
