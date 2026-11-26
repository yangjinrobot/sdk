#ifndef MJRUN_POLICY_RUNNER_H_
#define MJRUN_POLICY_RUNNER_H_

#include "policy_runner_base.hpp"

#include <array>
#include <string>
#include <vector>

class MjlabControlWalk2RunModePolicyRunner final : public PolicyRunnerBase {
public:
    static constexpr int kJoints = 21;
    static constexpr int kFrameSize = 72;
    static constexpr int kHistoryLength = 5;
    static constexpr int kInputSize = kFrameSize * kHistoryLength;
    static constexpr double kPolicySeconds = .02;
    static constexpr const char* kDefaultPolicyPath =
        "/opt/robot/share/motion_master/policy/amp/mjlab/model_19999.onnx";
    using JointVector = Eigen::Matrix<float, kJoints, 1>;

    explicit MjlabControlWalk2RunModePolicyRunner(RobotName robot, const std::string& path = kDefaultPolicyPath);

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override;
    RobotAction GetRobotAction(const RobotBasicState& state, const UserCommand& command) override;
    const Vec3f& CommandVelocity() const { return cmd_vel_input_; }

    // Policy action offset and PD gains, ordered like kPolicyJointOrder.
    VecXf dof_pos_default_, kp_, kd_;
    std::vector<int> robot2policy_idx;
    std::vector<int> policyandpd2robot_idx;
    const int num_dof_ = 21;
    const int act_dim_ = 21;
    Vec3f max_cmd_vel_ = Vec3f(3.0f, 0.6f, 1.0f);
    float action_scale_ = 0.25f;
    float observation_clip_ = 100.0f;
    float dof_vel_scale_ = 0.05f;
    float forward_acceleration_ = 0.8f;

private:
    void UpdateCommandVelocity(const Vec3f& axes);

    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "mjlab_control_walk2run_mode"};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::string onnx_path_;
    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};
    JointVector previous_action_ = JointVector::Zero();
    std::array<float, kInputSize> history_{};
    bool seed_history_ = true;
};

#endif
