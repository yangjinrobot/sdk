#ifndef RL_STAND_POLICY_RUNNER_HPP_
#define RL_STAND_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"
#include <array>
#include <filesystem>

// STD adapter of RLStandPolicyRunner. The STD actor differs from the Pro actor:
// [left leg, right leg, waist, left arm, right arm], 66*5 input, 13 actions.
// Observation/history/phase semantics follow sim2sim_dr02_std_stand.py.
class RLStandPolicyRunner final : public PolicyRunnerBase {
    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "restand"};
    Ort::SessionOptions options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::filesystem::path path_;
    // Training observation index -> SDK index (waist, arms, legs).
    const std::array<int, 21> to_robot_{{9,10,11,12,13,14,15,16,17,18,19,20,0,1,2,3,4,5,6,7,8}};
    VecXf defaults_, kp_, kd_;
    // 独立的控制目标；defaults_ 保持训练值，用于观测归一化。
    float elbow_target_ = -0.4f;
    VecXf observation_ = VecXf::Zero(66), history_ = VecXf::Zero(330);
    VecXf action_ = VecXf::Zero(13);
    double phase_time_ = 0;
    double policy_dt_, cycle_;
    float action_scale_, action_clip_, observation_clip_;
    float omega_scale_, dof_pos_scale_, dof_vel_scale_, command_scale_;
    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    void ValidateModel();

public:
    enum class ArmPosture { Lowered, Raised };
    void SetArmPosture(ArmPosture posture) {
        elbow_target_ = posture == ArmPosture::Lowered ? 1.2f : -0.4f;
    }

    static std::filesystem::path DefaultPolicyPath();

    explicit RLStandPolicyRunner(RobotName robot = RobotName::CR1STANDARD,
                                 const std::filesystem::path& path = DefaultPolicyPath());

    void OnEnter() override;
    void OnExit() override {}
    void DisplayPolicyInfo() override;
    const VecXf& Observation() const { return observation_; }
    const VecXf& History() const { return history_; }
    const VecXf& Action() const { return action_; }

    RobotAction GetRobotAction(const RobotBasicState& state, const UserCommand& user) override;
};

#endif // RL_STAND_POLICY_RUNNER_HPP_
