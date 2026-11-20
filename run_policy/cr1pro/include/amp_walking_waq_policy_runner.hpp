#pragma once

#include "policy_runner_base.hpp"
#include <algorithm>
#include <array>
#include <deque>

// walktorun/env(15).yaml + policy_1030.onnx: five 96-D frames, no phase or height scan.
// Keep the WAQ runner API/state ID; all 29 joints are controlled by walk-to-run.
class AmpWalkingWAQPolicyRunner : public PolicyRunnerBase {
    static constexpr int kFrame = 96, kHistory = 5, kInput = kFrame * kHistory, kActions = 29;
    // Isaac articulation order -> SDK waist/left arm/right arm/left leg/right leg.
    inline static constexpr std::array<int, kActions> kJoints{
        17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27,
        4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16};
    VecXf dof_pos_default_ = VecXf::Zero(29), kp_ = VecXf::Zero(29), kd_ = VecXf::Zero(29);
    VecXf last_action_ = VecXf::Zero(kActions), observation_total_ = VecXf::Zero(kInput);
    std::deque<VecXf> obs_history_deque;
    // Deployment limits: cap forward commands below the training maximum of 3 m/s.
    const Vec3f min_velocity_limit_{-.5f, -.5f, -1.57f};
    const Vec3f max_velocity_limit_{2.f, .5f, 1.57f};
    Vec3f min_cmd_vel_ = min_velocity_limit_, max_cmd_vel_ = max_velocity_limit_;

    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "amp_walk_to_run"};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    bool model_loaded_ = false;
    std::string onnx_path_;
    const char* input_names_[1]{"obs"};
    const char* output_names_[1]{"actions"};

    RobotAction ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) override;

public:
    AmpWalkingWAQPolicyRunner(const std::string& policy_name, RobotName robot_name);
    bool IsReady() const override;
    void OnEnter() override;
    void OnExit() override;
    void DisplayPolicyInfo() override;
    void SetDecimation(int value) override;
    void SetCmdMaxVel(const Vec3f& velocity);
    const VecXf& GetDefaultJointPosition() const;
};
