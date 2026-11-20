#pragma once

#include "policy_runner_base.hpp"
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <unordered_map>
#include <deque>

class AmpWalkingPolicyRunner : public PolicyRunnerBase {
private:
    static constexpr float kPi_ = 3.14159265358979323846f;

    // Control parameters
    VecXf kp_, kd_;
    VecXf dof_pos_default_;
    const Vec3f trained_min_cmd_vel_{-0.5f, -0.6f, -1.0f};
    const Vec3f trained_max_cmd_vel_{1.2f, 0.6f, 1.0f};
    Vec3f min_cmd_vel_, max_cmd_vel_;
    float action_scale_;
    float omega_scale_;
    float dof_vel_scale_;
    float clip_observations_;
    float clip_actions_;
    float signal_dt_;
    float gait_cycle_;
    Vec3f cmd_vel_scale_;
    Eigen::Vector2f gait_air_ratio_;
    Eigen::Vector2f gait_phase_offset_;
    std::string policy_path_;

    // Observation/action buffers
    int obs_dim_, obs_history_num_, act_dim_;
    int obs_total_dim_;
    VecXf current_observation_;
    std::deque<VecXf> obs_history_;          // 5 帧历史，不包含当前帧
    VecXf stacked_observation_;              // [5 帧历史, 当前帧]，共 612 维
    VecXf action_, last_action_;
    VecXf obs_joint_pos_, obs_joint_vel_;

    // Joint order mappings
    std::vector<int> robot_to_policy_idx_;
    std::vector<int> policy_to_output_idx_;

    // ONNX runtime resources
    // Env must outlive the session (members are destroyed in reverse order).
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    std::vector<Ort::Value> ort_inputs_;
    bool policy_loaded_ = false;
    VecXf arm_release_start_ = VecXf::Zero(14);
    bool arm_release_pending_ = false, arm_release_active_ = false;
    double arm_release_elapsed_ = 0., arm_release_duration_ = 2.4;

    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    void ClipVector(VecXf& value, float limit);

public:
    AmpWalkingPolicyRunner(const std::string& policy_name, RobotName robot_name);

    ~AmpWalkingPolicyRunner() override;

    void UpdateObservationHistory();

    void DisplayPolicyInfo() override;

    bool IsReady() const override;

    void OnEnter() override;

    void OnExit() override;

    void SetArmReleaseFrom(const Vec7f& left, const Vec7f& right, double duration = 2.4);

    RobotAction ComputeRobotAction(const RobotBasicState &ro, const UserCommand &uc) override;

    void SetCmdMaxVel(const Vec3f& vel);

    void SetCmdVelLimits(const Vec3f& min_vel, const Vec3f& max_vel);

    static std::vector<int> GeneratePermutation(
        const std::vector<std::string>& from,
        const std::vector<std::string>& to);
};
