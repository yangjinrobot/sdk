/**
 * @file common_policy_runner.hpp
 * @brief common policy runner
 * @author deeprobotics
 * @version 1.0
 * @date 2025-09-16
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */

#pragma once

#include "policy_runner_base.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <unordered_map>
#include <deque>

class HumanoidAmpPolicyRunner : public PolicyRunnerBase {
private:
    // Control parameters
    VecXf kp_, kd_;
    VecXf dof_pos_default_;
    Vec3f max_cmd_vel_;
    float action_scale_;
    float omega_scale_;
    float dof_vel_scale_;
    Vec3f cmd_vel_scale_;
    std::string policy_path_;

    // Observation/action buffers
    int obs_dim_, obs_history_num_, act_dim_;
    int obs_total_dim_;
    VecXf current_observation_;
    std::deque<VecXf> obs_history_;          // 存储历史观测（每个元素是 obs_dim_ 维向量）
    VecXf stacked_observation_;              // 堆叠后的总观测向量（维度 obs_total_dim_）
    VecXf action_, last_action_;
    VecXf obs_joint_pos_, obs_joint_vel_;
    float total_elapsed_time_ = 0.0f;

    float arm_interp_duration_s_ = 2.4f;
    float arm_interp_start_time_s_ = 0.0f;
    bool arm_interp_active_ = false;
    bool last_arm_preset_1_ = false;
    bool last_arm_preset_2_ = false;
    bool last_arm_preset_3_ = false;
    bool last_arm_preset_4_ = false;
    bool last_arm_preset_5_ = false;
    bool last_arm_preset_6_ = false;
    uint32_t last_arm_preset_seq_ = 0;
    Vec7f arm_interp_start_left_;
    Vec7f arm_interp_start_right_;
    Vec7f arm_interp_goal_left_;
    Vec7f arm_interp_goal_right_;
    Vec7f current_arm_target_left_;
    Vec7f current_arm_target_right_;
    Vec7f initial_arm_target_left_;
    Vec7f initial_arm_target_right_;
    bool has_initial_arm_targets_ = false;

    // Joint order mappings
    std::vector<int> robot_to_policy_idx_;
    std::vector<int> policy_to_output_idx_;
    std::vector<int> policy_to_arm_idx_;

    // ONNX runtime resources
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    bool policy_loaded_ = false;

    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    static constexpr float kSignalDt = 0.001f;
    

    static float SmoothStep(float t);

    static Vec7f SmoothInterpolate(const Vec7f& start, const Vec7f& end, float t);

    void UpdateArmPreset(const UserCommand& uc);

    void UpdateArmTargets(const RobotBasicState& ro, const UserCommand& uc);

public:
    HumanoidAmpPolicyRunner(const std::string& policy_name, RobotName robot_name);

    ~HumanoidAmpPolicyRunner() override;

    void UpdateObservationHistory();

    void DisplayPolicyInfo() override;

    bool IsReady() const override;

    void SetInitialArmTargets(const Vec7f& left, const Vec7f& right);

    void OnEnter() override;

    void OnExit() override;

    RobotAction ComputeRobotAction(const RobotBasicState &ro, const UserCommand &uc) override;

    void SetCmdMaxVel(const Vec3f& vel);

    static std::vector<int> GeneratePermutation(
        const std::vector<std::string>& from,
        const std::vector<std::string>& to,
        int default_index = 0);
};
