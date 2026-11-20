/**
 * @file common_policy_runner.h
 * @brief common policy runner for CR1
 * @author zhenglinqiang
 * @version 1.0
 * @date 2025-09-16
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef COMMON_POLICY_RUNNER_HPP_
#define COMMON_POLICY_RUNNER_HPP_
#define PI 3.14159264558979323846

#include "policy_runner_base.hpp"

class CommonPolicyRunner : public PolicyRunnerBase
{
protected:

public:
    VecXf kp_, kd_;
    VecXf dof_pos_default_;
    VecXf dof_pos_default_full_;
    Vec3f max_cmd_vel_;

    int num_dof_;

    float action_scale_;
    float action_clip_ = 100.0f;
    float observation_clip_ = 100.0f;
    float reverse_velocity_scale_ = 0.5f;
    float moving_flag_threshold_ = 0.15f;
    bool phase_active_at_threshold_ = false;

    // Input/output are normalized, including the asymmetric reverse scaling.
    // Override only where a policy needs its own command shaping.
    virtual void UpdateCommandVelocity(const Vec3f& target);
    float omega_scale_;
    float dof_vel_scale_;
    float gait_indices_;
    float payload_mass_scale_;
    float body_com_scale_;

    Vec3f cmd_vel_scale_;
    Vec3f cmd_pos_input_;
    float payload_mass_;
    Vec3f body_com_;
    float zero_cmd_threshold_xy, zero_cmd_threshold_z;

    int obs_dim_, obs_history_num_, act_dim_;
    int obs_total_dim_;
    VecXf current_observation_, observation_history_, observation_total_;
    VecXf action_, last_action_;
    VecXf obs_joint_pos_, obs_joint_vel_;

    std::vector<int> robot2policy_idx;
    std::vector<int> policyandpd2robot_idx;
    std::vector<int> policy2arm_idx;

    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    std::string onnx_path_;

    // std_terrain.onnx exported by the DR02_1 WAQ policy.
    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    Eigen::MatrixXf observation_input_;

public:
    CommonPolicyRunner(const std::string& policy_name, RobotName robot_name,
                       const std::string& onnx_path,
                       GraphOptimizationLevel graph_optimization_level = ORT_ENABLE_ALL,
                       PolicyObservationConfig observation_config = {78, 5});   // 单帧维度 历史长度
    ~CommonPolicyRunner();

    void DisplayPolicyInfo() override;

    void OnEnter() override;
    void OnExit() override;

    RobotAction GetRobotAction(const RobotBasicState &ro, const UserCommand &uc) override;

    // void SetDefaultJointPos();
    void SetCmdMaxVel(const Vec3f& vel);
    Vec3f GetCmdMaxVel();
    void SetPdGains(const VecXf& kp, const VecXf& kd) override;
    void SetSkipArmTransition(bool skip) override { skip_arm_transition_ = skip; }
    std::vector<int> generate_permutation(const std::vector<std::string>& from, const std::vector<std::string>& to, int default_index = 0);

protected:
    bool skip_arm_transition_ = false;
    bool seed_history_with_first_frame_ = false;
    bool history_needs_seed_ = false;
};

#endif
