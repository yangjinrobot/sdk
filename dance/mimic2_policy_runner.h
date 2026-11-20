/**
 * @file mimic2_policy_runner.h
 * @brief mimic2 policy runner for CR1
 * @author qiuxuejian
 * @version 1.0
 * @date 2026-10-06
 * 
 * @copyright Copyright (c) 2026  CETC21
 * 
 */

#ifndef MIMIC2_POLICY_RUNNER_HPP_
#define MIMIC2_POLICY_RUNNER_HPP_
#define PI 3.14159264558979323846

#include "policy_runner_base.hpp"

class Mimic2PolicyRunner : public PolicyRunnerBase
{
protected:

public:
    VecXf kp_, kd_;
    VecXf dof_pos_default_;
    VecXf dof_pos_default_full_;
    Vec3f max_cmd_vel_;

    int num_dof_;

    float action_scale_;
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

    int obs_dim_, obs_history_num_, act_dim_, time_step_dim_;
    int obs_total_dim_;
    int motion_command_dim_;
    VecXf current_observation_, observation_history_, observation_total_;
    VecXf action_, last_action_, action_policy_;
    VecXf obs_joint_pos_, obs_joint_vel_;
    VecXf motion_command;

    std::vector<int> robot2policy_idx;
    std::vector<int> policyandpd2robot_idx;
    std::vector<int> policy2arm_idx;

    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    std::string onnx_path_;

    const char* input_names_[2] = {"obs","time_step"}; // must keep the same as model export 
    const char* output_names_[7] =  {"actions","joint_pos","joint_vel","body_pos_w","body_quat_w","body_lin_vel_w","body_ang_vel_w"}; 

    Eigen::MatrixXf observation_input_;
    //qxj pro add 
    std::deque<Eigen::VectorXf> obs_history_deque;

public:
    Mimic2PolicyRunner(const std::string& policy_name, RobotName robot_name, const std::string& onnx_path);
    ~Mimic2PolicyRunner();
    
    void DisplayPolicyInfo() override;

    void OnEnter() override;
    void OnExit() override;

    RobotAction GetRobotAction(const RobotBasicState &ro, const UserCommand &uc) override;

    // void SetDefaultJointPos();
    void SetCmdMaxVel(const Vec3f& vel);
    Vec3f GetCmdMaxVel();
    void SetSkipArmTransition(bool skip) override { skip_arm_transition_ = skip; }
    std::vector<int> generate_permutation(const std::vector<std::string>& from, const std::vector<std::string>& to, int default_index = 0);

protected:
    bool skip_arm_transition_ = false;
};

#endif