/**
 * @file amp_policy_runner.h
 * @brief amp policy runner for CR1
 * @author qiuxuejian
 * @version 1.0
 * @date 2026-10-06
 * 
 * @copyright Copyright (c) 2026  CETC21
 * 
 */

#ifndef AMP_POLICY_RUNNER_HPP_
#define AMP_POLICY_RUNNER_HPP_


#include "policy_runner_base.hpp"
#include "copied_policy_validation.hpp"
#include <deque>
#include <unordered_map>

class AmpPolicyRunner : public PolicyRunnerBase
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

    int obs_dim_, obs_history_num_, act_dim_;
    int obs_total_dim_;
    VecXf current_observation_, observation_history_, observation_total_;
    VecXf action_, last_action_;
    VecXf obs_joint_pos_, obs_joint_vel_;

    std::vector<int> robot2policy_idx;
    std::vector<int> policyandpd2robot_idx;
    std::vector<int> policy2arm_idx;

    Ort::Env env_{nullptr};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};

    std::vector<Ort::Value> ort_inputs_;
    std::string onnx_path_;

    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    Eigen::MatrixXf observation_input_;
    //qxj pro add 
    std::deque<Eigen::VectorXf> obs_history_deque;
    //qxj gait
    int   gait_count_ = 0;
    float gait_dt_ = 0.02f;
    float gait_cycle_ = 0.9f;
    float gait_air_ratio_l_ = 0.35f;
    float gait_air_ratio_r_ = 0.35f;
    float gait_phase_offset_l_ = 0.35f;
    float gait_phase_offset_r_ = 0.85f;


public:
    AmpPolicyRunner(const std::string& policy_name, RobotName robot_name, const std::string& onnx_path = "");
    ~AmpPolicyRunner();
    
    void DisplayPolicyInfo() override;

    void OnEnter() override;
    void OnExit() override;

    bool IsReady() const override { return model_loaded_ && !HasInferenceFault(); }

    // void SetDefaultJointPos();
    void SetCmdMaxVel(const Vec3f& vel);
    Vec3f GetCmdMaxVel();
    void SetSkipArmTransition(bool skip) { skip_arm_transition_ = skip; }
    std::vector<int> generate_permutation(const std::vector<std::string>& from, const std::vector<std::string>& to, int default_index = 0);

protected:
    RobotAction ComputeRobotAction(const RobotBasicState&, const UserCommand&) override;
    void LoadModel();
    bool model_loaded_ = false;
    bool skip_arm_transition_ = false;
};

#endif