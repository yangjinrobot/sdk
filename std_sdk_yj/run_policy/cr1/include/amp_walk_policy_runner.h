#ifndef AMP_WALK_POLICY_RUNNER_H_
#define AMP_WALK_POLICY_RUNNER_H_

#include "policy_runner_base.hpp"

#include <string>
#include <vector>

// Dedicated runner for the AMP walk policy used by RLControlMode. Its defaults come from the
// training articulation configuration and are intentionally independent from
// the legacy std_amp_2 ChiqiangWalk configuration.
class AmpWalkPolicyRunner final : public PolicyRunnerBase {
public:
    AmpWalkPolicyRunner(const std::string& name, RobotName robot_name,
                       const std::string& path);

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override;
    RobotAction GetRobotAction(const RobotBasicState& state, const UserCommand& command) override;

    void SetCmdMaxVel(const Vec3f& vel);
    Vec3f GetCmdMaxVel() const { return max_cmd_vel_; }
    void SetPdGains(const VecXf& kp, const VecXf& kd) override;

    VecXf kp_, kd_, dof_pos_default_;
    Vec3f max_cmd_vel_;
    const int num_dof_ = 21;
    const int act_dim_ = 21;
    int obs_dim_, obs_history_num_, obs_total_dim_;
    float action_scale_ = 0.25f;
    float action_clip_ = 10.0f;
    float observation_clip_ = 100.0f;
    float reverse_velocity_scale_ = 0.5f;
    float moving_flag_threshold_ = 0.15f;
    bool phase_active_at_threshold_ = false;
    float omega_scale_ = 1.0f;
    float dof_vel_scale_ = 0.05f;
    VecXf current_observation_, observation_history_, observation_total_;
    VecXf action_, last_action_;
    std::vector<int> robot2policy_idx;
    std::vector<int> policyandpd2robot_idx;

private:
    void UpdateCommandVelocity(const Vec3f& target);

    bool seed_history_with_first_frame_;
    bool history_needs_seed_ = false;
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    std::string onnx_path_;
    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};
};

#endif
