#ifndef RUN_POLICY_RUNNER_H_
#define RUN_POLICY_RUNNER_H_

#include "policy_runner_base.hpp"

#include <string>
#include <vector>

// Forward command shaping in physical units, independent of ONNX inference.
// This shapes commanded velocity, not measured base velocity.
class RunDeceleration {
public:
    static constexpr double kTriggerSpeed = 1.0;  // m/s, strictly greater
    static constexpr double kMaxAcceleration = 1.0;  // m/s^2 while active
    static constexpr double kMaxJerk = 3.0;  // m/s^3

    void Reset() { active_ = false; acceleration_ = 0.0; }
    bool active() const { return active_; }
    double acceleration() const { return acceleration_; }

    double Update(double current, double target, double legacy_next, double dt);

private:
    bool active_ = false;
    double acceleration_ = 0.0;
};

// Default amp_dt: 78*(5+1) observations, 21 actions.
// 1022 models require an explicit 72-D observation configuration.
// 1023, 1026, 1027, 1101, 1102, 1103, 1103_2, 1106, 1111 and 1112 run use {73, 5, true},
// including a movement flag and no phase.
class RunPolicyRunner final : public PolicyRunnerBase {
public:
    // 1026 run: 73-D observations, with the parameters from 16500.yaml.
    enum class Variant {
        Run1026, Run1027, Run1101, Run1102, Run1103, Run1103_2, Run1106, Run1111,
        Run1112
    };
    RunPolicyRunner(const std::string& name, RobotName robot_name,
                    const std::string& path, Variant variant);

    RunPolicyRunner(const std::string& name, RobotName robot_name,
                    const std::string& path,
                    PolicyObservationConfig observation_config = {78, 5, true});

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
    RunDeceleration deceleration_;
    bool bypass_deceleration_ = false;
};

#endif
