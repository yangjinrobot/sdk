#include "run_policy_runner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {
std::vector<int> GeneratePermutation(const std::vector<std::string>& from,
                                     const std::vector<std::string>& to) {
    std::vector<int> indices;
    indices.reserve(to.size());
    for (const auto& name : to) {
        const auto match = std::find(from.begin(), from.end(), name);
        if (match == from.end()) {
            throw std::logic_error("Unknown policy joint: " + name);
        }
        indices.push_back(static_cast<int>(match - from.begin()));
    }
    return indices;
}
}  // namespace

double RunDeceleration::Update(double current, double target, double legacy_next, double dt) {
    if (!active_) {
        // Includes a reverse request while travelling forward (and vice versa).
        if (std::abs(current) <= kTriggerSpeed || current * (target - current) >= 0.0)
            return legacy_next;
        active_ = true;
        acceleration_ = 0.0;
    }

    const double error = target - current;
    const double acceleration_step = kMaxJerk * dt;
    if (std::abs(error) < 1e-7 && std::abs(acceleration_) <= acceleration_step) {
        Reset();
        return target;
    }

    // Exact discrete stopping distance, including this sample's a*dt.
    // With a reducing by jerk*dt each subsequent sample, find the largest
    // acceleration that can still taper to zero without passing a fixed target.
    const double n = std::floor((std::sqrt(1.0 + 8.0 * std::abs(error) /
                                                   (dt * acceleration_step)) - 1.0) / 2.0);
    const double stopping_acceleration =
        (std::abs(error) / dt + acceleration_step * n * (n + 1.0) / 2.0) / (n + 1.0);
    const double desired = std::copysign(std::min(kMaxAcceleration, stopping_acceleration), error);
    acceleration_ += std::clamp(desired - acceleration_, -acceleration_step, acceleration_step);
    // A moving target can temporarily be passed: preserve acceleration
    // continuity instead of snapping the velocity to the new target.
    return current + acceleration_ * dt;
}

RunPolicyRunner::RunPolicyRunner(const std::string& name, RobotName robot_name,
                                 const std::string& path, Variant variant)
    : RunPolicyRunner(name, robot_name, path,
                      PolicyObservationConfig{73, 5, true}) {
    SetCmdMaxVel(Vec3f(2.0f, 0.6f, 1.0f));
    reverse_velocity_scale_ = 1.0f / 2.0f; // X range [-1, 2] m/s.
    if (variant == Variant::Run1027 || variant == Variant::Run1101 ||
        variant == Variant::Run1102 || variant == Variant::Run1103 ||
        variant == Variant::Run1103_2 || variant == Variant::Run1106 ||
        variant == Variant::Run1111) {
        // 1027_run/run.yaml and sim2sim_dr02_1027_run.py.
        dof_pos_default_(9) = 0.25f;   // left_shoulder_roll_joint
        dof_pos_default_(10) = -0.25f; // right_shoulder_roll_joint
        moving_flag_threshold_ = 0.1f;
    }
    if (variant == Variant::Run1112) {
        // 1112/env.yaml (DR02_1): default pose and movement threshold.
        dof_pos_default_(0) = -0.1f;   // left_hip_pitch_joint
        dof_pos_default_(1) = -0.1f;   // right_hip_pitch_joint
        dof_pos_default_(9) = 0.15f;   // left_shoulder_roll_joint
        dof_pos_default_(10) = -0.15f; // right_shoulder_roll_joint
        dof_pos_default_(11) = 0.2f;   // left_knee_pitch_joint
        dof_pos_default_(12) = 0.2f;   // right_knee_pitch_joint
        dof_pos_default_(15) = -0.1f;  // left_ankle_pitch_joint
        dof_pos_default_(16) = -0.1f;  // right_ankle_pitch_joint
        moving_flag_threshold_ = 0.1f;
    }
    if (variant == Variant::Run1101 || variant == Variant::Run1102 ||
        variant == Variant::Run1103 || variant == Variant::Run1103_2 ||
        variant == Variant::Run1106 || variant == Variant::Run1111 ||
        variant == Variant::Run1112) {
        // 1101, 1102, 1103, 1103_2, 1106, 1111 and 1112 use the same deployment parameters.
        SetCmdMaxVel(Vec3f(3.0f, 0.6f, 1.0f));
        reverse_velocity_scale_ = 1.0f / 3.0f; // X range [-1, 3] m/s.
    }
}

RunPolicyRunner::RunPolicyRunner(const std::string& name, RobotName robot_name,
                                 const std::string& path,
                                 PolicyObservationConfig observation_config)
    : PolicyRunnerBase(name, robot_name), obs_dim_(observation_config.observation_dim),
      obs_history_num_(observation_config.history_frames),
      seed_history_with_first_frame_(observation_config.seed_history_with_first_frame),
      onnx_path_(path) {
    if (robot_name != RobotName::CR1STANDARD) {
        throw std::invalid_argument("run requires CR1STANDARD (21 DOF)");
    }

    // SDK order and model order for the 21 Standard joints.
    const std::vector<std::string> robot_order{
            "waist_z_joint",
            "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
            "left_elbow_joint",
            "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
            "right_elbow_joint",
            "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint",
            "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
            "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint",
            "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint"
    };
    const std::vector<std::string> policy_order{
            "left_hip_y_joint", "right_hip_y_joint",
            "waist_z_joint",
            "left_hip_x_joint", "right_hip_x_joint",
            "left_shoulder_y_joint", "right_shoulder_y_joint",
            "left_hip_z_joint", "right_hip_z_joint",
            "left_shoulder_x_joint", "right_shoulder_x_joint",
            "left_knee_joint", "right_knee_joint",
            "left_shoulder_z_joint", "right_shoulder_z_joint",
            "left_ankle_y_joint", "right_ankle_y_joint",
            "left_elbow_joint", "right_elbow_joint",
            "left_ankle_x_joint", "right_ankle_x_joint"
    };
    robot2policy_idx = GeneratePermutation(robot_order, policy_order);
    policyandpd2robot_idx = GeneratePermutation(policy_order, robot_order);

    // PD gains follow policy_order above.
    kp_.resize(act_dim_);
    kd_.resize(act_dim_);
    kp_ << 300, 300, 200, 300, 300, 100, 100, 200, 200,
           100, 100, 300, 300, 100, 100, 80, 80, 100, 100, 30, 30;
    kd_ << 10, 10, 5, 10, 10, 5, 5, 5, 5,
           5, 5, 10, 10, 5, 5, 3, 3, 5, 5, 1, 1;

    dof_pos_default_.resize(act_dim_);
    // Default pose for the original Run export in policy_order.
    dof_pos_default_ << -0.2f, -0.2f, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 0.0f, 0.15f, -0.15f,
                         0.4f, 0.4f, 0.0f, 0.0f, -0.2f, -0.2f,
                         -0.2f, -0.2f, 0.0f, 0.0f;
    if (obs_dim_ == 72) {
        // env_nirenxingzou_1020.yaml: elbows at +1.2 rad, arm kp=100.
        dof_pos_default_(17) = dof_pos_default_(18) = 1.2f;
        kp_(13) = kp_(14) = 100.0f;
    }
    if (obs_dim_ == 73) {
        // 1023 / 1026 run: both elbow defaults are +1.2 rad.
        dof_pos_default_(17) = dof_pos_default_(18) = 1.2f;
    }
    // Legacy amp_dt X range [-0.5, 1.5]; 1022 range [-0.5, 1.2].
    reverse_velocity_scale_ = 1.0f / 3.0f;
    SetCmdMaxVel(Vec3f(1.5f, 0.6f, 1.0f));
    if (obs_dim_ == 72) {
        reverse_velocity_scale_ = 0.5f / 1.2f;
        SetCmdMaxVel(Vec3f(1.2f, 0.5f, 1.0f));
    }
    // Only |vx|, |vy|, |yaw| all strictly below 0.1 suppress sin/cos.
    phase_active_at_threshold_ = true;

    decimation_ = 20;
    if ((obs_dim_ != 72 && obs_dim_ != 73 && obs_dim_ != 78) || obs_history_num_ < 1) {
        throw std::invalid_argument("Unsupported policy observation configuration");
    }
    obs_total_dim_ = obs_dim_ + obs_dim_ * obs_history_num_;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);

    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    try {
        // 检查 ONNX 文件是否存在
        std::ifstream f(onnx_path_.c_str());
        if (!f.good()) {
            std::cerr << "error: ONNX model file not found: " << onnx_path_ << std::endl;
            std::exit(1);
        }
        // 从文件加载 ONNX 模型
        session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
        const auto input_shape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto output_shape = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1 ||
            input_shape.size() != 2 || input_shape[1] != obs_total_dim_ ||
            output_shape.size() != 2 || output_shape[1] != act_dim_)
            throw std::runtime_error("Policy ONNX dimensions do not match observation/action configuration");
        std::cout << "loaded " << policy_name_ << " ONNX model from: "
                  << onnx_path_ << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "error loading policy as " << policy_name_ << "\n" << e.what() << std::endl;
        throw;
    }

    // Warm up the ONNX session before it is used by the control loop.
    const std::array<int64_t, 2> shape{1, obs_total_dim_};
    for (int i = 0; i < 10; ++i) {
        VecXf observation = VecXf::Random(obs_total_dim_);
        auto tensor = Ort::Value::CreateTensor<float>(memory_info_, observation.data(),
                                                     observation.size(), shape.data(), shape.size());
        session_.Run(Ort::RunOptions{nullptr}, input_names_, &tensor, 1, output_names_, 1);
    }
}

void RunPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " network test success" << std::endl;
    std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_
              << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

void RunPolicyRunner::OnEnter() {
    history_needs_seed_ = seed_history_with_first_frame_;
    action_.setZero(act_dim_);
    last_action_.setZero(act_dim_);

    current_observation_.setZero(obs_dim_);
    observation_history_.setZero(obs_dim_ * obs_history_num_);
    observation_total_.setZero(obs_total_dim_);

    run_cnt_ = 0;
    run_cnt_record_ = 0;
    cmd_vel_input_.setZero();
    deceleration_.Reset();
    bypass_deceleration_ = false;
}

void RunPolicyRunner::OnExit() {
    deceleration_.Reset();
}

void RunPolicyRunner::UpdateCommandVelocity(const Vec3f& target) {
    const double current = cmd_vel_input_(0) * max_cmd_vel_(0);
    Eigen::Vector3f vel_delta = target - cmd_vel_input_;
    vel_delta_const_ << 0.02, 0.02, 0.04;
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(vel_delta(i)) > vel_delta_const_(i)) vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
    }
    cmd_vel_input_ += vel_delta;
    if (bypass_deceleration_) return;
    // Control loop is 1 kHz; update only on policy inference ticks.
    const double dt = static_cast<double>(decimation_) * 0.001;
    const double next = deceleration_.Update(current, target(0) * max_cmd_vel_(0),
                                             cmd_vel_input_(0) * max_cmd_vel_(0), dt);
    cmd_vel_input_(0) = static_cast<float>(next / max_cmd_vel_(0));
}

RobotAction RunPolicyRunner::GetRobotAction(const RobotBasicState& ro,
                                            const UserCommand& command) {
    if (ro.joint_pos.size() != num_dof_ || ro.joint_vel.size() != num_dof_ ||
        !ro.joint_pos.allFinite() || !ro.joint_vel.allFinite() ||
        !ro.base_rot_mat.allFinite() || !ro.base_omega.allFinite() ||
        !std::isfinite(command.forward_vel_scale) ||
        !std::isfinite(command.side_vel_scale) ||
        !std::isfinite(command.turnning_vel_scale))
        throw std::runtime_error("run: invalid robot observation");
    UserCommand bounded = command;
    bounded.forward_vel_scale = std::clamp(command.forward_vel_scale, -1.0f, 1.0f);
    bounded.side_vel_scale = std::clamp(command.side_vel_scale, -1.0f, 1.0f);
    bounded.turnning_vel_scale = std::clamp(command.turnning_vel_scale, -1.0f, 1.0f);
    // RunState bypasses inference on a safety stop. Also reset here
    // for direct runner users; safety commands never enter the S curve.
    if (command.safe_control_mode != 0) deceleration_.Reset();
    bypass_deceleration_ = command.safe_control_mode != 0;

    Vec3f user_cmd_vel = Vec3f(bounded.forward_vel_scale,
                               bounded.side_vel_scale,
                               bounded.turnning_vel_scale);
    if (bounded.forward_vel_scale < 0) user_cmd_vel[0] = reverse_velocity_scale_ * bounded.forward_vel_scale;
    if (bounded.safe_control_mode != 0) {
        std::cout << "safe_control_mode: " << std::dec << bounded.safe_control_mode << std::endl;
        user_cmd_vel.setZero();
    }

    UpdateCommandVelocity(user_cmd_vel);
    const Vec3f cmd_vel = cmd_vel_input_.cwiseProduct(max_cmd_vel_).unaryExpr(&ApplyCommandDeadband);
    // Log the actual command vector fed to the policy, after all limiting and
    // scaling. Rate-limit stdout so inference is not disturbed by 50 Hz I/O.
    if (run_cnt_ % 50 == 0) {
        std::cout << "policy cmd_vel [m/s, m/s, rad/s]: " << cmd_vel.transpose() << std::endl;
    }

    // The policy uses a 20 ms step (1 kHz control / decimation 20). The first
    // policy observation has zero phase; thereafter phase follows training.
    float sin_phase_l = 0.0f;
    float sin_phase_r = 0.0f;
    float cos_phase_l = 0.0f;
    float cos_phase_r = 0.0f;
    if (obs_dim_ == 78 && run_cnt_ > 0) {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kPolicyDt = 0.020f;
        constexpr float kGaitCycle = 0.85f;
        constexpr float kLeftPhaseOffset = 0.38f;
        constexpr float kRightPhaseOffset = 0.88f;
        const float policy_time = static_cast<float>(run_cnt_) * kPolicyDt;
        const float phase_l = std::fmod(policy_time / kGaitCycle + kLeftPhaseOffset, 1.0f);
        const float phase_r = std::fmod(policy_time / kGaitCycle + kRightPhaseOffset, 1.0f);
        sin_phase_l = std::sin(2.0f * kPi * phase_l);
        sin_phase_r = std::sin(2.0f * kPi * phase_r);
        cos_phase_l = std::cos(2.0f * kPi * phase_l);
        cos_phase_r = std::cos(2.0f * kPi * phase_r);
    }
    const float command_magnitude = cmd_vel.cwiseAbs().maxCoeff();
    const float phase_mask = (phase_active_at_threshold_ ? command_magnitude >= 0.1f
                                                        : command_magnitude > 0.1f) ? 1.0f : 0.0f;

    current_observation_.setZero(obs_dim_);
    Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);

    current_observation_.head(9) << omega_scale_ * ro.base_omega,
            project_gravity, cmd_vel;
    // 1023 env: the movement flag follows the physical command, before joints.
    // 72-D and legacy 78-D policies retain their original joint offsets.
    const int joint_offset = obs_dim_ == 73 ? 10 : 9;
    if (obs_dim_ == 73)
        current_observation_(9) =
            (cmd_vel.head<2>().norm() > moving_flag_threshold_ ||
             std::fabs(cmd_vel(2)) > moving_flag_threshold_) ? 1.0f : 0.0f;
    for (int i = 0; i < act_dim_; ++i) {
        current_observation_(joint_offset + i) =
            ro.joint_pos(robot2policy_idx[i]) - dof_pos_default_(i);
        current_observation_(joint_offset + act_dim_ + i) =
            dof_vel_scale_ * ro.joint_vel(robot2policy_idx[i]);
    }
    current_observation_.segment(joint_offset + 2 * act_dim_, act_dim_) = last_action_;
    if (obs_dim_ == 78) {
        current_observation_.tail(6) << phase_mask * sin_phase_l, phase_mask * sin_phase_r,
            phase_mask * cos_phase_l, phase_mask * cos_phase_r,
            0.38f, 0.38f;
    }
    if (!current_observation_.allFinite())
        throw std::runtime_error("Non-finite policy observation");
    current_observation_ = current_observation_.cwiseMax(-observation_clip_).cwiseMin(observation_clip_);

    // The history option selects a live first-frame seed or an initial zero history.
    VecXf next_history = observation_history_;
    if (history_needs_seed_) {
        for (int i = 0; i < obs_history_num_; ++i)
            next_history.segment(i * obs_dim_, obs_dim_) = current_observation_;
    } else {
        next_history.head((obs_history_num_ - 1) * obs_dim_) =
            observation_history_.tail((obs_history_num_ - 1) * obs_dim_);
        next_history.tail(obs_dim_) = current_observation_;
    }

    // ONNX input layout: [history (oldest -> newest), current].
    VecXf next_input(obs_total_dim_);
    next_input << next_history, current_observation_;

    const std::array<int64_t, 2> shape{1, obs_total_dim_};
    auto tensor = Ort::Value::CreateTensor<float>(memory_info_, next_input.data(),
                                                 next_input.size(), shape.data(), shape.size());
    auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, input_names_, &tensor, 1,
                                       output_names_, 1);
    action_ = Eigen::Map<VecXf>(output_tensors[0].GetTensorMutableData<float>(), act_dim_);
    if (!action_.allFinite()) throw std::runtime_error("Non-finite policy action");
    action_ = action_.cwiseMax(-action_clip_).cwiseMin(action_clip_);

    // 推理成功后才提交缓存，失败不消耗首次填充标记或污染历史。
    observation_history_ = std::move(next_history);
    observation_total_ = std::move(next_input);
    history_needs_seed_ = false;
    last_action_ = action_;
    RobotAction result;
    result.goal_joint_pos = VecXf::Zero(act_dim_);
    result.goal_joint_vel = VecXf::Zero(act_dim_);
    result.tau_ff = VecXf::Zero(act_dim_);
    result.kp = VecXf::Zero(act_dim_);
    result.kd = VecXf::Zero(act_dim_);
    for (int i = 0; i < act_dim_; i++) {
        result.goal_joint_pos(i) =
                action_scale_ * action_(policyandpd2robot_idx[i]) + dof_pos_default_(policyandpd2robot_idx[i]);
        result.kp(i) = kp_(policyandpd2robot_idx[i]);
        result.kd(i) = kd_(policyandpd2robot_idx[i]);
    }

    ++run_cnt_;
    if (!action_.allFinite() || action_.cwiseAbs().maxCoeff() > 10.0f ||
        !result.goal_joint_pos.allFinite() || !result.kp.allFinite() ||
        !result.kd.allFinite())
        throw std::runtime_error("run: invalid or out-of-range policy action");
    return result;
}

void RunPolicyRunner::SetCmdMaxVel(const Vec3f &vel) {
    for (int i = 0; i < 3; ++i) {
        if (vel(i) < 0) {
            std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
        }
    }
    max_cmd_vel_ = vel;
}

void RunPolicyRunner::SetPdGains(const VecXf& kp, const VecXf& kd) {
    if (kp.size() != act_dim_ || kd.size() != act_dim_) {
        throw std::invalid_argument("PD gain size must match policy action dimension");
    }
    kp_ = kp;
    kd_ = kd;
}
