#include "amp_walking_policy_runner.hpp"

void AmpWalkingPolicyRunner::ClipVector(VecXf& value, float limit) {
    value = value.cwiseMax(-limit).cwiseMin(limit);
}

AmpWalkingPolicyRunner::AmpWalkingPolicyRunner(const std::string& policy_name, RobotName robot_name) : PolicyRunnerBase(policy_name, robot_name) {
    kp_ = VecXf::Zero(29);
    kp_ << 300, 300,   // left/right hip pitch
           200,        // waist yaw
           300, 300,   // left/right hip roll
           200,        // waist roll
           200, 200,   // left/right hip yaw
           300,        // waist pitch
           300, 300,   // left/right knee pitch
           100, 100,   // left/right shoulder pitch
            80,  80,   // left/right ankle pitch
           100, 100,   // left/right shoulder roll
            30,  30,   // left/right ankle roll
           100, 100,   // left/right shoulder yaw
           100, 100,   // left/right elbow pitch
            30,  30,   // left/right wrist yaw
            30,  30,   // left/right wrist pitch
            30,  30;   // left/right wrist roll
    
    kd_ = VecXf::Zero(29);
    kd_ << 10, 10,      // left/right hip pitch
            5,          // waist yaw
           10, 10,      // left/right hip roll
           10,          // waist roll   old is 5
            5,  5,      // left/right hip yaw
           10,          // waist pitch
           10, 10,      // left/right knee pitch
            5,  5,      // left/right shoulder pitch
            3,  3,      // left/right ankle pitch
            5,  5,      // left/right shoulder roll
            1,  1,      // left/right ankle roll
            5,  5,      // left/right shoulder yaw
            5,  5,      // left/right elbow pitch
            1,  1,      // left/right wrist yaw
            1,  1,      // left/right wrist pitch
            1,  1;      // left/right wrist roll

    std::vector<std::string> robot_order;
    //DR2PRO
    robot_order = {
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    };
    
    std::vector<std::string> policy_order = {
        "left_hip_pitch_joint", "right_hip_pitch_joint",
        "waist_yaw_joint",
        "left_hip_roll_joint", "right_hip_roll_joint",
        "waist_roll_joint",
        "left_hip_yaw_joint", "right_hip_yaw_joint",
        "waist_pitch_joint",
        "left_knee_pitch_joint", "right_knee_pitch_joint",
        "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
        "left_ankle_pitch_joint", "right_ankle_pitch_joint",
        "left_shoulder_roll_joint", "right_shoulder_roll_joint",
        "left_ankle_roll_joint", "right_ankle_roll_joint",
        "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
        "left_elbow_pitch_joint", "right_elbow_pitch_joint",
        "left_wrist_yaw_joint", "right_wrist_yaw_joint", 
        "left_wrist_pitch_joint", "right_wrist_pitch_joint",
        "left_wrist_roll_joint","right_wrist_roll_joint"
    };

    std::vector<std::string> output_order = {
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    };

    robot_to_policy_idx_ = GeneratePermutation(robot_order, policy_order);      // 将机器人的关节位置对应到policy
    policy_to_output_idx_ = GeneratePermutation(policy_order, output_order);    // 将机器人的关节位置对应到policy
    // Training: 0.005 s * 4 = 0.020 s; SDK state machine runs at 1 kHz.
    decimation_ = 20;
    action_scale_ = 0.25;
    omega_scale_ = 1.0;
    dof_vel_scale_ = 0.05;
    clip_observations_ = 100.0;
    clip_actions_ = 10.0;
    signal_dt_ = 0.001f;
    gait_cycle_ = 0.85f;
    cmd_vel_scale_ = Vec3f(1., 1., 1);
    gait_air_ratio_ << 0.38f, 0.38f;
    gait_phase_offset_ << 0.38f, 0.88f;
    obs_dim_ = 102;
    obs_history_num_ = 5;
    act_dim_ = 29;
    obs_total_dim_ = obs_dim_ * (obs_history_num_ + 1);

    dof_pos_default_.setZero(act_dim_);
    dof_pos_default_ << -0.2, -0.2, // left_hip_pitch, right_hip_pitch
                        0.0,          // waist_yaw
                        0.0,  0.0,    // left_hip_roll, right_hip_roll
                        0.0,          // waist_roll
                        0.0,  0.0,    // left_hip_yaw, right_hip_yaw
                        0.0,          // waist_pitch
                        0.4,  0.4,    // left_knee_pitch, right_knee_pitch
                        0.0,  0.0,    // left_shoulder_pitch, right_shoulder_pitch
                       -0.2, -0.2,    // left_ankle_pitch, right_ankle_pitch
                        0.15, -0.15,   // left_shoulder_roll, right_shoulder_roll
                        0.0,  0.0,    // left_ankle_roll, right_ankle_roll
                        0.0,  0.0,    // left_shoulder_yaw, right_shoulder_yaw
                        1.2,  1.2,    // left_elbow_pitch, right_elbow_pitch
                        0.0,  0.0,    // left_wrist_yaw, right_wrist_yaw
                        0.0,  0.0,    // left_wrist_pitch, right_wrist_pitch
                        0.0,  0.0;    // left_wrist_roll, right_wrist_roll

    min_cmd_vel_ = trained_min_cmd_vel_;
    max_cmd_vel_ = trained_max_cmd_vel_;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    ort_inputs_.reserve(1);

    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    try {
        policy_path_ = pro_policy::ResolveModelPath("amp_policy/amp_1018.onnx");
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        pro_policy::ValidateModelSchema(session_, obs_total_dim_, act_dim_, input_names_[0], output_names_[0]);
        policy_loaded_ = true;
    } catch (const std::exception& e) {
        SetInferenceFault(e.what());
        std::cerr << "error loading policy as " << policy_name_ << "\n" << e.what();
        policy_path_ = "load failed";
        policy_loaded_ = false;
    }
}

AmpWalkingPolicyRunner::~AmpWalkingPolicyRunner() {
    std::cout << "AmpWalkingPolicyRunner " << policy_name_ << " destroyed" << std::endl;
}

void AmpWalkingPolicyRunner::UpdateObservationHistory() {
    // ONNX obs[:510] is old history; obs[510:612] is the current frame.
    // Append only after stacking, so the current frame is not duplicated.
    if (obs_history_.size() != static_cast<size_t>(obs_history_num_))
        throw std::runtime_error("AMP observation history is not initialized; call OnEnter first");
    int idx = 0;
    for (const auto& historical_obs : obs_history_) {
        stacked_observation_.segment(idx * obs_dim_, obs_dim_) = historical_obs;
        idx++;
    }
    stacked_observation_.segment(obs_history_num_ * obs_dim_, obs_dim_) = current_observation_;
    obs_history_.pop_front();
    obs_history_.push_back(current_observation_);
}

void AmpWalkingPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << (policy_loaded_ ? " network test success" : " network load failed") << std::endl;
    std::cout << "Policy file: " << policy_path_ << std::endl;
    std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_ << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "min_v: " << min_cmd_vel_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

bool AmpWalkingPolicyRunner::IsReady() const {
    return policy_loaded_ && !HasInferenceFault();
}

void AmpWalkingPolicyRunner::OnEnter() {
    if (policy_loaded_) ResetInferenceFault();
    action_.setZero(act_dim_);
    last_action_.setZero(act_dim_);
    obs_joint_pos_.setZero(act_dim_);
    obs_joint_vel_.setZero(act_dim_);

    current_observation_.setZero(obs_dim_);
    stacked_observation_.setZero(obs_total_dim_);
    obs_history_.clear();
    // Match sim2sim: five zero history frames, then sample the current frame.
    for (int i = 0; i < obs_history_num_; ++i) {
        obs_history_.push_back(VecXf::Zero(obs_dim_));
    }

    run_cnt_ = 0;
    cmd_vel_input_.setZero();
    arm_release_active_ = arm_release_pending_;
    arm_release_pending_ = false;
    arm_release_elapsed_ = 0.;
}

void AmpWalkingPolicyRunner::OnExit() {
    arm_release_pending_ = arm_release_active_ = false;
}

void AmpWalkingPolicyRunner::SetArmReleaseFrom(const Vec7f& left, const Vec7f& right, double duration) {
    if (!left.allFinite() || !right.allFinite() || !std::isfinite(duration) || duration <= 0.)
        throw std::invalid_argument("Invalid AMP arm release pose/duration");
    arm_release_start_ << left, right;
    arm_release_duration_ = duration;
    arm_release_pending_ = true;
}

RobotAction AmpWalkingPolicyRunner::ComputeRobotAction(const RobotBasicState &ro, const UserCommand &uc) {
    if (!policy_loaded_) {
        throw std::runtime_error("policy is not loaded: " + policy_path_);
    }

    Vec3f user_cmd_vel = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
    user_cmd_vel = user_cmd_vel.cwiseMax(-1.0f).cwiseMin(1.0f);
    Eigen::Vector3f vel_delta = user_cmd_vel - cmd_vel_input_;
    vel_delta_const_ << 0.02, 0.02, 0.04;
    for (int i = 0; i < 3; ++i) {
        if (fabs(vel_delta(i)) > vel_delta_const_(i)) {
            vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
        }
    }
    cmd_vel_input_ += vel_delta;
    Vec3f cmd_vel = cmd_vel_input_.cwiseProduct(max_cmd_vel_);
    for (int i = 0; i < 3; ++i) {
        const float lower = std::min(0.0f, min_cmd_vel_(i));
        const float upper = std::max(0.0f, max_cmd_vel_(i));
        cmd_vel(i) = std::clamp(cmd_vel(i), lower, upper);
    }
    cmd_vel = ApplyCommandDeadband(cmd_vel);

    // ========== 步态计算 ==========
    const float step_dt = static_cast<float>(decimation_) * signal_dt_;   // 策略推理时间步长 (0.02s)
    const float gait_time = static_cast<float>(run_cnt_) * step_dt / gait_cycle_;
    Eigen::Vector2f gait_phase;
    Eigen::Vector2f gait_sin;
    Eigen::Vector2f gait_cos;
    for (int i = 0; i < 2; ++i) {
        gait_phase(i) = std::fmod(gait_time + gait_phase_offset_(i), 1.0f);
        gait_sin(i) = std::sin(2.0f * kPi_ * gait_phase(i));
        gait_cos(i) = std::cos(2.0f * kPi_ * gait_phase(i));
    }
    // Training/sim2sim enables phase only when any command exceeds 0.1.
    if (cmd_vel.cwiseAbs().maxCoeff() <= 0.1f) {
        gait_sin.setZero();
        gait_cos.setZero();
    }
    // =============================

    current_observation_.setZero(obs_dim_);
    Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);


    for (int i = 0; i < act_dim_; i++) {
        obs_joint_pos_(i) = ro.joint_pos(robot_to_policy_idx_[i]);  // 这里变成policy的观测位置
        obs_joint_vel_(i) = ro.joint_vel(robot_to_policy_idx_[i]);  // 这里变成policy的观测速度
    }
    current_observation_ << omega_scale_ * ro.base_omega,
                            project_gravity,
                            cmd_vel.cwiseProduct(cmd_vel_scale_),
                            obs_joint_pos_ - dof_pos_default_,
                            dof_vel_scale_ * obs_joint_vel_,
                            last_action_,
                            gait_sin,
                            gait_cos,
                            gait_air_ratio_;

    pro_policy::ValidateObservation(current_observation_, obs_dim_);
    ClipVector(current_observation_, clip_observations_);

    UpdateObservationHistory();
    pro_policy::ValidateObservation(stacked_observation_, obs_total_dim_);

    const std::vector<int64_t> state_shape = {1, obs_total_dim_};
    auto state_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, stacked_observation_.data(), stacked_observation_.size(), state_shape.data(), state_shape.size());

    ort_inputs_.clear();
    ort_inputs_.push_back(std::move(state_tensor));

    auto output_tensors = session_.Run(
        Ort::RunOptions{nullptr}, input_names_, ort_inputs_.data(), ort_inputs_.size(), output_names_, 1);
    action_ = pro_policy::ReadAction(output_tensors, act_dim_);
    ClipVector(action_, clip_actions_);

    last_action_ = action_;
    RobotAction ra;
    ra.goal_joint_pos.setZero(act_dim_);
    ra.goal_joint_vel.setZero(act_dim_);
    ra.tau_ff.setZero(act_dim_);
    ra.kp.setZero(act_dim_);
    ra.kd.setZero(act_dim_);
    for (int i = 0; i < act_dim_; i++) {
        const int idx = policy_to_output_idx_[i];
        ra.goal_joint_pos(i) = action_scale_ * action_(idx) + dof_pos_default_(idx);
        ra.kp(i) = kp_(idx);
        ra.kd(i) = kd_(idx);
    }
    if (arm_release_active_) {
        const float t = std::clamp(float(arm_release_elapsed_ / arm_release_duration_), 0.f, 1.f);
        const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
        ra.goal_joint_pos.segment(3,14) = arm_release_start_ +
            blend*(ra.goal_joint_pos.segment(3,14)-arm_release_start_);
        arm_release_elapsed_ += decimation_ * signal_dt_;
        if (t >= 1.f) arm_release_active_ = false;
    }
    ++run_cnt_;
    return ra;
}

void AmpWalkingPolicyRunner::SetCmdMaxVel(const Vec3f& vel) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(vel(i)) || vel(i) < 0)
            throw std::invalid_argument("AMP maximum command velocity must be finite and nonnegative");
    }
    // Keep the existing state-machine API, bounded by this model's training ranges.
    const Vec3f max_vel = vel.cwiseMin(trained_max_cmd_vel_);
    SetCmdVelLimits((-max_vel).cwiseMax(trained_min_cmd_vel_), max_vel);
}

void AmpWalkingPolicyRunner::SetCmdVelLimits(const Vec3f& min_vel, const Vec3f& max_vel) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(min_vel(i)) || !std::isfinite(max_vel(i)) ||
            min_vel(i) > 0 || max_vel(i) < 0 || min_vel(i) > max_vel(i) ||
            min_vel(i) < trained_min_cmd_vel_(i) || max_vel(i) > trained_max_cmd_vel_(i))
            throw std::invalid_argument("AMP command velocity limits must include zero and stay within training ranges");
    }
    min_cmd_vel_ = min_vel;
    max_cmd_vel_ = max_vel;
}

std::vector<int> AmpWalkingPolicyRunner::GeneratePermutation(
    const std::vector<std::string>& from,
    const std::vector<std::string>& to)
{
    std::unordered_map<std::string, int> idx_map;
    for (int i = 0; i < static_cast<int>(from.size()); ++i) {
        if (!idx_map.emplace(from[i], i).second)
            throw std::invalid_argument("duplicate AMP joint name: " + from[i]);
    }

    std::vector<int> perm;
    for (const auto& name : to) {
        auto it = idx_map.find(name);
        if (it != idx_map.end()) {
            perm.push_back(it->second);
        } else {
            throw std::invalid_argument("missing AMP joint name: " + name);
        }
    }

    return perm;
}
