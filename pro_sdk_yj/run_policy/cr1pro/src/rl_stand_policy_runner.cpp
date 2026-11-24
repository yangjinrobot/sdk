#include "rl_stand_policy_runner.hpp"

#ifdef POLICY_RUNNER_DEBUG
#include <sys/stat.h>
#endif

namespace {
using namespace types;

inline Vec7f ArmWaypoint(float a, float b, float c, float d, float e = 0.0f, float f = 0.0f, float g = 0.0f) {
    Vec7f v;
    v << a, b, c, d, e, f, g;
    return v;
}
}  // namespace

// // 关节限位常量 (7 DOF)
// constexpr double LEFT_ARM_LOWER[]  = {-3.3161, -0.43633, -2.9671, -0.767945, -2.9671, -1.5708, -1.5708};
// constexpr double LEFT_ARM_UPPER[]  = {1.5708, 3.5799, 2.9671, 1.74533, 2.9671, 1.5708, 1.5708};
// constexpr double RIGHT_ARM_LOWER[] = {-3.3161, -0.43633, -2.9671, -0.767945, -2.9671, -1.5708, -1.5708};
// constexpr double RIGHT_ARM_UPPER[] = {1.5708, 3.5799, 2.9671, 1.74533, 2.9671, 1.5708, 1.5708};

// // 关节限位常量 (7 DOF) - 使用 float 匹配 udp_target 类型
// constexpr float LEFT_ARM_LOWER[]  = {-3.3161f, -0.43633f, -2.9671f, -0.767945f, -2.9671f, -1.5708f, -1.5708f};
// constexpr float LEFT_ARM_UPPER[]  = {1.5708f, 3.5799f, 2.9671f, 1.74533f, 2.9671f, 1.5708f, 1.5708f};
// constexpr float RIGHT_ARM_LOWER[] = {-3.3161f, -0.43633f, -2.9671f, -0.767945f, -2.9671f, -1.5708f, -1.5708f};
// constexpr float RIGHT_ARM_UPPER[] = {1.5708f, 3.5799f, 2.9671f, 1.74533f, 2.9671f, 1.5708f, 1.5708f};

constexpr float LEFT_ARM_LOWER[]  = {-3.8397f, -0.349066f, -2.9671f, -0.767945f, -2.9671f, -1.5708f, -1.5708f};
constexpr float LEFT_ARM_UPPER[]  = {1.5708f, 3.49066f, 2.9671f, 1.74533f, 2.9671f, 1.5708f, 1.5708f};
constexpr float RIGHT_ARM_LOWER[] = {-3.8397f, -3.49066f, -2.9671f, -0.767945f, -2.9671f, -1.5708f, -1.5708f};
constexpr float RIGHT_ARM_UPPER[] = {1.5708f, 0.349066f, 2.9671f, 1.74533f, 2.9671f, 1.5708f, 1.5708f};

void RLStandPolicyRunner::LogOpen() {
#ifdef POLICY_RUNNER_DEBUG
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << "/home/user/dr2_sdk_amp_deploy/dr2_pro_sdk/log/policy_runner_"
        << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";

    std::string log_dir = "/home/user/dr2_sdk_amp_deploy/dr2_pro_sdk/log";
    struct stat st;
    if (stat(log_dir.c_str(), &st) != 0) {
        mkdir(log_dir.c_str(), 0755);
    }

    log_file_.open(oss.str(), std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
        log_open_ = true;
        log_file_ << std::fixed << std::setprecision(6);
        Log("=== PolicyRunner log started ===");
        Log(std::string("File: ") + oss.str());
        std::cout << "[PolicyRunner] Log file: " << oss.str() << std::endl;
    } else {
        std::cerr << "[PolicyRunner] FAILED to open log: " << oss.str() << std::endl;
    }
#endif
}

void RLStandPolicyRunner::Log(const std::string& msg) {
#ifdef POLICY_RUNNER_DEBUG
    if (!log_open_) return;
    log_file_ << "[" << total_elapsed_time_ << "s] " << msg << "\n";
#endif
}

void RLStandPolicyRunner::LogFlush() {
#ifdef POLICY_RUNNER_DEBUG
    if (log_open_) log_file_.flush();
#endif
}

void RLStandPolicyRunner::LogArmArray(const char* label, const float* arr, int n) {
#ifdef POLICY_RUNNER_DEBUG
    if (!log_open_) return;
    log_file_ << "[" << total_elapsed_time_ << "s] " << label << " [";
    for (int i = 0; i < n; i++) {
        log_file_ << arr[i] << (i < n - 1 ? ", " : "");
    }
    log_file_ << "]\n";
#endif
}

void RLStandPolicyRunner::ClipVector(VecXf& value, float limit) {
    value = value.cwiseMax(-limit).cwiseMin(limit);
}

float RLStandPolicyRunner::SmoothStep(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

Vec7f RLStandPolicyRunner::SmoothInterpolate(const Vec7f& start, const Vec7f& end, float t) {
    return start + SmoothStep(t) * (end - start);
}

VecXf RLStandPolicyRunner::JointPosToGymOrder(const VecXf& pos, const VecXf& default_pos) {
    VecXf robot_joint_obs = pos.head(kDofDim) - default_pos;
    VecXf gym_obs_joint(29);
// Isaac Gym observation joint order: left leg + right leg + waist + left arm + right arm.
    gym_obs_joint << robot_joint_obs.segment(kLeftLegStart, 6),
                     robot_joint_obs.segment(kRightLegStart, 6),
                     robot_joint_obs.segment(kWaistStart, 3),
                     robot_joint_obs.segment(kLeftArmStart, 7),
                     robot_joint_obs.segment(kRightArmStart, 7);
    return gym_obs_joint;
}

VecXf RLStandPolicyRunner::JointVelToGymOrder(const VecXf& vel) {
    VecXf gym_obs_joint(29);
// Isaac Gym observation joint order: left leg + right leg + waist + left arm + right arm.
    gym_obs_joint << vel.segment(kLeftLegStart, 6),
                     vel.segment(kRightLegStart, 6),
                     vel.segment(kWaistStart, 3),
                     vel.segment(kLeftArmStart, 7),
                     vel.segment(kRightArmStart, 7);
    return gym_obs_joint;
}

VecXf RLStandPolicyRunner::ActionToDeployJointPos(const VecXf& scaled_action) const {
    VecXf goal = dof_pos_default_;

    goal.segment(kLeftLegStart, 6) = dof_pos_default_.segment(kLeftLegStart, 6) + scaled_action.segment(0, 6);
    goal.segment(kRightLegStart, 6) = dof_pos_default_.segment(kRightLegStart, 6) + scaled_action.segment(6, 6);
    goal.segment(kWaistStart, 3) = dof_pos_default_.segment(kWaistStart, 3) + scaled_action.segment(12, 3);

    goal.segment(kLeftArmStart, 7) = current_arm_target_left_;
    goal.segment(kRightArmStart, 7) = current_arm_target_right_;
    return goal;
}

void RLStandPolicyRunner::UpdateObservationHistory() {
    while (static_cast<int>(obs_history_.size()) >= obs_history_num_) {
        obs_history_.pop_front();
    }
    obs_history_.push_back(current_observation_);

    stacked_observation_.setZero();
    int idx = 0;
    for (const auto& obs : obs_history_) {
        stacked_observation_.segment(idx * kSingleObsDim, kSingleObsDim) = obs;
        ++idx;
    }
}

void RLStandPolicyRunner::UpdateArmPreset(const UserCommand& uc) {
    const bool preset_cmd_edge = uc.arm_preset_seq != last_arm_preset_seq_;
    const bool preset_1_edge = uc.arm_preset_1 && !last_arm_preset_1_;
    const bool preset_2_edge = uc.arm_preset_2 && !last_arm_preset_2_;
    const bool preset_3_edge = uc.arm_preset_3 && !last_arm_preset_3_;
    const bool preset_4_edge = uc.arm_preset_4 && !last_arm_preset_4_;
    const bool preset_5_edge = uc.arm_preset_5 && !last_arm_preset_5_;
    const bool preset_6_edge = uc.arm_preset_6 && !last_arm_preset_6_;

    const Vec7f pose_home_left = ArmWaypoint(0.0f, 0.25f, 0.0f, 1.4f);
    const Vec7f pose_home_right = ArmWaypoint(0.0f, -0.25f, 0.0f, 1.4f);
    const Vec7f pose_a_left = ArmWaypoint(0.0f, 0.3f, 0.0f, 0.0f);
    const Vec7f pose_a_right = ArmWaypoint(0.0f, -0.3f, 0.0f, 0.0f);
// const Vec7f pose_a_left = ArmWaypoint(0.0f, 1.3f, 0.0f, 0.0f);
    // const Vec7f pose_a_right = ArmWaypoint(0.0f, -1.3f, 0.0f, 0.0f);
    const Vec7f pose_b_left = ArmWaypoint(0.0f, 0.0f, 0.0f, 0.0f);
    const Vec7f pose_b_right = ArmWaypoint(0.0f, 0.0f, 0.0f, 0.0f);
    const Vec7f pose_c_left = ArmWaypoint(0.0f, 0.25f, 0.0f, -0.0f);
    const Vec7f pose_c_right = ArmWaypoint(0.0f, -0.25f, 0.0f, -0.0f);

    bool start_new_interp = false;
    const int preset = preset_cmd_edge ? uc.arm_preset : 0;
    if (preset == 1 || (!preset_cmd_edge && preset_1_edge)) {
        arm_interp_goal_left_ = pose_a_left;
        arm_interp_goal_right_ = pose_a_right;
        start_new_interp = true;
    } else if (preset == 2 || (!preset_cmd_edge && preset_2_edge)) {
        arm_interp_goal_left_ = pose_b_left;
        arm_interp_goal_right_ = pose_b_right;
        start_new_interp = true;
    } else if (preset == 3 || (!preset_cmd_edge && preset_3_edge)) {
        arm_interp_goal_left_ = pose_c_left;
        arm_interp_goal_right_ = pose_c_right;
        start_new_interp = true;
    } else if (preset == 4 || (!preset_cmd_edge && preset_4_edge)) {
        arm_interp_goal_left_ = pose_b_left;
        arm_interp_goal_right_ = pose_b_right;
        start_new_interp = true;
    } else if (preset == 5 || (!preset_cmd_edge && preset_5_edge)) {
        arm_interp_goal_left_ = pose_a_left;
        arm_interp_goal_right_ = pose_a_right;
        start_new_interp = true;
    } else if (preset == 6 || (!preset_cmd_edge && preset_6_edge)) {
        arm_interp_goal_left_ = pose_home_left;
        arm_interp_goal_right_ = pose_home_right;
        start_new_interp = true;
    }

    if (start_new_interp) {
        arm_interp_start_left_ = current_arm_target_left_;
        arm_interp_start_right_ = current_arm_target_right_;
        arm_interp_start_time_s_ = total_elapsed_time_;
        arm_interp_active_ = true;
    }

    if (arm_interp_active_) {
        const float t = (total_elapsed_time_ - arm_interp_start_time_s_) / std::max(arm_interp_duration_s_, 1e-3f);
        if (t >= 1.0f) {
            current_arm_target_left_ = arm_interp_goal_left_;
            current_arm_target_right_ = arm_interp_goal_right_;
            arm_interp_active_ = false;
        } else {
            current_arm_target_left_ = SmoothInterpolate(arm_interp_start_left_, arm_interp_goal_left_, t);
            current_arm_target_right_ = SmoothInterpolate(arm_interp_start_right_, arm_interp_goal_right_, t);
        }
    }

    last_arm_preset_1_ = uc.arm_preset_1;
    last_arm_preset_2_ = uc.arm_preset_2;
    last_arm_preset_3_ = uc.arm_preset_3;
    last_arm_preset_4_ = uc.arm_preset_4;
    last_arm_preset_5_ = uc.arm_preset_5;
    last_arm_preset_6_ = uc.arm_preset_6;
    last_arm_preset_seq_ = uc.arm_preset_seq;
}

void RLStandPolicyRunner::UpdateArmTargets(const RobotBasicState& ro, const UserCommand& uc) {
    if (!arm_targets_initialized_ && ro.joint_pos.size() >= kRightArmStart + 7) {
        current_arm_target_left_ = ro.joint_pos.segment(kLeftArmStart, 7);
        current_arm_target_right_ = ro.joint_pos.segment(kRightArmStart, 7);
        arm_targets_initialized_ = true;
    }

    if (uc.disable_arm_motion_planning) {
        current_arm_target_left_ = ro.joint_pos.segment(kLeftArmStart, 7);
        current_arm_target_right_ = ro.joint_pos.segment(kRightArmStart, 7);
        arm_interp_active_ = false;
        last_arm_preset_1_ = last_arm_preset_2_ = last_arm_preset_3_ = false;
        last_arm_preset_4_ = last_arm_preset_5_ = last_arm_preset_6_ = false;
        last_arm_preset_seq_ = uc.arm_preset_seq;
        return;
    }
    UpdateArmPreset(uc);
}

std::string RLStandPolicyRunner::ResolvePolicyPath() const {
    return pro_policy::ResolveModelPath("rl_policy/model_0610_rl_moveboxes.onnx");
}

bool RLStandPolicyRunner::ExternalArmControlEnabled() const { return true; }

UdpNode* RLStandPolicyRunner::GetUdpNode() { return pNode; }

void RLStandPolicyRunner::SetUdpNode(UdpNode* node) { pNode = node; }

RLStandPolicyRunner::RLStandPolicyRunner(const std::string& policy_name, RobotName robot_name,
                    float ankle_pitch_default,
                    float hip_pitch_default) : PolicyRunnerBase(policy_name, robot_name) {
    // 打开日志文件
    LogOpen();

    decimation_ = 10;
    max_cmd_vel_.setZero();
    cmd_vel_scale_ << 2.0f, 2.0f, 1.0f;

    kp_ = VecXf::Zero(kDofDim);
    kp_ << 200.0, 200.0, 200.0,
           300.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0, // left
           300.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0, // right
           300.0, 300.0, 200.0, 300.0, 50.0, 30.0,
           300.0, 300.0, 200.0, 300.0, 50.0, 30.0;
    kp_ *= 1.0f;

    kd_ = VecXf::Zero(kDofDim);
    kd_ << 5.0, 5.0, 5.0,
           15.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0,
           15.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0,
           15.0, 15.0, 10.0, 15.0, 2.0, 1.0,
           15.0, 15.0, 10.0, 15.0, 2.0, 1.0;

// kp_ = VecXf::Zero(kDofDim);
    // kp_ << 200.0, 200.0, 200.0,
    //        100.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0,
    //        100.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0,
    //        300.0, 300.0, 200.0, 300.0, 50.0, 30.0,
    //        300.0, 300.0, 200.0, 300.0, 50.0, 30.0;
    // kp_ *= 1.0f;

    // kd_ = VecXf::Zero(kDofDim);
    // kd_ << 5.0, 5.0, 5.0,
    //        3.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0,
    //        3.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0,
    //        15.0, 15.0, 10.0, 15.0, 2.0, 1.0,
           //        15.0, 15.0, 10.0, 15.0, 2.0, 1.0;

    dof_pos_default_ = VecXf::Zero(kDofDim);
    dof_pos_default_.segment(kLeftArmStart, 7) = ArmWaypoint(0.0f, 0.15f, 0.0f, 1.4f);
    dof_pos_default_.segment(kRightArmStart, 7) = ArmWaypoint(0.0f, -0.15f, 0.0f, 1.4f);
    dof_pos_default_.segment(kLeftLegStart, 6) << hip_pitch_default, 0.0f, 0.0f, 0.3f, ankle_pitch_default, 0.0f;
    dof_pos_default_.segment(kRightLegStart, 6) << hip_pitch_default, 0.0f, 0.0f, 0.3f, ankle_pitch_default, 0.0f;

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    ort_inputs_.reserve(1);

    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    try {
        policy_path_ = ResolvePolicyPath();
#ifdef _WIN32
        const std::wstring policy_path_w(policy_path_.begin(), policy_path_.end());
        session_ = Ort::Session(env_, policy_path_w.c_str(), session_options_);
#else
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
#endif
        pro_policy::ValidateModelSchema(session_, obs_dim_, act_dim_, input_names_[0], output_names_[0]);
        policy_loaded_ = true;
        std::cout << "loaded policy: " << policy_path_ << std::endl;
        Log(std::string("Policy loaded: ") + policy_path_);
    } catch (const std::exception& e) {
        SetInferenceFault(e.what());
        policy_loaded_ = false;
        std::cerr << "error loading policy at " << policy_path_ << "\n" << e.what() << std::endl;
        Log(std::string("FAIL policy load: ") + e.what());
    }
    
     // TODO: ip
    // The state machine owns and starts the shared UDP node.


    if (pNode) {
    pNode->setTargetPosition(0, 0.0f);
    pNode->setTargetPosition(1, 0.3f);
    pNode->setTargetPosition(2, 0.0f);
    pNode->setTargetPosition(3, 1.4f);
    pNode->setTargetPosition(4, 0.0f);
    pNode->setTargetPosition(5, 0.0f);
    pNode->setTargetPosition(6, 0.0f);
    pNode->setTargetPosition(7, 0.0f);
    pNode->setTargetPosition(8, -0.3f);
    pNode->setTargetPosition(9, 0.0f);
    pNode->setTargetPosition(10, 1.4f);
    pNode->setTargetPosition(11, 0.0f);
    pNode->setTargetPosition(12, 0.0f);
    pNode->setTargetPosition(13, 0.0f);
    }

    const float arm_vel_limits[14] = {
        1.5f, 1.5f, 1.5f, 2.0f, 2.0f, 2.0f, 2.0f,
        1.5f, 1.5f, 1.5f, 2.0f, 2.0f, 2.0f, 2.0f,
    };
    for (int i = 0; i < 14; i++) {
        intp[i].c_vel_lmt = arm_vel_limits[i];
    }

    Log("Constructor complete");
    LogFlush();
}

RLStandPolicyRunner::~RLStandPolicyRunner() {
    Log("=== RLStandPolicyRunner destroyed ===");
    LogFlush();
    if (log_file_.is_open()) log_file_.close();
    std::cout << "RLStandPolicyRunner " << policy_name_ << " destroyed" << std::endl;
}

void RLStandPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " policy runner" << std::endl;
    std::cout << "dim  : single_obs=" << kSingleObsDim
              << " history=" << obs_history_num_
              << " obs=" << obs_dim_
              << " act=" << act_dim_
              << " dof=" << kDofDim << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

void RLStandPolicyRunner::OnEnter() {
    if (policy_loaded_) ResetInferenceFault();
    std::cout << "\n[PolicyRunner] ╔══════════════════════════════════╗\n"
              << "[PolicyRunner] ║   RLStandPolicyRunner OnEnter    ║\n"
              << "[PolicyRunner] ║   RL Stand Control RL站立控制    ║\n"
              << "[PolicyRunner] ╚══════════════════════════════════╝\n"
              << std::endl;
    action_.setZero(kActDim);
    last_action_.setZero(kActDim);
    goal_joint_pos_ = dof_pos_default_;
    current_observation_.setZero(kSingleObsDim);
    stacked_observation_.setZero(obs_dim_);
    obs_history_.clear();
    for (int i = 0; i < obs_history_num_; ++i) {
        obs_history_.push_back(VecXf::Zero(kSingleObsDim));
    }

    run_cnt_ = 0;
    phase_count_ = 0;
    total_elapsed_time_ = 0.0f;
    cmd_vel_input_.setZero();
    if (has_initial_arm_target_) {
        current_arm_target_left_ = initial_arm_target_left_;
        current_arm_target_right_ = initial_arm_target_right_;
        has_initial_arm_target_ = false;
        arm_targets_initialized_ = true;
    } else {
        current_arm_target_left_ = dof_pos_default_.segment(kLeftArmStart, 7);
        current_arm_target_right_ = dof_pos_default_.segment(kRightArmStart, 7);
        arm_targets_initialized_ = false;
    }
    goal_joint_pos_.segment(kLeftArmStart, 7) = current_arm_target_left_;
    goal_joint_pos_.segment(kRightArmStart, 7) = current_arm_target_right_;
    arm_interp_active_ = false;
    last_arm_preset_1_ = last_arm_preset_2_ = last_arm_preset_3_ = false;
    last_arm_preset_4_ = last_arm_preset_5_ = last_arm_preset_6_ = false;
    last_arm_preset_seq_ = 0;
    udp_arm_control_active_ = false;
    udp_no_data_count_ = 0;

#ifdef POLICY_RUNNER_DEBUG
    // 重置诊断变量
    has_last_udp_arrival_ = false;
    max_udp_gap_ms_ = 0.0f;
    udp_packet_count_ = 0;
    udp_gap_warn_count_ = 0;
    has_last_udp_targets_ = false;
    udp_target_unchanged_count_ = 0;
    udp_target_last_change_time_ = 0.0f;
    has_last_goal_ = false;
    goal_unchanged_count_ = 0;
    goal_last_change_time_ = 0.0f;
    goal_stale_alert_count_ = 0;
    diag_log_counter_ = 0;
    fast_log_counter_ = 0;
#endif

    Log("=== OnEnter: all state reset ===");
    LogFlush();
}

void RLStandPolicyRunner::OnExit() {
    std::cout << "\n[PolicyRunner] ╔══════════════════════════════════╗\n"
              << "[PolicyRunner] ║   RLStandPolicyRunner OnExit     ║\n"
              << "[PolicyRunner] ║   退出 RL Stand Control 模式      ║\n"
              << "[PolicyRunner] ╚══════════════════════════════════╝\n"
              << std::endl;
    Log("=== OnExit ===");
    LogFlush();
}

bool RLStandPolicyRunner::IsReady() const {
    return policy_loaded_ && !HasInferenceFault();
}

RobotAction RLStandPolicyRunner::ComputeRobotAction(const RobotBasicState& ro, const UserCommand& uc) {
    if (!policy_loaded_) {
        throw std::runtime_error("policy is not loaded: " + policy_path_);
    }

    if (!pNode || !ExternalArmControlEnabled()) udp_arm_control_active_ = false;
    total_elapsed_time_ += decimation_ * signal_dt_;
    UpdateArmTargets(ro, uc);

    Vec3f user_cmd_vel(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
    Eigen::Vector3f vel_delta = user_cmd_vel - cmd_vel_input_;
    vel_delta_const_ << 0.02f, 0.02f, 0.04f;
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(vel_delta(i)) > vel_delta_const_(i)) {
            vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
        }
    }
    cmd_vel_input_ += vel_delta;
    const Vec3f cmd_vel =
        ApplyCommandDeadband(cmd_vel_input_.cwiseProduct(max_cmd_vel_));

    if (cmd_vel.head<2>().norm() > 0.1f) {
        phase_count_ = run_cnt_ * decimation_;
    } else {
        phase_count_ = 0;
    }
    const float phase = 2.0f * kPi * phase_count_ * signal_dt_ / gait_cycle_time_;
    Eigen::Vector2f sin_cos(std::sin(phase), std::cos(phase));

    const Vec3f projected_gravity = ro.base_rot_mat.transpose() * Vec3f(0.0f, 0.0f, -1.0f);
    current_observation_ << ro.base_omega * omega_scale_,
                            projected_gravity,
                            cmd_vel.cwiseProduct(cmd_vel_scale_),
                            JointPosToGymOrder(ro.joint_pos, dof_pos_default_) * dof_pos_scale_,
                            JointVelToGymOrder(ro.joint_vel) * dof_vel_scale_,
                            last_action_,
                            sin_cos(0),
                            sin_cos(1);
    pro_policy::ValidateObservation(current_observation_, kSingleObsDim);
    ClipVector(current_observation_, clip_observations_);
    UpdateObservationHistory();
    pro_policy::ValidateObservation(stacked_observation_, obs_dim_);

    const std::vector<int64_t> state_shape = {1, obs_dim_};
    auto state_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, stacked_observation_.data(), stacked_observation_.size(), state_shape.data(), state_shape.size());
    ort_inputs_.clear();
    ort_inputs_.push_back(std::move(state_tensor));

    auto output_tensors = session_.Run(
        Ort::RunOptions{nullptr}, input_names_, ort_inputs_.data(), ort_inputs_.size(), output_names_, 1);
    action_ = pro_policy::ReadAction(output_tensors, act_dim_);
    ClipVector(action_, clip_actions_);
    last_action_ = action_;

    goal_joint_pos_ = ActionToDeployJointPos(action_ * action_scale_);

    // ══════════════════════════════════════════════════════════
    //  手臂双模式控制逻辑 + 文件日志
    // ══════════════════════════════════════════════════════════
    if (pNode && ExternalArmControlEnabled() && pNode->consumeNewTargetData()) {
        // ── UDP 间隙检测: 断连后重连时平滑重置插补器 ──
        if (udp_arm_control_active_ && udp_no_data_count_ > 50) {
            for (int i = 0; i < 14; i++) {
                float tgt = 0.0f;
                pNode->getTargetPosition(i, tgt);
                intp[i].o_pos = tgt;
            }
            std::cout << "[PolicyRunner] UDP 数据恢复，插补器已平滑重置 (断连 "
                      << udp_no_data_count_ << " 周期)" << std::endl;
        }
        udp_no_data_count_ = 0;
#ifdef POLICY_RUNNER_DEBUG
        const auto now_ts = SteadyClock::now();

        // ── UDP 包到达间隔监控 ──
        ++udp_packet_count_;
        if (has_last_udp_arrival_) {
            float gap_ms = std::chrono::duration<float, std::milli>(now_ts - last_udp_arrival_time_).count();
            if (gap_ms > max_udp_gap_ms_) max_udp_gap_ms_ = gap_ms;
            if (gap_ms > 15.0f) {
                ++udp_gap_warn_count_;
                Log("[WARN] UDP gap=" + std::to_string(gap_ms) + "ms pkt#" +
                    std::to_string(udp_packet_count_));
            }
        }
        last_udp_arrival_time_ = now_ts;
        has_last_udp_arrival_ = true;

        // ── UDP 目标值跳变 & 停滞检测 ──
        float new_targets[14];
        for (int i = 0; i < 14; i++) pNode->getTargetPosition(i, new_targets[i]);

        if (has_last_udp_targets_) {
            float max_jump = 0.0f;
            int worst_j = -1;
            bool any_changed = false;
            for (int i = 0; i < 14; i++) {
                float jump = std::fabs(new_targets[i] - last_udp_targets_[i]);
                if (jump > 1e-5f) any_changed = true;
                if (jump > max_jump) { max_jump = jump; worst_j = i; }
            }

            if (any_changed) {
                udp_target_unchanged_count_ = 0;
                udp_target_last_change_time_ = total_elapsed_time_;
            } else {
                ++udp_target_unchanged_count_;
            }

            if (max_jump > 0.1f) {
                Log("[WARN] UDP target jump: joint[" + std::to_string(worst_j) +
                    "] delta=" + std::to_string(max_jump) + " rad");
                LogArmArray("  old_tgt", last_udp_targets_, 14);
                LogArmArray("  new_tgt", new_targets, 14);
            }
            for (int i = 0; i < 14; i++) last_udp_targets_[i] = new_targets[i];
        } else {
            for (int i = 0; i < 14; i++) last_udp_targets_[i] = new_targets[i];
            has_last_udp_targets_ = true;
            udp_target_last_change_time_ = total_elapsed_time_;
        }
#endif

        // ── 首次切换到 UDP 模式 ──
        if (!udp_arm_control_active_) {
            for (int i = 0; i < 14; i++) {
                intp[i].o_pos = ro.joint_pos(i + kLeftArmStart);
                intp[i].i_currPos = ro.joint_pos(i + kLeftArmStart);
            }
            udp_arm_control_active_ = true;
            std::cout << "\n[PolicyRunner] ★★★ 手臂控制切换: ONNX → UDP 远程模式 ★★★\n" << std::endl;

#ifdef POLICY_RUNNER_DEBUG
            Log("*** MODE SWITCH: ONNX -> UDP ***");
            float new_targets[14];
            for (int i = 0; i < 14; i++) pNode->getTargetPosition(i, new_targets[i]);
            float cur_arm[14];
            for (int i = 0; i < 14; i++) cur_arm[i] = ro.joint_pos(i + kLeftArmStart);
            LogArmArray("  anchor_joints", cur_arm, 14);
            LogArmArray("  udp_target", new_targets, 14);

            float anchor_gap = 0.0f;
            int anchor_worst = -1;
            for (int i = 0; i < 14; i++) {
                float g = std::fabs(cur_arm[i] - new_targets[i]);
                if (g > anchor_gap) { anchor_gap = g; anchor_worst = i; }
            }
            Log("  anchor-target gap: joint[" + std::to_string(anchor_worst) +
                "]=" + std::to_string(anchor_gap) + " rad (" +
                std::to_string(anchor_gap * 180.0f / kPi) + " deg)");
            if (anchor_gap > 0.5f) {
                Log("[CRIT] anchor gap HUGE! interp vel=" +
                    std::to_string(intp[anchor_worst].c_vel_lmt) + " rad/s, est chase=" +
                    std::to_string(anchor_gap / intp[anchor_worst].c_vel_lmt) + "s");
            }
            LogFlush();
#endif
        }
    } else {
        if (udp_arm_control_active_) {
            udp_no_data_count_++;
        }
    }

    if (udp_arm_control_active_) {
        for (int i = 0; i < 14; i++) {
            float udp_target = 0.0f;
            pNode->getTargetPosition(i, udp_target);
            float original_target = udp_target;

            if (i < 7) {
                udp_target = std::clamp(udp_target, LEFT_ARM_LOWER[i], LEFT_ARM_UPPER[i]);
            } else {
                udp_target = std::clamp(udp_target, RIGHT_ARM_LOWER[i-7], RIGHT_ARM_UPPER[i-7]);
            }

#ifdef POLICY_RUNNER_DEBUG
            if (std::fabs(udp_target - original_target) > 1e-4f) {
                Log("[WARN] joint clamp: j[" + std::to_string(i) + "] " +
                    std::to_string(original_target) + " -> " + std::to_string(udp_target));
            }
#endif

            intp[i].i_tgtPos = udp_target;
            intp[i].i_currPos = ro.joint_pos(i + kLeftArmStart);
            intp[i].loop(decimation_ * signal_dt_);
            goal_joint_pos_(i + kLeftArmStart) = intp[i].o_pos;
        }
    }

#ifdef POLICY_RUNNER_DEBUG
    // ══════════════════════════════════════════════════════════
    //  核心: goal_joint_pos_ 停滞检测
    //  这是触发 joint_damping (error_code=10) 的直接原因
    // ══════════════════════════════════════════════════════════
    {
        float cur_goal_arm[14];
        for (int i = 0; i < 7; i++) {
            cur_goal_arm[i]     = goal_joint_pos_(i + kLeftArmStart);
            cur_goal_arm[i + 7] = goal_joint_pos_(i + kRightArmStart);
        }

        if (has_last_goal_) {
            float max_diff = 0.0f;
            for (int i = 0; i < 14; i++) {
                float d = std::fabs(cur_goal_arm[i] - last_goal_arm_[i]);
                if (d > max_diff) max_diff = d;
            }

            if (max_diff < 1e-5f) {
                ++goal_unchanged_count_;
            } else {
                if (goal_unchanged_count_ > 50) {
                    Log("[INFO] goal was frozen for " + std::to_string(goal_unchanged_count_) +
                        " cycles (" + std::to_string(goal_unchanged_count_ * decimation_ * signal_dt_) +
                        "s) before t=" + std::to_string(total_elapsed_time_) + "s");
                }
                goal_unchanged_count_ = 0;
                goal_last_change_time_ = total_elapsed_time_;
            }

            // 停滞告警: 超过 100 个周期 (~100ms) 未变化
            if (goal_unchanged_count_ > 0 && goal_unchanged_count_ % 100 == 0) {
                float frozen_sec = goal_unchanged_count_ * decimation_ * signal_dt_;
                ++goal_stale_alert_count_;
                Log("[ALERT] goal_joint_pos FROZEN " + std::to_string(goal_unchanged_count_) +
                    " cycles (" + std::to_string(frozen_sec) + "s)! " +
                    "alert#" + std::to_string(goal_stale_alert_count_));
                LogArmArray("  frozen_goal_arm", cur_goal_arm, 14);
                float actual_arm[14];
                for (int i = 0; i < 14; i++) actual_arm[i] = ro.joint_pos(i + kLeftArmStart);
                LogArmArray("  actual_joints", actual_arm, 14);

                if (udp_arm_control_active_) {
                    Log("  mode=UDP, udp_target_unchanged=" +
                        std::to_string(udp_target_unchanged_count_) + " cycles");
                    LogArmArray("  udp_targets", last_udp_targets_, 14);
                    float interp_out[14];
                    for (int i = 0; i < 14; i++) interp_out[i] = intp[i].o_pos;
                    LogArmArray("  interp_output", interp_out, 14);
                } else {
                    Log("  mode=ONNX");
                }

                if (frozen_sec >= 0.5f) {
                    Log("[CRIT] *** FROZEN > 0.5s — robot WILL enter joint_damping! ***");
                }
                LogFlush();
            }
        } else {
            goal_last_change_time_ = total_elapsed_time_;
        }

        for (int i = 0; i < 14; i++) last_goal_arm_[i] = cur_goal_arm[i];
        has_last_goal_ = true;
    }

    // ══════════════════════════════════════════════════════════
    //  高频日志: 每 10 周期 (~10ms) 记录手臂段 goal 和实际关节
    // ══════════════════════════════════════════════════════════
    ++fast_log_counter_;
    if (fast_log_counter_ >= 10) {
        fast_log_counter_ = 0;
        if (log_open_) {
            log_file_ << "[" << total_elapsed_time_ << "s] GOAL ";
            for (int i = 0; i < kDofDim; i++) {
                log_file_ << goal_joint_pos_(i) << (i < kDofDim - 1 ? "," : "");
            }
            log_file_ << "\n";

            log_file_ << "[" << total_elapsed_time_ << "s] ACTUAL ";
            for (int i = 0; i < kDofDim; i++) {
                log_file_ << ro.joint_pos(i) << (i < kDofDim - 1 ? "," : "");
            }
            log_file_ << "\n";
        }
    }

    // ══════════════════════════════════════════════════════════
    //  周期性汇总日志 (每 ~1s)
    // ══════════════════════════════════════════════════════════
    ++diag_log_counter_;
    if (diag_log_counter_ >= 1000) {
        diag_log_counter_ = 0;

        Log("--- PERIODIC STATUS ---");
        Log("  UDP: pkts=" + std::to_string(udp_packet_count_) +
            ", max_gap=" + std::to_string(max_udp_gap_ms_) + "ms" +
            ", gap_warns=" + std::to_string(udp_gap_warn_count_));
        Log("  UDP target: unchanged=" + std::to_string(udp_target_unchanged_count_) +
            " cycles, last_change=" + std::to_string(udp_target_last_change_time_) + "s");
        Log("  Goal arm: unchanged=" + std::to_string(goal_unchanged_count_) +
            " cycles, last_change=" + std::to_string(goal_last_change_time_) + "s" +
            ", stale_alerts=" + std::to_string(goal_stale_alert_count_));
        Log("  Mode: " + std::string(udp_arm_control_active_ ? "UDP" : "ONNX") +
            ", interp_active=" + std::string(arm_interp_active_ ? "yes" : "no"));

        if (udp_arm_control_active_) {
            LogArmArray("  udp_tgt", last_udp_targets_, 14);
            float interp_out[14];
            for (int i = 0; i < 14; i++) interp_out[i] = intp[i].o_pos;
            LogArmArray("  interp_out", interp_out, 14);
        }

        float cur_arm_goal[14];
        for (int i = 0; i < 7; i++) {
            cur_arm_goal[i] = goal_joint_pos_(i + kLeftArmStart);
            cur_arm_goal[i+7] = goal_joint_pos_(i + kRightArmStart);
        }
        LogArmArray("  goal_arm", cur_arm_goal, 14);

        float cur_arm_actual[14];
        for (int i = 0; i < 14; i++) cur_arm_actual[i] = ro.joint_pos(i + kLeftArmStart);
        LogArmArray("  actual_arm", cur_arm_actual, 14);

        // 插补器跟踪误差
        if (udp_arm_control_active_) {
            float max_ie = 0.0f;
            int iw = -1;
            for (int i = 0; i < 14; i++) {
                float e = std::fabs(intp[i].o_pos - intp[i].i_tgtPos);
                if (e > max_ie) { max_ie = e; iw = i; }
            }
            Log("  interp track err: j[" + std::to_string(iw) + "]=" +
                std::to_string(max_ie) + " rad");
        }

        LogFlush();
    }
#endif

    // ── 向Python端回传机器人关节状态（降频到50Hz）──
    ++cntr;
    if (pNode && cntr >= 20) {
        cntr = 0;
        for (int i = 0; i < 7; i++) {
            pNode->setCurrentPosition(i,     ro.joint_pos(i + kLeftArmStart));
            pNode->setCurrentVelocity(i,     ro.joint_vel(i + kLeftArmStart));
            pNode->setCurrentTorque(i,       ro.joint_tau(i + kLeftArmStart));
        }
        for (int i = 0; i < 7; i++) {
            pNode->setCurrentPosition(i + 7, ro.joint_pos(i + kRightArmStart));
            pNode->setCurrentVelocity(i + 7, ro.joint_vel(i + kRightArmStart));
            pNode->setCurrentTorque(i + 7,   ro.joint_tau(i + kRightArmStart));
        }
        pNode->sendCurrentStates();
    }

    // ── 周期性终端状态日志 (每 ~2s) ──
    ++status_print_cnt_;
    if (status_print_cnt_ >= 200) {
        status_print_cnt_ = 0;
        std::cout << "\n[PolicyRunner] ──── 状态概览 ────\n"
                  << "  手臂控制模式: " << (udp_arm_control_active_ ? "UDP (远程)" : "ONNX (策略)") << "\n"
                  << "  速度指令: fwd=" << cmd_vel_input_(0)
                  << " side=" << cmd_vel_input_(1)
                  << " turn=" << cmd_vel_input_(2) << "\n"
                  << "  手臂插补: " << (arm_interp_active_ ? "进行中" : "空闲") << "\n"
                  << "  运行时间: " << total_elapsed_time_ << "s\n"
                  << "────────────────────────────────\n"
                  << std::endl;
    }

    RobotAction ra;
    ra.goal_joint_pos = goal_joint_pos_;
    ra.goal_joint_vel = VecXf::Zero(kDofDim);
    ra.tau_ff = VecXf::Zero(kDofDim);
    ra.kp = kp_;
    ra.kd = kd_;

    ++run_cnt_;
    if (cmd_vel_input_.head<2>().norm() < 0.1f || run_cnt_ % 100 == 0) {
        run_cnt_ = 0;
    }
    return ra;
}

void RLStandPolicyRunner::SetCmdMaxVel(const Vec3f& vel) {
    for (int i = 0; i < 3; ++i) {
        if (vel(i) < 0) {
            std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
        }
    }
    max_cmd_vel_ = vel;
}

void RLStandPolicyRunner::SetArmInterpDuration(float duration_s) {
    if (duration_s > 0.05f) {
        arm_interp_duration_s_ = duration_s;
    }
}

const VecXf& RLStandPolicyRunner::GetDefaultJointPosition() const {
    return dof_pos_default_;
}

void RLStandPolicyRunner::SetInitialArmTargets(const Vec7f& left_arm, const Vec7f& right_arm) {
    initial_arm_target_left_ = left_arm;
    initial_arm_target_right_ = right_arm;
    has_initial_arm_target_ = true;
}

void RLStandPolicyRunner::MotorIntp::loop(float ts){
//限制ts
    if(ts<0.001f)ts=0.001f;
    if(ts>0.5f)ts=0.5f;
//直线插补
    if((o_pos+c_vel_lmt*ts)<i_tgtPos){
        o_pos=o_pos+c_vel_lmt*ts;
    }else if((o_pos-c_vel_lmt*ts)>i_tgtPos){
        o_pos=o_pos-c_vel_lmt*ts;
    }else{
        o_pos=i_tgtPos;
    }
}

void RLStandPolicyRunner::MotorIntp::markCurrPos(){
    o_pos=i_currPos;
}
