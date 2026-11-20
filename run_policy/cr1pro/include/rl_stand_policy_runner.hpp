/**
 * @file common_policy_runner _log.hpp
 * @brief Isaac Gym 15-action policy runner for 29-DOF DR2 Pro deploy
 *        — 带文件日志版本，专注排查 joint_damping 触发原因
 *
 * 核心排查逻辑:
 *   机器人 error_code=10 (Joint data error!) 进入 joint_damping，
 *   通常是因为 goal_joint_pos_ 长时间不变（stale），
 *   运控板检测到关节指令停滞超时后主动切阻尼保护。
 *
 * 本文件日志重点:
 *   1. goal_joint_pos_ 手臂段（14关节）的逐周期变化检测 → 发现"冻住"
 *   2. UDP 接收目标的逐包记录 → 发现 Python 端发送停滞
 *   3. 插补器输出 vs 目标 vs 实际关节 → 发现插补器卡死
 *   4. 最终发布的完整 29-DOF goal → 回溯阻尼前的指令序列
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <onnxruntime_cxx_api.h>

#include "policy_runner_base.hpp"
#include "udp_node.hpp"

// 定义此宏启用调试日志，注释掉则为 release 模式（无日志开销）
// 也可通过编译选项 -DPOLICY_RUNNER_DEBUG 启用
// #define POLICY_RUNNER_DEBUG

class RLStandPolicyRunner : public PolicyRunnerBase {
private:
    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr int kDofDim = 29;
    static constexpr int kActDim = 15;
    static constexpr int kSingleObsDim = 84;
    static constexpr int kObsHistoryNum = 6;
    static constexpr int kWaistStart = 0;
    static constexpr int kLeftArmStart = 3;
    static constexpr int kRightArmStart = 10;
    static constexpr int kLeftLegStart = 17;
    static constexpr int kRightLegStart = 23;

    VecXf kp_, kd_;
    VecXf dof_pos_default_;
    Vec3f max_cmd_vel_;
    Vec3f cmd_vel_scale_;

    float action_scale_ = 0.25f;
    float omega_scale_ = 1.0f;
    float dof_pos_scale_ = 1.0f;
    float dof_vel_scale_ = 0.05f;
    float clip_observations_ = 100.0f;
    float clip_actions_ = 100.0f;
    float gait_cycle_time_ = 1.0f;
    float signal_dt_ = 0.001f;
    float total_elapsed_time_ = 0.0f;
    int phase_count_ = 0;

    int obs_dim_ = kSingleObsDim * kObsHistoryNum;
    int obs_history_num_ = kObsHistoryNum;
    int act_dim_ = kActDim;

    VecXf current_observation_;
    VecXf stacked_observation_;
    VecXf action_, last_action_;
    VecXf goal_joint_pos_;
    std::deque<VecXf> obs_history_;

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
    bool has_initial_arm_target_ = false;
    bool arm_targets_initialized_ = false;

    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    bool policy_loaded_ = false;
    std::string policy_path_;

    const char* input_names_[1] = {"Input"};
    const char* output_names_[1] = {"Output"};

    bool arm_motion_enabled_ = true;
    float arm_motion_start_time_ = 5.0;
    int arm_waypoints_num_ = 32;  // 路径点数量
    float arm_motion_duration_per_segment_ = 0.6;
    int current_waypoint_index_ = 0;
    float arm_motion_elapsed_time_ = 0.0;
    bool arm_motion_started_ = false;
    bool arm_motion_completed_ = false;

    inline static UdpNode* pNode = nullptr;  // shared across instances for GUI command routing

    // ── UDP手臂控制模式 ──
    // false: ONNX模型控制手臂（默认），true: Python端通过UDP控制手臂
    bool udp_arm_control_active_ = false;
    int udp_no_data_count_ = 0;

    // ══════════════════════════════════════════════════════════
    //  文件日志系统
    // ══════════════════════════════════════════════════════════
    std::ofstream log_file_;
    bool log_open_ = false;

    // void LogOpen() {
    //     auto now = std::chrono::system_clock::now();
    //     auto t = std::chrono::system_clock::to_time_t(now);
    //     std::tm tm{};
    //     localtime_r(&t, &tm);

    //     std::ostringstream oss;
    //     oss << "/tmp/policy_runner_"
    //         << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";

    //     log_file_.open(oss.str(), std::ios::out | std::ios::trunc);
    //     if (log_file_.is_open()) {
    //         log_open_ = true;
    //         log_file_ << std::fixed << std::setprecision(6);
    //         Log("=== PolicyRunner log started ===");
    //         Log(std::string("File: ") + oss.str());
    //         std::cout << "[PolicyRunner] Log file: " << oss.str() << std::endl;
    //     } else {
    //         std::cerr << "[PolicyRunner] FAILED to open log: " << oss.str() << std::endl;
    //     }
    // }

    void LogOpen();

    void Log(const std::string& msg);

    void LogFlush();

    void LogArmArray(const char* label, const float* arr, int n);

#ifdef POLICY_RUNNER_DEBUG
    // ══════════════════════════════════════════════════════════
    //  诊断追踪变量
    // ══════════════════════════════════════════════════════════
    using SteadyClock = std::chrono::steady_clock;

    // UDP 包到达监控
    SteadyClock::time_point last_udp_arrival_time_{};
    bool has_last_udp_arrival_ = false;
    float max_udp_gap_ms_ = 0.0f;
    int udp_packet_count_ = 0;
    int udp_gap_warn_count_ = 0;

    // UDP 目标值跳变 & 停滞检测
    float last_udp_targets_[14] = {};
    bool has_last_udp_targets_ = false;
    int udp_target_unchanged_count_ = 0;
    float udp_target_last_change_time_ = 0.0f;

    // goal_joint_pos_ 停滞检测 (最关键!)
    float last_goal_arm_[14] = {};
    bool has_last_goal_ = false;
    int goal_unchanged_count_ = 0;
    float goal_last_change_time_ = 0.0f;
    int goal_stale_alert_count_ = 0;

    // 周期性日志节流
    int diag_log_counter_ = 0;
    int fast_log_counter_ = 0;
#endif

    // ── 辅助 ──
    static void ClipVector(VecXf& value, float limit);

    static float SmoothStep(float t);

    static Vec7f SmoothInterpolate(const Vec7f& start, const Vec7f& end, float t);

    static VecXf JointPosToGymOrder(const VecXf& pos, const VecXf& default_pos);

    static VecXf JointVelToGymOrder(const VecXf& vel);

    VecXf ActionToDeployJointPos(const VecXf& scaled_action) const;

    void UpdateObservationHistory();

    void UpdateArmPreset(const UserCommand& uc);

    void UpdateArmTargets(const RobotBasicState& ro, const UserCommand& uc);

    std::string ResolvePolicyPath() const;

protected:
    // Automatic gestures keep their arm trajectory even when the shared UDP
    // node receives targets. Normal RL Stand retains its existing behavior.
    virtual bool ExternalArmControlEnabled() const;

public:
    static UdpNode* GetUdpNode();
    static void SetUdpNode(UdpNode* node);

    RLStandPolicyRunner(const std::string& policy_name, RobotName robot_name,
                        float ankle_pitch_default = -0.19f,
                        float hip_pitch_default = -0.1f);

    ~RLStandPolicyRunner() override;

    void DisplayPolicyInfo() override;

    int cntr=0;
    bool firstRun=true;
    int status_print_cnt_=0;

    void OnEnter() override;

    struct MotorIntp{
    ///电动关节轨迹直线插补（非S轨迹察布）
        float i_tgtPos=0;
        float i_currPos=0;
        float o_pos=0;
        float c_vel_lmt=0.5f; //关节速度 rad/s

        void loop(float ts);

        void markCurrPos();
    };
    struct MotorIntp intp[14];

    void OnExit() override;

    bool IsReady() const override;

    RobotAction ComputeRobotAction(const RobotBasicState& ro, const UserCommand& uc) override;

    void SetCmdMaxVel(const Vec3f& vel);

    void SetArmInterpDuration(float duration_s);

    const VecXf& GetDefaultJointPosition() const;

    void SetInitialArmTargets(const Vec7f& left_arm, const Vec7f& right_arm);
};
