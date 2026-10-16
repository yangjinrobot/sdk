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
#include <filesystem>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <unordered_map>
#include <deque>

class ArmPolicyRunner : public PolicyRunnerBase {
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

    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    // 打招呼控制
    bool waving_active_ = false;
    int wave_phase_ = 0;                // 0:屈肘, 1:摆动右肩
    int wave_step_counter_ = 0;         // 当前阶段已执行步数
    int wave_total_steps_phase0_ = 0;   // 屈肘阶段总步数
    int wave_total_steps_phase1_ = 0;   // 每个摆动半周期步数
    int wave_swing_count_ = 0;          // 已完成摆动周期数（一次来回算1周期）
    bool wave_swing_direction_ = true;  // true:目标-0.45, false:目标-0.15
    float wave_original_right_shoulder_roll_ = 0.0f;  // 暂存原始默认位置（可选）
    int right_wrist_yaw_idx_;   // 右手腕偏航关节在 action_ 中的索引

    // 恢复阶段
    int wave_restore_steps_ = 0;          // 恢复阶段总步数
    int wave_restore_counter_ = 0;        // 当前步数

    VecXf wave_start_action_;             // 恢复开始时各关节的 action_ 副本

    // 动作平滑
    VecXf smooth_action_;               // 平滑后的动作（用于输出）
    float action_smooth_coef_;          // 平滑系数，0~1之间，越大响应越快
    bool first_smooth_run_;   // 平滑首帧标志

public:
    ArmPolicyRunner(const std::string& policy_name, RobotName robot_name) : PolicyRunnerBase(policy_name, robot_name) {
        kp_ = VecXf::Zero(29);
        kp_ << 300, 300,   // left/right hip pitch
               200,        // waist yaw
               300, 300,   // left/right hip roll
               200,        // waist roll
               300, 300,   // left/right hip yaw
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
               10,          // waist yaw
               10, 10,      // left/right hip roll
               10,          // waist roll
               10, 10,      // left/right hip yaw
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

        kp_ = kp_ * 0.8;      // 修改

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

        // 找到右手腕偏航关节在 policy_order 中的索引
        auto it = std::find(policy_order.begin(), policy_order.end(), "right_wrist_yaw_joint");
        if (it != policy_order.end()) {
            right_wrist_yaw_idx_ = std::distance(policy_order.begin(), it);
        } else {
            std::cerr << "Warning: right_wrist_yaw_joint not found in policy_order!" << std::endl;
            right_wrist_yaw_idx_ = 0; // fallback
        }

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

        std::vector<std::string> arm_joints_order = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint"
        };

        robot_to_policy_idx_ = GeneratePermutation(robot_order, policy_order);      // 将机器人的关节位置对应到policy
        policy_to_output_idx_ = GeneratePermutation(policy_order, output_order);    // 将机器人的关节位置对应到policy
        policy_to_arm_idx_ = GeneratePermutation(policy_order, arm_joints_order);

        decimation_ = 20;
        action_scale_ = 0.25;
        omega_scale_ = 1.0;
        dof_vel_scale_ = 0.05;
        cmd_vel_scale_ = Vec3f(1., 1., 1);
        obs_dim_ = 96;
        obs_history_num_ = 5;
        act_dim_ = 29;
        obs_total_dim_ = obs_dim_ * obs_history_num_;

        dof_pos_default_.setZero(act_dim_);
        dof_pos_default_ << -0.15, -0.15, // left_hip_pitch, right_hip_pitch
                            0.0,          // waist_yaw
                            0.0,  0.0,    // left_hip_roll, right_hip_roll
                            0.0,          // waist_roll
                            0.0,  0.0,    // left_hip_yaw, right_hip_yaw
                            0.0,          // waist_pitch
                            0.3,  0.3,    // left_knee_pitch, right_knee_pitch
                            0.0,  0.0,    // left_shoulder_pitch, right_shoulder_pitch
                            -0.15, -0.15, // left_ankle_pitch, right_ankle_pitch
                            0.15,-0.15,   // left_shoulder_roll, right_shoulder_roll
                            0.0,  0.0,    // left_ankle_roll, right_ankle_roll
                            0.0,  0.0,    // left_shoulder_yaw, right_shoulder_yaw
                            0.0,  0.0,    // left_elbow_pitch, right_elbow_pitch
                            0.0,  0.0,    // left_wrist_yaw, right_wrist_yaw
                            0.0,  0.0,    // left_wrist_pitch, right_wrist_pitch
                            0.0,  0.0;    // left_wrist_roll, right_wrist_roll

        max_cmd_vel_.setZero(3);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        ort_inputs_.reserve(1);

        env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
        memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

        try {
            namespace fs = std::filesystem;
            const fs::path base = fs::path(__FILE__).parent_path();
            policy_path_ = fs::canonical(base / ".." / "policy" / "policy_arm.onnx").string();
            session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        } catch (const std::exception& e) {
            std::cerr << "error loading policy as " << policy_name_ << "\n" << e.what();
            policy_path_ = "load failed";
        }
        
        wave_total_steps_phase0_ = decimation_ * 4;    // 屈肘持续约 2.4 秒
        wave_total_steps_phase1_ = decimation_ * 4;    // 每个半周期持续约 2.4 秒
        wave_restore_steps_ = decimation_ * 4;         // 恢复持续约 1.6 秒
        action_smooth_coef_ = 0.25f;    // 可调，例如 0.25
        first_smooth_run_ = true;
        smooth_action_.setZero(act_dim_);
    }
    // 启动右手打招呼动作
    void StartWavingRightArm() {
        waving_active_ = true;
        wave_phase_ = 0;
        wave_step_counter_ = 0;
        wave_swing_count_ = 0;
        wave_swing_direction_ = true;   // 先到 -0.45
        // 可选：保存原始肩关节默认值（dof_pos_default_ 中右肩roll的索引16）
        wave_original_right_shoulder_roll_ = dof_pos_default_(16);
    }

    // 强制停止打招呼（例如手柄松开时调用）
    void StopWaving() {
        waving_active_ = false;
    }
    ~ArmPolicyRunner() override {
        std::cout << "ArmPolicyRunner " << policy_name_ << " destroyed" << std::endl;
    }

    void UpdateObservationHistory() {
        // 确保队列长度不超过历史帧数
        while (obs_history_.size() >= obs_history_num_) {
            obs_history_.pop_front();
        }
        obs_history_.push_back(current_observation_);
    
        // 重新堆叠观测：按时间顺序 [最旧 ... 最新]
        stacked_observation_.resize(obs_total_dim_);
        int idx = 0;
        for (const auto& historical_obs : obs_history_) {
            stacked_observation_.segment(idx * obs_dim_, obs_dim_) = historical_obs;
            idx++;
        }
        // 如果历史不足（初始时），用零填充剩余部分
        while (idx < obs_history_num_) {
            stacked_observation_.segment(idx * obs_dim_, obs_dim_).setZero();
            idx++;
        }
    }

    void DisplayPolicyInfo() override {
        std::cout << policy_name_ << " network test success" << std::endl;
        std::cout << "Policy file: " << policy_path_ << std::endl;
        std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_ << std::endl;
        std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
        std::cout << "kp   : " << kp_.transpose() << std::endl;
        std::cout << "kd   : " << kd_.transpose() << std::endl;
        std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
    }

    void OnEnter() override {
        action_.setZero(act_dim_);
        last_action_.setZero(act_dim_);
        obs_joint_pos_.setZero(act_dim_);
        obs_joint_vel_.setZero(act_dim_);

        current_observation_.setZero(obs_dim_);
        stacked_observation_.setZero(obs_total_dim_);
        obs_history_.clear();
        // 用零填充历史缓冲区（或使用当前初始观测重复填充）
        for (int i = 0; i < obs_history_num_; ++i) {
            obs_history_.push_back(VecXf::Zero(obs_dim_));
        }

        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        waving_active_ = false;
        wave_phase_ = 0;
        wave_step_counter_ = 0;
        smooth_action_.setZero(act_dim_);
        StartWavingRightArm();
    }

    void OnExit() override {
    }

    RobotAction GetRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        Vec3f user_cmd_vel = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
        Eigen::Vector3f vel_delta = user_cmd_vel - cmd_vel_input_;
        vel_delta_const_ << 0.02, 0.02, 0.04;
        for (int i = 0; i < 3; ++i) {
            if (fabs(vel_delta(i)) > vel_delta_const_(i)) {
                vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
            }
        }
        cmd_vel_input_ += vel_delta;
        // Vec3f cmd_vel = cmd_vel_input_.cwiseProduct(max_cmd_vel_);
        Vec3f cmd_vel;
        // X轴非对称映射
        float forward_scale = cmd_vel_input_(0);
        if (forward_scale >= 0.0f) {
            cmd_vel(0) = forward_scale * max_cmd_vel_(0);   // 正向最大 2.5
        } else {
            cmd_vel(0) = forward_scale * 1.0f;              // 负向最大 -1.0
        }
        // Y轴和Z轴保持对称
        cmd_vel(1) = cmd_vel_input_(1) * max_cmd_vel_(1);
        cmd_vel(2) = cmd_vel_input_(2) * max_cmd_vel_(2);
        // 每秒打印一次手柄速度
        static auto last_print_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count() >= 1000) {
            last_print_time = now;
            std::cout << "[Handstick] forward=" << user_cmd_vel(0) 
                    << " side=" << user_cmd_vel(1) 
                    << " turn=" << user_cmd_vel(2)
                    << " | cmd_vel=" << cmd_vel.transpose() << std::endl;
        }

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
                                last_action_;

        UpdateObservationHistory();

        const std::vector<int64_t> state_shape = {1, obs_total_dim_};
        auto state_tensor = Ort::Value::CreateTensor<float>(
            memory_info_, stacked_observation_.data(), stacked_observation_.size(), state_shape.data(), state_shape.size());

        ort_inputs_.clear();
        ort_inputs_.push_back(std::move(state_tensor));

        auto output_tensors = session_.Run(
            Ort::RunOptions{nullptr}, input_names_, ort_inputs_.data(), ort_inputs_.size(), output_names_, 1);
        action_ = Eigen::Map<VecXf>(output_tensors[0].GetTensorMutableData<float>(), act_dim_);


        // ========== 打招呼规划覆盖（若激活） ==========
        if (waving_active_) {
            const int right_shoulder_pitch_idx = 11;   // 右肩俯仰 (shoulder_x)
            const int right_shoulder_roll_idx  = 16;   // 右肩滚转 (不再摆动)
            const int right_shoulder_yaw_idx   = 20;   // 右肩偏航 (shoulder_z) 用于摆动
            const int right_elbow_pitch_idx    = 22;   // 右肘俯仰 (elbow_y)

            if (wave_phase_ == 0) {
                // 阶段0：屈肘并固定肩关节位置
                for (int idx : policy_to_arm_idx_) action_(idx) = 0.0f;

                // 右肩俯仰固定 -0.15 rad
                action_(right_shoulder_pitch_idx) = (-0.15f - dof_pos_default_(right_shoulder_pitch_idx)) / action_scale_;
                // 右肘屈曲 -0.7 rad
                action_(right_elbow_pitch_idx) = (-0.7f - dof_pos_default_(right_elbow_pitch_idx)) / action_scale_;
                // 右肩滚转保持默认（action=0）
                // 右肩偏航初始为0（不摆动）

                wave_step_counter_++;
                if (wave_step_counter_ >= wave_total_steps_phase0_) {
                    wave_phase_ = 1;
                    wave_step_counter_ = 0;
                    wave_swing_count_ = 0;               // 重置摆动周期计数
                    wave_swing_direction_ = false;       // 第一次移动到 -0.3
                }
            }
            else if (wave_phase_ == 1) {
                // 阶段1：保持肘和肩俯仰固定，摆动肩偏航
                for (int idx : policy_to_arm_idx_) action_(idx) = 0.0f;

                // 固定值
                action_(right_shoulder_pitch_idx) = (-0.15f - dof_pos_default_(right_shoulder_pitch_idx)) / action_scale_;
                action_(right_elbow_pitch_idx) = (-0.7f - dof_pos_default_(right_elbow_pitch_idx)) / action_scale_;

                // 右肩偏航摆动：-0.3 ↔ 0.3
                float shoulder_yaw_target = wave_swing_direction_ ? 0.3f : -0.3f;
                action_(right_shoulder_yaw_idx) = (shoulder_yaw_target - dof_pos_default_(right_shoulder_yaw_idx)) / action_scale_;

                wave_step_counter_++;
                if (wave_step_counter_ >= wave_total_steps_phase1_) {
                    wave_step_counter_ = 0;
                    wave_swing_direction_ = !wave_swing_direction_;   // 切换方向
                    wave_swing_count_++;   // 每完成2个半周期（一个来回）加1，这里半周期计数

                    // 完成3个完整来回（即6个半周期）后进入恢复阶段
                    if (wave_swing_count_ >= 6) {
                        wave_phase_ = 2;
                        wave_restore_counter_ = 0;
                        wave_start_action_ = smooth_action_;   // 原为 action_，改为 smooth_action_
                    }
                }
            }
            else if (wave_phase_ == 2) {
                // 阶段2：线性插值恢复所有手臂关节到默认位置（action=0）
                float t = (float)wave_restore_counter_ / wave_restore_steps_;
                t = std::min(1.0f, t);
                for (int idx : policy_to_arm_idx_) {
                    action_(idx) = wave_start_action_(idx) * (1.0f - t);
                }
                wave_restore_counter_++;
                if (wave_restore_counter_ >= wave_restore_steps_) {
                    waving_active_ = false;
                }
            }
        }

        // ========== 条件平滑：仅在打招呼时对手臂关节进行一阶低通滤波 ==========
        VecXf final_action(act_dim_);   // 最终输出的动作

        if (waving_active_) {
            // 打招呼激活：对手臂关节平滑，其他关节直接使用原始 action_
            if (first_smooth_run_) {
                smooth_action_ = action_;   // 初始化平滑历史
                first_smooth_run_ = false;
            }
            for (int i = 0; i < act_dim_; ++i) {
                // 判断当前关节是否属于手臂关节
                bool is_arm = false;
                for (int arm_idx : policy_to_arm_idx_) {
                    if (arm_idx == i) { is_arm = true; break; }
                }
                if (is_arm) {
                    // 一阶低通滤波
                    smooth_action_(i) = action_smooth_coef_ * action_(i) + (1.0f - action_smooth_coef_) * smooth_action_(i);
                    final_action(i) = smooth_action_(i);
                } else {
                    final_action(i) = action_(i);   // 非手臂关节无平滑
                }
            }
        } else {
            // 未激活打招呼：无平滑，直接使用原始 action_，并重置平滑状态
            final_action = action_;
            smooth_action_ = action_;      // 重置平滑历史，避免影响下次打招呼
            first_smooth_run_ = true;      // 下次打招呼重新初始化
        }

        // 观测历史中记录平滑后的动作
        last_action_ = final_action;

        RobotAction ra;
        ra.goal_joint_pos.setZero(act_dim_);
        ra.goal_joint_vel.setZero(act_dim_);
        ra.tau_ff.setZero(act_dim_);
        ra.kp.setZero(act_dim_);
        ra.kd.setZero(act_dim_);
        for (int i = 0; i < act_dim_; ++i) {
            const int idx = policy_to_output_idx_[i];
            ra.goal_joint_pos(i) = action_scale_ * final_action(idx) + dof_pos_default_(idx);
            ra.kp(i) = kp_(idx);
            ra.kd(i) = kd_(idx);
        }

        // last_action_ = action_;
        // RobotAction ra;
        // ra.goal_joint_pos.setZero(act_dim_);
        // ra.goal_joint_vel.setZero(act_dim_);
        // ra.tau_ff.setZero(act_dim_);
        // ra.kp.setZero(act_dim_);
        // ra.kd.setZero(act_dim_);


        // for (int i = 0; i < act_dim_; i++) {
        //     const int idx = policy_to_output_idx_[i];
        //     ra.goal_joint_pos(i) = action_scale_ * action_(idx) + dof_pos_default_(idx);
        //     // ra.goal_joint_pos(i) = dof_pos_default_(idx);
        //     ra.kp(i) = kp_(idx);
        //     ra.kd(i) = kd_(idx);
        // }



        // 添加打印（带关节名称）
        static const double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
        static const std::vector<std::string> joint_names = {
            "waist_yaw", "waist_roll", "waist_pitch",
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
            "left_elbow_pitch", "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll",
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
            "right_elbow_pitch", "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll",
            "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
            "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
            "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll"
        };
        static int print_counter = 0;
        if (++print_counter >= 100) {
            print_counter = 0;
            std::cout << "Goal joint positions (deg): ";
            for (int i = 0; i < act_dim_; ++i) {
                std::cout << joint_names[i] << ":" << ra.goal_joint_pos(i) * RAD_TO_DEG << " ";
            }
            std::cout << std::endl;
        }

        // 如果想要看数据解开下方注释
        // ra.goal_joint_pos.setZero();

        // if (run_cnt_ < 50) {
        //     for (int i = 0; i < policy_to_arm_idx_.size(); ++i) {
        //         const int arm_idx = policy_to_arm_idx_[i];
        //         float delta = dof_pos_default_(arm_idx) - obs_joint_pos_(arm_idx);
        //         delta = std::clamp(delta, -0.08f, 0.08f);
        //         ra.goal_joint_pos(arm_idx) = obs_joint_pos_(arm_idx) + delta;
        //     }
        // }
        ++run_cnt_;
        return ra;
    }

    void SetCmdMaxVel(const Vec3f& vel) {
        for (int i = 0; i < 3; ++i) {
            if (vel(i) < 0) {
                std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
            }
        }
        max_cmd_vel_ = vel;
    }

    static std::vector<int> GeneratePermutation(
        const std::vector<std::string>& from,
        const std::vector<std::string>& to,
        int default_index = 0)
    {
        std::unordered_map<std::string, int> idx_map;
        for (int i = 0; i < static_cast<int>(from.size()); ++i) {
            idx_map[from[i]] = i;
        }

        std::vector<int> perm;
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            if (it != idx_map.end()) {
                perm.push_back(it->second);
            } else {
                perm.push_back(default_index);
            }
        }

        return perm;
    }
};
