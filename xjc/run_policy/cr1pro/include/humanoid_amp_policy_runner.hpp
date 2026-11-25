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
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <unordered_map>
#include <deque>

namespace {
inline Vec7f AmpArmWaypoint(float a, float b, float c, float d, float e = 0.0f, float f = 0.0f, float g = 0.0f) {
    Vec7f v;
    v << a, b, c, d, e, f, g;
    return v;
}
}  // namespace

class HumanoidAmpPolicyRunner : public PolicyRunnerBase {
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
    float total_elapsed_time_ = 0.0f;

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
    bool has_initial_arm_targets_ = false;

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
    bool policy_loaded_ = false;

    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};

    static constexpr float kSignalDt = 0.001f;
    

    static float SmoothStep(float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    }

    static Vec7f SmoothInterpolate(const Vec7f& start, const Vec7f& end, float t) {
        return start + SmoothStep(t) * (end - start);
    }

    void UpdateArmPreset(const UserCommand& uc) {
        const bool preset_cmd_edge = uc.arm_preset_seq != last_arm_preset_seq_;
        const bool preset_1_edge = uc.arm_preset_1 && !last_arm_preset_1_;
        const bool preset_2_edge = uc.arm_preset_2 && !last_arm_preset_2_;
        const bool preset_3_edge = uc.arm_preset_3 && !last_arm_preset_3_;
        const bool preset_4_edge = uc.arm_preset_4 && !last_arm_preset_4_;
        const bool preset_5_edge = uc.arm_preset_5 && !last_arm_preset_5_;
        const bool preset_6_edge = uc.arm_preset_6 && !last_arm_preset_6_;

        const Vec7f pose_home_left = AmpArmWaypoint(0.0f, 0.2f, 0.0f, 1.4f);
        const Vec7f pose_home_right = AmpArmWaypoint(0.0f, -0.2f, 0.0f, 1.4f);
        const Vec7f pose_a_left = AmpArmWaypoint(0.0f, 0.5f, 0.0f, 0.0f);
        const Vec7f pose_a_right = AmpArmWaypoint(0.0f, -0.5f, 0.0f, 0.0f);
        const Vec7f pose_b_left = AmpArmWaypoint(0.0f, 0.0f, 0.0f, 0.0f);
        const Vec7f pose_b_right = AmpArmWaypoint(0.0f, 0.0f, 0.0f, 0.0f);
        const Vec7f pose_c_left = AmpArmWaypoint(0.0f, 0.0f, 0.0f, -0.2f);
        const Vec7f pose_c_right = AmpArmWaypoint(0.0f, 0.0f, 0.0f, -0.2f);

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

    void UpdateArmTargets(const RobotBasicState& ro, const UserCommand& uc) {
        if (uc.disable_arm_motion_planning) {
            current_arm_target_left_ = ro.joint_pos.segment(3, 7);
            current_arm_target_right_ = ro.joint_pos.segment(10, 7);
            arm_interp_active_ = false;
            last_arm_preset_1_ = last_arm_preset_2_ = last_arm_preset_3_ = false;
            last_arm_preset_4_ = last_arm_preset_5_ = last_arm_preset_6_ = false;
            last_arm_preset_seq_ = uc.arm_preset_seq;
            return;
        }

        UpdateArmPreset(uc);
    }

public:
    HumanoidAmpPolicyRunner(const std::string& policy_name, RobotName robot_name) : PolicyRunnerBase(policy_name, robot_name) {
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
            //waist joints
            "waist_yaw_joint", 
            "waist_roll_joint", 
            "waist_pitch_joint",
            //left arm joints
            "left_shoulder_pitch_joint", 
            "left_shoulder_roll_joint", 
            "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint", 
            "left_wrist_yaw_joint", 
            "left_wrist_pitch_joint", 
            "left_wrist_roll_joint",
            //right arm joints
            "right_shoulder_pitch_joint", 
            "right_shoulder_roll_joint", 
            "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint", 
            "right_wrist_yaw_joint", 
            "right_wrist_pitch_joint", 
            "right_wrist_roll_joint",
            //left leg joints
            "left_hip_pitch_joint", 
            "left_hip_roll_joint", 
            "left_hip_yaw_joint",
            "left_knee_pitch_joint", 
            "left_ankle_pitch_joint", 
            "left_ankle_roll_joint",
            //right leg joints
            "right_hip_pitch_joint", 
            "right_hip_roll_joint", 
            "right_hip_yaw_joint",
            "right_knee_pitch_joint", 
            "right_ankle_pitch_joint", 
            "right_ankle_roll_joint"
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
            //waist
            "waist_yaw_joint", 
            "waist_roll_joint", 
            "waist_pitch_joint",
            //left arm
            "left_shoulder_pitch_joint", 
            "left_shoulder_roll_joint", 
            "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint", 
            "left_wrist_yaw_joint", 
            "left_wrist_pitch_joint", 
            "left_wrist_roll_joint",
            //right arm
            "right_shoulder_pitch_joint", 
            "right_shoulder_roll_joint", 
            "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint", 
            "right_wrist_yaw_joint", 
            "right_wrist_pitch_joint", 
            "right_wrist_roll_joint",
            //left leg
            "left_hip_pitch_joint", 
            "left_hip_roll_joint", 
            "left_hip_yaw_joint",
            "left_knee_pitch_joint", 
            "left_ankle_pitch_joint", 
            "left_ankle_roll_joint",
            //right leg
            "right_hip_pitch_joint", 
            "right_hip_roll_joint", 
            "right_hip_yaw_joint",
            "right_knee_pitch_joint", 
            "right_ankle_pitch_joint", 
            "right_ankle_roll_joint"
        };

        std::vector<std::string> arm_joints_order = {
            //left arm
            "left_shoulder_pitch_joint", 
            "left_shoulder_roll_joint", 
            "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint", 
            "left_wrist_yaw_joint", 
            "left_wrist_pitch_joint", 
            "left_wrist_roll_joint",

            //right arm
            "right_shoulder_pitch_joint", 
            "right_shoulder_roll_joint", 
            "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint", 
            "right_wrist_yaw_joint", 
            "right_wrist_pitch_joint", 
            "right_wrist_roll_joint"
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

        current_arm_target_left_ = AmpArmWaypoint(0.0f, 0.15f, 0.0f, 0.0f);
        current_arm_target_right_ = AmpArmWaypoint(0.0f, -0.15f, 0.0f, 0.0f);

        max_cmd_vel_.setZero(3);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        ort_inputs_.reserve(1);

        env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
        memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

        try {
            policy_path_ = pro_policy::ResolveModelPath("amp_policy/policy_arm.onnx");
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

    ~HumanoidAmpPolicyRunner() override {
        std::cout << "HumanoidAmpPolicyRunner " << policy_name_ << " destroyed" << std::endl;
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
        std::cout << policy_name_ << (policy_loaded_ ? " network test success" : " network load failed") << std::endl;
        std::cout << "Policy file: " << policy_path_ << std::endl;
        std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_ << std::endl;
        std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
        std::cout << "kp   : " << kp_.transpose() << std::endl;
        std::cout << "kd   : " << kd_.transpose() << std::endl;
        std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
    }

    bool IsReady() const override {
        return policy_loaded_ && !HasInferenceFault();
    }

    void SetInitialArmTargets(const Vec7f& left, const Vec7f& right) {
        if (!left.allFinite() || !right.allFinite())
            throw std::invalid_argument("AMP initial arm targets must be finite");
        initial_arm_target_left_ = left;
        initial_arm_target_right_ = right;
        has_initial_arm_targets_ = true;
    }

    void OnEnter() override {
        if (policy_loaded_) ResetInferenceFault();
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
        total_elapsed_time_ = 0.0f;
        current_arm_target_left_ = has_initial_arm_targets_ ? initial_arm_target_left_
            : AmpArmWaypoint(0.0f, 0.15f, 0.0f, 1.4f);
        current_arm_target_right_ = has_initial_arm_targets_ ? initial_arm_target_right_
            : AmpArmWaypoint(0.0f, -0.15f, 0.0f, 1.4f);
        has_initial_arm_targets_ = false;
        arm_interp_active_ = false;
        last_arm_preset_1_ = last_arm_preset_2_ = last_arm_preset_3_ = false;
        last_arm_preset_4_ = last_arm_preset_5_ = last_arm_preset_6_ = false;
        last_arm_preset_seq_ = 0;
    }

    void OnExit() override {
    }

    RobotAction ComputeRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        if (!policy_loaded_) {
            throw std::runtime_error("policy is not loaded: " + policy_path_);
        }

        total_elapsed_time_ += decimation_ * kSignalDt;
        UpdateArmTargets(ro, uc);

        Vec3f user_cmd_vel = Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
        Eigen::Vector3f vel_delta = user_cmd_vel - cmd_vel_input_;
        vel_delta_const_ << 0.02, 0.02, 0.04;
        for (int i = 0; i < 3; ++i) {
            if (fabs(vel_delta(i)) > vel_delta_const_(i)) {
                vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
            }
        }
        cmd_vel_input_ += vel_delta;
        Vec3f cmd_vel = cmd_vel_input_.cwiseProduct(max_cmd_vel_);

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

        pro_policy::ValidateObservation(current_observation_, obs_dim_);
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

        for (int i = 0; i < 7; ++i) {
            ra.goal_joint_pos(3 + i) = current_arm_target_left_(i);
            ra.goal_joint_pos(10 + i) = current_arm_target_right_(i);
        }

        last_action_ = action_;
        for (int i = 0; i < 7; ++i) {
            const int left_policy_idx = policy_to_arm_idx_[i];
            const int right_policy_idx = policy_to_arm_idx_[i + 7];
            last_action_(left_policy_idx) =
                (current_arm_target_left_(i) - dof_pos_default_(left_policy_idx)) / action_scale_;
            last_action_(right_policy_idx) =
                (current_arm_target_right_(i) - dof_pos_default_(right_policy_idx)) / action_scale_;
        }

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
