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
                            -0.3,  -0.3,    // left_elbow_pitch, right_elbow_pitch
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
            // policy_path_ = fs::canonical(base / ".." / "policy" / "policy_arm.onnx").string(); # 正常双手拿起位姿
            // policy_path_ = fs::canonical(base / ".." / "policy" / "policy_armload.onnx").string(); // -0.15
            policy_path_ = fs::canonical(base / ".." / "policy" / "arm_03rad.onnx").string(); // -0.3

            
            session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        } catch (const std::exception& e) {
            std::cerr << "error loading policy as " << policy_name_ << "\n" << e.what();
            policy_path_ = "load failed";
        }
        
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

        last_action_ = action_;

        RobotAction ra;
        ra.goal_joint_pos.setZero(act_dim_);
        ra.goal_joint_vel.setZero(act_dim_);
        ra.tau_ff.setZero(act_dim_);
        ra.kp.setZero(act_dim_);
        ra.kd.setZero(act_dim_);
        for (int i = 0; i < act_dim_; ++i) {
            const int idx = policy_to_output_idx_[i];
            ra.goal_joint_pos(i) = action_scale_ * action_(idx) + dof_pos_default_(idx);
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
