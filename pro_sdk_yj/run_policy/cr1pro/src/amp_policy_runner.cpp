/**
 * @file amp_policy_runner.hpp
 * @brief amp policy runner for CR1
 * @author qiuxuejian
 * @version 1.0
 * @date 2026-10-06
 * 
 * @copyright Copyright (c) 2026  CETC21
 * 
 */

#include "amp_policy_runner.h"
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <fstream>

AmpPolicyRunner::AmpPolicyRunner(const std::string &policy_name, RobotName robot_name, const std::string &onnx_path) : PolicyRunnerBase(
        policy_name, robot_name) {
    onnx_path_ = onnx_path;
    VecXf kp_temp;
    kp_temp = VecXf::Zero(29);
    kp_temp << 300.0, 300.0,//0820 50k walk2run
               150.0,
               300.0, 300.0,//300
               150.0,
               100.0, 100.0,//150
               300.0,
               180.0, 180.0,//200
               50.0, 50.0,
               90.0, 90.0,//100
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0;
    kp_ = kp_temp * 1.0;
    kd_ = VecXf::Zero(29);
    kd_ << 12.0, 12.0,
            5.0,
            12.0, 12.0,
            5.0,
            6.0, 6.0,
            10.0,
            10.0, 10.0,
            2.0, 2.0,
            4.0, 4.0,
            2.0, 2.0,
            2.0, 2.0,
            2.0, 2.0,
            2.0, 2.0,
            2.0, 2.0,
            2.0, 2.0,
            2.0, 2.0;
    std::vector<std::string> robot_order;
    if (robot_name == RobotName::CR1PRO) {
        robot_order = {
                "waist_z_joint", "waist_x_joint", "waist_y_joint",
                "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
                "left_elbow_joint", "left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
                "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
                "right_elbow_joint", "right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint",
                "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint",
                "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
                "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint",
                "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint",
                // "neck1_joint", "neck2_joint"
        };
    } else {
        robot_order = {
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
        std::vector<std::string> policy_order = {
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
        std::vector<std::string> output_order = {
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
        std::vector<std::string> arm_joints_order = {
            "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
            "left_elbow_joint",
            "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
            "right_elbow_joint"
        };
    }
    std::vector<std::string> policy_order = {
            "left_hip_y_joint", "right_hip_y_joint",
            "waist_z_joint",
            "left_hip_x_joint", "right_hip_x_joint",
            "waist_x_joint",
            "left_hip_z_joint", "right_hip_z_joint",
            "waist_y_joint",
            "left_knee_joint", "right_knee_joint",
            "left_shoulder_y_joint", "right_shoulder_y_joint",
            "left_ankle_y_joint", "right_ankle_y_joint",
            "left_shoulder_x_joint", "right_shoulder_x_joint",
            "left_ankle_x_joint", "right_ankle_x_joint",
            "left_shoulder_z_joint", "right_shoulder_z_joint",
            "left_elbow_joint", "right_elbow_joint",
            "left_wrist_z_joint", "right_wrist_z_joint",
            "left_wrist_y_joint", "right_wrist_y_joint",
            "left_wrist_x_joint", "right_wrist_x_joint"
        };
    std::vector<std::string> output_order = {
            "waist_z_joint", "waist_x_joint", "waist_y_joint",
            "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
            "left_elbow_joint", "left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
            "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
            "right_elbow_joint", "right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint",
            "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint",
            "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
            "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint",
            "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint"
        };
    std::vector<std::string> arm_joints_order = {
            "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
            "left_elbow_joint", "left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
            "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
            "right_elbow_joint","right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint"
        };
    

    num_dof_ = robot_order.size();
    robot2policy_idx = generate_permutation(robot_order, policy_order);
    // policyandpd2robot_idx: policy_order → output_order（29维输出空间内重排）
    policyandpd2robot_idx = generate_permutation(policy_order, output_order);
    policy2arm_idx = generate_permutation(policy_order, arm_joints_order);

    act_dim_ = 29;
    dof_pos_default_.setZero(act_dim_);
    dof_pos_default_ << -0.1, -0.1, 0.0,
                         0.0,  0.0, 0.0,
                         0.0,  0.0, 0.0,
                         0.2,  0.2, 0.0,  0.0,
                        -0.1, -0.1, 0.15,-0.15,
                         0.0,  0.0, 0.0,  0.0,
                         1.0,  1.0,
                         0.0,  0.0,
                         0.0,  0.0,
                         0.0,  0.0;

    decimation_ = 20;
    action_scale_ = 0.25;
    omega_scale_ = 1.0;
    dof_vel_scale_ = 0.05;
    gait_indices_ = 0.;
    // cmd_vel_scale_ = Vec3f(2., 2., 0.25);
    cmd_vel_scale_ = Vec3f(1., 1., 1.);
    // obs_dim_ = 96;//wo gait
    obs_dim_ = 102;//w gait
    obs_history_num_ = 5;
    obs_total_dim_ = obs_dim_ * obs_history_num_;
    // obs_total_dim_ = obs_dim_ + obs_dim_ * obs_history_num_;

    max_cmd_vel_.setZero(3);
    // observation_input_.setZero(1000, obs_dim_);
    // observation_input_ = readCSV("../data/observations.csv", 1000, obs_dim_).cast<float>();
    // std::cout<<observation_input_<<std::endl;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);

    //初始化设置ONNXRUNTIME 的环境
    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    try { LoadModel(); }
    catch (const std::exception& error) { SetInferenceFault(error.what()); }
}

void AmpPolicyRunner::LoadModel() {
    model_loaded_ = false;
    if (onnx_path_.empty()) {
        const char* configured = std::getenv("DR2_MODEL_PATH");
        onnx_path_ = configured && *configured ? std::string(configured)
            : pro_policy::ResolveModelPath("amp_policy/policy_1022_75k.onnx");
    }
    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
    pro_policy::ValidateCopiedModel(session_, obs_total_dim_, false);
    model_loaded_ = true;
}

AmpPolicyRunner::~AmpPolicyRunner() {
    std::cout << "AmpPolicyRunner " << policy_name_ << " destroyed" << std::endl;
}

std::vector<int> AmpPolicyRunner::generate_permutation(
        const std::vector<std::string> &from,
        const std::vector<std::string> &to,
        int default_index) {
    std::unordered_map<std::string, int> idx_map;
    for (int i = 0; i < from.size(); ++i) {
        idx_map[from[i]] = i;
    }

    std::vector<int> perm;
    for (const auto &name: to) {
        auto it = idx_map.find(name);
        if (it != idx_map.end()) {
            perm.push_back(it->second);
        } else {
            perm.push_back(default_index);  // 如果找不到，就填默认值
        }
    }

    return perm;
}

void AmpPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " network test success" << std::endl;
    // std::cout << "name : " << policy_name_ << std::endl; 
    std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_
              << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

void AmpPolicyRunner::OnEnter() {
    ResetInferenceFault();
    if (!model_loaded_) LoadModel();
    action_.setZero(act_dim_);
    last_action_.setZero(act_dim_);
    obs_joint_pos_.setZero(act_dim_);
    obs_joint_vel_.setZero(act_dim_);

    current_observation_.setZero(obs_dim_);
    observation_history_.setZero(obs_dim_ * obs_history_num_);
    observation_total_.setZero(obs_total_dim_);

    run_cnt_ = 0;
    run_cnt_record_ = 0;
    gait_indices_ = 0.;
    gait_count_ = 0;
    cmd_vel_input_.setZero();
    cmd_pos_input_.setZero();

}

void AmpPolicyRunner::OnExit() {
    skip_arm_transition_ = false;
}

RobotAction AmpPolicyRunner::ComputeRobotAction(const RobotBasicState &ro, const UserCommand &uc) {
    // 步态相位计算（与原始版本一致，当前模型未使用sin/cos作为观测输入）
    // Eigen::VectorXf gaits_commands;
    // gaits_commands.setZero(3);
    // gaits_commands << 1.5, 0.5, 0.5;
    // gait_indices_ = float(gait_indices_ + gaits_commands[0] * 0.02) - floor(float(gait_indices_ + gaits_commands[0] * 0.02));
    // float sin_pos = sin(2 * M_PI * gait_indices_);
    // float cos_pos = cos(2 * M_PI * gait_indices_);
    // qxj步态相位计算
    // --- Gait phase observation (must match the training implementation) ---
    const float gait_t_cycle = static_cast<float>(gait_count_) * gait_dt_ / gait_cycle_;
    Eigen::Vector2f gait_sin, gait_cos, gait_ratio;
    for (int i = 0; i < 2; ++i) {                       // 0 = 左腿, 1 = 右腿
        const float offset = (i == 0) ? gait_phase_offset_l_ : gait_phase_offset_r_;
        const float ratio  = (i == 0) ? gait_air_ratio_l_   : gait_air_ratio_r_;
        const float phase  = std::fmod(gait_t_cycle + offset, 1.0f);   // (t + offset) % 1
        gait_sin(i)   = std::sin(2.0f * M_PI * phase);
        gait_cos(i)   = std::cos(2.0f * M_PI * phase);
        gait_ratio(i) = ratio;
    }
    Vec3f user_cmd_vel = Vec3f(uc.forward_vel_scale,
                               uc.side_vel_scale,
                               uc.turnning_vel_scale);
    // std::cout<<"cmd:"<<user_cmd_vel.transpose()<<std::endl;
    if (uc.forward_vel_scale < 0)user_cmd_vel[0] = 0.3 * uc.forward_vel_scale;
    if (uc.safe_control_mode != 0) {
        std::cout << "safe_control_mode: " << std::dec << uc.safe_control_mode << std::endl;
        run_cnt_record_ = run_cnt_;
    }
    // if (run_cnt_ - run_cnt_record_ < 100) user_cmd_vel.setZero(3);

    Eigen::Vector3f vel_delta = user_cmd_vel - cmd_vel_input_;
    vel_delta_const_ << 0.02, 0.02, 0.04;
    for (int i = 0; i < 3; ++i) {
        if (fabs(vel_delta(i)) > vel_delta_const_(i)) vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
    }
    cmd_vel_input_ += vel_delta;
    Vec3f cmd_vel = ApplyCommandDeadband(cmd_vel_input_.cwiseProduct(max_cmd_vel_));

    //******qxj_amp */
    if(run_cnt_ == 0){
        const Eigen::VectorXf zero_vec = Eigen::VectorXf::Zero(obs_dim_);
        obs_history_deque.clear();
        for (int i = 0; i < obs_history_num_; ++i) 
        obs_history_deque.push_back(zero_vec);
    }

    current_observation_.setZero(obs_dim_);
    Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);

    for (int i = 0; i < act_dim_; i++) {
        obs_joint_pos_(i) = ro.joint_pos(robot2policy_idx[i]);
        obs_joint_vel_(i) = ro.joint_vel(robot2policy_idx[i]);
    }
    // w/o gait
    // current_observation_ << omega_scale_ * ro.base_omega,
    //                         project_gravity,
    //                         cmd_vel.cwiseProduct(cmd_vel_scale_),
    //                         obs_joint_pos_ - dof_pos_default_,
    //                         dof_vel_scale_ * obs_joint_vel_,
    //                         last_action_;
    // w gait
    current_observation_ << omega_scale_ * ro.base_omega,
                            project_gravity,
                            cmd_vel.cwiseProduct(cmd_vel_scale_),
                            obs_joint_pos_ - dof_pos_default_,
                            dof_vel_scale_ * obs_joint_vel_,
                            last_action_,
                            gait_sin,
                            gait_cos,
                            gait_ratio;

    obs_history_deque.push_back(current_observation_);
    obs_history_deque.pop_front();
    Eigen::VectorXf obs_his_temp(obs_dim_ * obs_history_num_);
    int current_pos = 0;
    for (int i = 0; i < obs_history_num_; ++i) {
        int vec_size = obs_history_deque[i].size();
        obs_his_temp.segment(current_pos, vec_size) = obs_history_deque[i];
        current_pos += vec_size;
    }

    pro_policy::ValidateObservation(obs_his_temp, obs_total_dim_);
    const std::vector<int64_t> state_shape = {1, obs_total_dim_};
    auto state_tensor = Ort::Value::CreateTensor<float>(memory_info_, obs_his_temp.data(),
                                                            obs_his_temp.size(), state_shape.data(), state_shape.size());
    ort_inputs_.clear();
    ort_inputs_.push_back(std::move(state_tensor));


    auto output_tensors = session_.Run(Ort::RunOptions{nullptr},
                                       input_names_,
                                       ort_inputs_.data(),
                                       ort_inputs_.size(),
                                       output_names_, 1);
    action_ = pro_policy::ReadAction(output_tensors, act_dim_);

    last_action_ = action_;
    RobotAction ra;
    ra.goal_joint_pos = VecXf::Zero(act_dim_);
    ra.goal_joint_vel = VecXf::Zero(act_dim_);
    ra.tau_ff = VecXf::Zero(act_dim_);
    ra.kp = VecXf::Zero(act_dim_);
    ra.kd = VecXf::Zero(act_dim_);
    for (int i = 0; i < act_dim_; i++) {
        ra.goal_joint_pos(i) =
                action_scale_ * action_(policyandpd2robot_idx[i]) + dof_pos_default_(policyandpd2robot_idx[i]);
        ra.kp(i) = kp_(policyandpd2robot_idx[i]);
        ra.kd(i) = kd_(policyandpd2robot_idx[i]);
    }

    ++run_cnt_;
    ++gait_count_;
    // 手臂平滑过渡：切换到本策略的前100步，手臂从当前位置逐步移向默认位姿
    // if (run_cnt_ < 100 && !skip_arm_transition_) {
    //     for (int i = 0; i < 14; ++i) {
    //         float delta = dof_pos_default_(policy2arm_idx[i]) - obs_joint_pos_(policy2arm_idx[i]);
    //         if (fabs(delta) > 0.08) delta = (delta > 0 ? 1 : -1) * 0.08;
    //         ra.goal_joint_pos(3 + i) = delta + obs_joint_pos_(policy2arm_idx[i]);
    //     }
    // }
    return ra;
}


void AmpPolicyRunner::SetCmdMaxVel(const Vec3f &vel) {
    for (int i = 0; i < 3; ++i) {
        if (vel(i) < 0) {
            std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
        }
    }
    max_cmd_vel_ = vel;
}

Vec3f AmpPolicyRunner::GetCmdMaxVel() {
    return max_cmd_vel_;
}

