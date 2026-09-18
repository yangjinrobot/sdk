/**
 * @file common_policy_runner.hpp
 * @brief common policy runner for CR1
 * @author zhenglinqiang
 * @version 1.0
 * @date 2025-09-16
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#include "common_policy_runner.h"
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <fstream>

CommonPolicyRunner::CommonPolicyRunner(const std::string &policy_name, RobotName robot_name,
                                       const std::string &onnx_path,
                                       GraphOptimizationLevel graph_optimization_level)
    : PolicyRunnerBase(policy_name, robot_name) {
    onnx_path_ = onnx_path;
    VecXf kp_temp;
    kp_temp = VecXf::Zero(21);
    // 顺序与 policy_order 一致：左髋Y、右髋Y、腰Z、左髋X、右髋X、
    // 左肩Y、右肩Y、左髋Z、右髋Z、左肩X、右肩X、左膝、右膝、
    // 左肩Z、右肩Z、左踝Y、右踝Y、左肘、右肘、左踝X、右踝X。
    kp_temp << 300.0, 300.0,  // left_hip_y_joint, right_hip_y_joint
            200.0,            // waist_z_joint
            300.0, 300.0,    // left_hip_x_joint, right_hip_x_joint
            100.0, 100.0,    // left_shoulder_y_joint, right_shoulder_y_joint
            300.0, 300.0,    // left_hip_z_joint, right_hip_z_joint
            100.0, 100.0,    // left_shoulder_x_joint, right_shoulder_x_joint
            300.0, 300.0,    // left_knee_joint, right_knee_joint
            100.0, 100.0,    // left_shoulder_z_joint, right_shoulder_z_joint
            80.0, 80.0,      // left_ankle_y_joint, right_ankle_y_joint
            100.0, 100.0,    // left_elbow_joint, right_elbow_joint
            30.0, 30.0;      // left_ankle_x_joint, right_ankle_x_joint
    kp_ = kp_temp * 0.9;
    kd_ = VecXf::Zero(21);
    kd_ << 10, 10,           // left_hip_y_joint, right_hip_y_joint
            10,               // waist_z_joint
            10, 10,           // left_hip_x_joint, right_hip_x_joint
            5, 5,             // left_shoulder_y_joint, right_shoulder_y_joint
            10, 10,           // left_hip_z_joint, right_hip_z_joint
            5, 5,             // left_shoulder_x_joint, right_shoulder_x_joint
            10, 10,           // left_knee_joint, right_knee_joint
            5, 5,             // left_shoulder_z_joint, right_shoulder_z_joint
            3, 3,             // left_ankle_y_joint, right_ankle_y_joint
            5, 5,             // left_elbow_joint, right_elbow_joint
            1, 1;             // left_ankle_x_joint, right_ankle_x_joint
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
                "neck1_joint", "neck2_joint"
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
    }
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
    // for (const auto& robot : robot_order) {
    //     std::cout << robot << std::endl;
    // }
    num_dof_ = robot_order.size();
    robot2policy_idx = generate_permutation(robot_order, policy_order);
    // policyandpd2robot_idx: policy_order → output_order（21维输出空间内重排）
    policyandpd2robot_idx = generate_permutation(policy_order, output_order);
    policy2arm_idx = generate_permutation(policy_order, arm_joints_order);

    act_dim_ = 21;
    // POLICY_JOINT_ORDER.  Keep this identical to
    // sim2sim_dr02_21dof_waq.py / DR02_1WalkFlatEnvCfg.
    dof_pos_default_.setZero(act_dim_);
    dof_pos_default_ << -0.2, -0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.15, -0.15,
            0.4, 0.4, 0.0, 0.0, -0.2, -0.2,
            1.2, 1.2, 0.0, 0.0;

    decimation_ = 20;
    action_scale_ = 0.25;
    omega_scale_ = 1.0;
    dof_vel_scale_ = 0.05;
    gait_indices_ = 0.;
    cmd_vel_scale_ = Vec3f::Ones();
    // WAQ input: omega(3), gravity(3), command(3), q(21), dq(21),
    // previous action(21), sin/cos phase(4), phase ratio(2).
    obs_dim_ = 78;
    obs_history_num_ = 5;
    obs_total_dim_ = obs_dim_ + obs_dim_ * obs_history_num_;


    max_cmd_vel_.setZero(3);
    // observation_input_.setZero(1000, obs_dim_);
    // observation_input_ = readCSV("../data/observations.csv", 1000, obs_dim_).cast<float>();
    // std::cout<<observation_input_<<std::endl;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(graph_optimization_level);

    //初始化设置ONNXRUNTIME 的环境
    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    try {
        // 检查 ONNX 文件是否存在
        std::ifstream f(onnx_path_.c_str());
        if (!f.good()) {
            std::cerr << "error: ONNX model file not found: " << onnx_path_ << std::endl;
            exit(1);
        }
        f.close();
        // 从文件加载 ONNX 模型
        session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
        std::cout << "loaded " << policy_name_ << " ONNX model from: "
                  << onnx_path_ << std::endl;
    }
    catch (const std::exception &e) { std::cerr << "error loading policy as " << policy_name_ << "\n" << e.what(); }

    for (int i = 0; i < 10; i++) {//network pre-running TODO

        VecXf state_eigen;
        state_eigen.setRandom(obs_total_dim_);
        const std::vector<int64_t> state_shape = {1, obs_total_dim_};
        auto state_tensor = Ort::Value::CreateTensor<float>(memory_info_, state_eigen.data(),
                                                            state_eigen.size(), state_shape.data(), state_shape.size());
        ort_inputs_.clear();
        ort_inputs_.push_back(std::move(state_tensor));

        auto output_tensors = session_.Run(Ort::RunOptions{nullptr},
                                           input_names_,
                                           ort_inputs_.data(),
                                           ort_inputs_.size(),
                                           output_names_, 1);

        Eigen::Matrix<float, 21, 1> tVec{output_tensors[0].GetTensorMutableData<float>()};

        if (i == 9) {
            // std::cout << policy_name_ << " network test success" << std::endl;
        }
    }
}

CommonPolicyRunner::~CommonPolicyRunner() {
    std::cout << "CommonPolicyRunner " << policy_name_ << " destroyed" << std::endl;
}

std::vector<int> CommonPolicyRunner::generate_permutation(
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

void CommonPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " network test success" << std::endl;
    // std::cout << "name : " << policy_name_ << std::endl; 
    std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_
              << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

void CommonPolicyRunner::OnEnter() {
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
    cmd_vel_input_.setZero();
    cmd_pos_input_.setZero();
}

void CommonPolicyRunner::OnExit() {
    skip_arm_transition_ = false;
}

RobotAction CommonPolicyRunner::GetRobotAction(const RobotBasicState &ro, const UserCommand &uc) {
    Vec3f user_cmd_vel = Vec3f(uc.forward_vel_scale,
                               uc.side_vel_scale,
                               uc.turnning_vel_scale);
    // Keep the keyboard command normalized. The gait policy uses half-scale
    // reverse velocity, so apply that asymmetry here rather than at input.
    if (uc.forward_vel_scale < 0) user_cmd_vel[0] = 0.5f * uc.forward_vel_scale;
    if (uc.safe_control_mode != 0) {
        std::cout << "safe_control_mode: " << std::dec << uc.safe_control_mode << std::endl;
        run_cnt_record_ = run_cnt_;
    }
    if (run_cnt_ - run_cnt_record_ < 100) user_cmd_vel.setZero(3);

    Eigen::Vector3f vel_delta = user_cmd_vel - cmd_vel_input_;
    vel_delta_const_ << 0.02, 0.02, 0.04;
    for (int i = 0; i < 3; ++i) {
        if (fabs(vel_delta(i)) > vel_delta_const_(i)) vel_delta(i) = Sign(vel_delta(i)) * vel_delta_const_(i);
    }
    cmd_vel_input_ += vel_delta;
    Vec3f cmd_vel = cmd_vel_input_.cwiseProduct(max_cmd_vel_);
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
    if (run_cnt_ > 0) {
        constexpr float kPolicyDt = 0.020f;
        constexpr float kGaitCycle = 0.85f;
        constexpr float kLeftPhaseOffset = 0.38f;
        constexpr float kRightPhaseOffset = 0.88f;
        const float policy_time = static_cast<float>(run_cnt_) * kPolicyDt;
        const float phase_l = std::fmod(policy_time / kGaitCycle + kLeftPhaseOffset, 1.0f);
        const float phase_r = std::fmod(policy_time / kGaitCycle + kRightPhaseOffset, 1.0f);
        sin_phase_l = std::sin(2.0f * static_cast<float>(M_PI) * phase_l);
        sin_phase_r = std::sin(2.0f * static_cast<float>(M_PI) * phase_r);
        cos_phase_l = std::cos(2.0f * static_cast<float>(M_PI) * phase_l);
        cos_phase_r = std::cos(2.0f * static_cast<float>(M_PI) * phase_r);
    }
    const float phase_mask = (std::fabs(cmd_vel(0)) > 0.1f ||
                              std::fabs(cmd_vel(1)) > 0.1f ||
                              std::fabs(cmd_vel(2)) > 0.1f) ? 1.0f : 0.0f;

    current_observation_.setZero(obs_dim_);
    Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);

    for (int i = 0; i < act_dim_; i++) {
        obs_joint_pos_(i) = ro.joint_pos(robot2policy_idx[i]);
        obs_joint_vel_(i) = ro.joint_vel(robot2policy_idx[i]);
    }

    current_observation_ << omega_scale_ * ro.base_omega,
            project_gravity,
            cmd_vel,
            obs_joint_pos_ - dof_pos_default_,
            dof_vel_scale_ * obs_joint_vel_,
            last_action_,
            phase_mask * sin_phase_l, phase_mask * sin_phase_r,
            phase_mask * cos_phase_l, phase_mask * cos_phase_r,
            0.38f, 0.38f;
    current_observation_ = current_observation_.cwiseMax(-100.0f).cwiseMin(100.0f);

    VecXf obs_history_record = observation_history_.segment(obs_dim_, (obs_history_num_ - 1) * obs_dim_).eval();
    observation_history_.segment(0, (obs_history_num_ - 1) * obs_dim_) = obs_history_record;
    observation_history_.segment((obs_history_num_ - 1) * obs_dim_, obs_dim_) = current_observation_;

    // WAQ ONNX input layout: [history (oldest -> newest), current].
    // The history encoder consumes the first 5 * 78 values and the actor
    // consumes the final current 78-D observation together with its latent.
    observation_total_.segment(0, obs_dim_ * obs_history_num_) = observation_history_;
    observation_total_.segment(obs_dim_ * obs_history_num_, obs_dim_) = current_observation_;

    const std::vector<int64_t> state_shape = {1, obs_total_dim_};
    auto state_tensor = Ort::Value::CreateTensor<float>(memory_info_, observation_total_.data(),
                                                        observation_total_.size(), state_shape.data(),
                                                        state_shape.size());
    ort_inputs_.clear();
    ort_inputs_.push_back(std::move(state_tensor));


    auto output_tensors = session_.Run(Ort::RunOptions{nullptr},
                                       input_names_,
                                       ort_inputs_.data(),
                                       ort_inputs_.size(),
                                       output_names_, 1);
    action_ = Eigen::Map<VecXf>(output_tensors[0].GetTensorMutableData<float>(), act_dim_);
    action_ = action_.cwiseMax(-100.0f).cwiseMin(100.0f);

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
    // 手臂平滑过渡：切换到本策略的前100步，手臂从当前位置逐步移向默认位姿
    if (run_cnt_ < 100 && !skip_arm_transition_) {
        for (int i = 0; i < 8; ++i) {
            float delta = dof_pos_default_(policy2arm_idx[i]) - obs_joint_pos_(policy2arm_idx[i]);
            if (fabs(delta) > 0.08) delta = (delta > 0 ? 1 : -1) * 0.08;
            ra.goal_joint_pos(1 + i) = delta + obs_joint_pos_(policy2arm_idx[i]);
        }
    }
    return ra;
}


void CommonPolicyRunner::SetCmdMaxVel(const Vec3f &vel) {
    for (int i = 0; i < 3; ++i) {
        if (vel(i) < 0) {
            std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
        }
    }
    max_cmd_vel_ = vel;
}

Vec3f CommonPolicyRunner::GetCmdMaxVel() {
    return max_cmd_vel_;
}

void CommonPolicyRunner::SetPdGains(const VecXf& kp, const VecXf& kd) {
    if (kp.size() != act_dim_ || kd.size() != act_dim_) {
        throw std::invalid_argument("PD gain size must match policy action dimension");
    }
    kp_ = kp;
    kd_ = kd;
}
