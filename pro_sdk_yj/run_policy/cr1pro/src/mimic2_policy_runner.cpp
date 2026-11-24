/**
 * @file mimic2_policy_runner.cpp
 * @brief mimic2 (dance) policy runner for CR1
 * @author qiuxuejian
 * @version 1.0
 * @date 2026-10-06
 * 
 * @copyright Copyright (c) 2026  CETC21
 * 
 */

#include "mimic2_policy_runner.h"
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>

Mimic2PolicyRunner::Mimic2PolicyRunner(const std::string &policy_name, RobotName robot_name, const std::string &onnx_path) : PolicyRunnerBase(
        policy_name, robot_name) {
    onnx_path_ = onnx_path;
    VecXf kp_temp;
    kp_temp = VecXf::Zero(29);
    kp_temp << 100.0, 100.0,
               100.0,
               100.0, 100.0,
               100.0,
               100.0, 100.0,
               100.0,
               100.0, 100.0,
               50.0, 50.0,
               70.0, 70.0,
               50.0, 50.0,
               30.0, 30.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0,
               50.0, 50.0;
    kp_ = kp_temp * 1.0;
    kd_ = VecXf::Zero(29);
    kd_ << 3.0, 3.0,
            3,//10.0,
            3.0, 3.0,
            3,//10.0,
            3.0, 3.0, 
            3,//20.0,
            3.0, 3.0,
            5.0, 5.0,
            2.0, 2.0,
            5.0, 5.0,
            1.0, 1.0,
            5.0, 5.0,
            5.0, 5.0,
            5.0, 5.0,
            5.0, 5.0,
            5.0, 5.0;
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
        throw std::invalid_argument("Mimic2 requires RobotName::CR1PRO / 29 policy joints");
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
                         1.57,  1.57,
                         0.0,  0.0,
                         0.0,  0.0,
                         0.0,  0.0;

    decimation_ = 20;
    action_scale_ = 0.5;
    omega_scale_ = 1.0;
    dof_vel_scale_ = 1.0;
    gait_indices_ = 0.;
    cmd_vel_scale_ = Vec3f(1., 1., 1.);
    obs_dim_ = 158;
    motion_command_dim_=65;
    obs_history_num_ = 1;
    time_step_dim_ = 1;
    obs_total_dim_ = obs_dim_ * obs_history_num_;

    max_cmd_vel_.setZero(3);
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);

    //初始化设置ONNXRUNTIME 的环境
    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, policy_name_.data());
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    try { LoadModel(); }
    catch (const std::exception& error) { SetInferenceFault(error.what()); }
}

void Mimic2PolicyRunner::LoadModel() {
    model_loaded_ = false;
    if (onnx_path_.empty()) {
        const char* configured = std::getenv("DR2_MIMIC2_MODEL_PATH");
        onnx_path_ = configured && *configured ? std::string(configured)
            : pro_policy::ResolveModelPath("mimic_policy/box_fight.onnx");
    }
    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
    pro_policy::ValidateCopiedModel(session_, obs_total_dim_, true);
    model_loaded_ = true;
    std::cout << "loaded Mimic2 ONNX model from: " << onnx_path_ << std::endl;
}

Mimic2PolicyRunner::~Mimic2PolicyRunner() {
    std::cout << "Mimic2PolicyRunner " << policy_name_ << " destroyed" << std::endl;
}

std::vector<int> Mimic2PolicyRunner::generate_permutation(
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

void Mimic2PolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " mimic2 network test success" << std::endl;
    std::cout << "dim  : " << obs_dim_ << " " << obs_history_num_ << " " << obs_total_dim_ << " " << act_dim_
              << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

void Mimic2PolicyRunner::OnEnter() {
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
    cmd_vel_input_.setZero();
    cmd_pos_input_.setZero();

}

void Mimic2PolicyRunner::OnExit() {
    skip_arm_transition_ = false;
}

RobotAction Mimic2PolicyRunner::ComputeRobotAction(const RobotBasicState &ro, const UserCommand &uc) {
    
    //******qxj */
    if(run_cnt_ == 0){
        const Eigen::VectorXf zero_vec = Eigen::VectorXf::Zero(obs_dim_);
        obs_history_deque.clear();
        for (int i = 0; i < obs_history_num_; ++i) 
        obs_history_deque.push_back(zero_vec);
        ////***************************************初始化t=0 */
        VecXf state_eigen(obs_dim_ * obs_history_num_);
        state_eigen.setZero(obs_dim_ * obs_history_num_);
        const std::vector<int64_t> state_shape_i = {1, obs_dim_* obs_history_num_};
        auto state_tensor_i = Ort::Value::CreateTensor<float>(memory_info_, state_eigen.data(),
                                                            state_eigen.size(), state_shape_i.data(), state_shape_i.size());           
        // 转换时间步数据
        std::vector<float> time_step_data_first{0.0f};
        const std::vector<int64_t> time_step_shape_first= {1, 1};
        auto time_step_tensor_first = Ort::Value::CreateTensor<float>(memory_info_, time_step_data_first.data(),
                                                                    time_step_data_first.size(), time_step_shape_first.data(), time_step_shape_first.size());                          
        ort_inputs_.clear();
        ort_inputs_.push_back(std::move(state_tensor_i));
        ort_inputs_.push_back(std::move(time_step_tensor_first));

        auto output_tensors_first = session_.Run(Ort::RunOptions{ nullptr },
                                            input_names_,
                                            ort_inputs_.data(),
                                            ort_inputs_.size(),
                                            output_names_, 7);
        
        pro_policy::ValidateMimicOutputs(output_tensors_first);
        Eigen::Map<Eigen::VectorXf> joint_pos_first(output_tensors_first[1].GetTensorMutableData<float>(), 
                                            output_tensors_first[1].GetTensorTypeAndShapeInfo().GetElementCount());

        Eigen::Map<Eigen::VectorXf> joint_vel_first(output_tensors_first[2].GetTensorMutableData<float>(), 
                                            output_tensors_first[2].GetTensorTypeAndShapeInfo().GetElementCount());
        
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> body_pos_first(
            output_tensors_first[3].GetTensorMutableData<float>(), 
            output_tensors_first[3].GetTensorTypeAndShapeInfo().GetShape()[1], 3); 
        
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor>> body_quat_first(
            output_tensors_first[4].GetTensorMutableData<float>(), 
            output_tensors_first[4].GetTensorTypeAndShapeInfo().GetShape()[1], 4);
        
        motion_command = Eigen::VectorXf::Zero(motion_command_dim_);
        motion_command <<   joint_pos_first, //29
                            joint_vel_first, //29
                            body_pos_first.block(1, 0, 1, body_pos_first.cols()).transpose(),//3
                            body_quat_first.block(1, 0, 1, body_quat_first.cols()).transpose();//4
    }

    current_observation_.setZero(obs_dim_);
    Vec3f project_gravity = ro.base_rot_mat.transpose() * Vec3f(0., 0., -1);

    for (int i = 0; i < act_dim_; i++) {
        obs_joint_pos_(i) = ro.joint_pos(robot2policy_idx[i]);
        obs_joint_vel_(i) = ro.joint_vel(robot2policy_idx[i]);
    }

    current_observation_ << motion_command,//65*
                            omega_scale_ * ro.base_omega,
                            project_gravity,
                            obs_joint_pos_ - dof_pos_default_,
                            dof_vel_scale_ * obs_joint_vel_,
                            last_action_;

    pro_policy::ValidateObservation(current_observation_, obs_total_dim_);
    const std::vector<int64_t> state_shape = {1, obs_dim_ * obs_history_num_};
    auto state_tensor = Ort::Value::CreateTensor<float>(memory_info_, current_observation_.data(),
                                                        current_observation_.size(), state_shape.data(), state_shape.size());
    // 转换时间步数据
    std::vector<float> time_step_data_ = {static_cast<float>(run_cnt_ + 1)};
    const std::vector<int64_t> time_step_shape_= {1, time_step_dim_};
    
    auto time_step_tensor_ = Ort::Value::CreateTensor<float>(memory_info_, time_step_data_.data(),
                                                                time_step_data_.size(), time_step_shape_.data(), time_step_shape_.size());
    ort_inputs_.clear();
    ort_inputs_.push_back(std::move(state_tensor));
    ort_inputs_.push_back(std::move(time_step_tensor_));
    
    auto output_tensors_ = session_.Run(Ort::RunOptions{ nullptr },
                                        input_names_,
                                        ort_inputs_.data(),
                                        ort_inputs_.size(),
                                        output_names_, 7);
    
    pro_policy::ValidateMimicOutputs(output_tensors_);
    // 获取动作输出
    Eigen::Map<Eigen::VectorXf> joint_pos_bm(output_tensors_[1].GetTensorMutableData<float>(), 
                                            output_tensors_[1].GetTensorTypeAndShapeInfo().GetElementCount());

    Eigen::Map<Eigen::VectorXf> joint_vel_bm(output_tensors_[2].GetTensorMutableData<float>(), 
                                        output_tensors_[2].GetTensorTypeAndShapeInfo().GetElementCount());

    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> body_pos_bm(
        output_tensors_[3].GetTensorMutableData<float>(), 
        output_tensors_[3].GetTensorTypeAndShapeInfo().GetShape()[1], 3); 
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor>> body_quat_bm(
        output_tensors_[4].GetTensorMutableData<float>(), 
        output_tensors_[4].GetTensorTypeAndShapeInfo().GetShape()[1], 4);
    
    action_ = Eigen::Map<VecXf>(output_tensors_[0].GetTensorMutableData<float>(), act_dim_);

    last_action_ = action_;
    motion_command.setZero(motion_command_dim_);
    motion_command <<   joint_pos_bm, //29
                        joint_vel_bm, //29
                        body_pos_bm.block(1, 0, 1, body_pos_bm.cols()).transpose(),//3
                        body_quat_bm.block(1, 0, 1, body_quat_bm.cols()).transpose();//4


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
    if(run_cnt_>1150){
        run_cnt_ = 1150;//simpson
    }
    return ra;
}


void Mimic2PolicyRunner::SetCmdMaxVel(const Vec3f &vel) {
    for (int i = 0; i < 3; ++i) {
        if (vel(i) < 0) {
            std::cerr << policy_name_ << " max_vel " << i << " set error" << std::endl;
        }
    }
    max_cmd_vel_ = vel;
}

Vec3f Mimic2PolicyRunner::GetCmdMaxVel() {
    return max_cmd_vel_;
}
