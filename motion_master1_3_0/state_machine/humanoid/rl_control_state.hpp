/**
 * @file rl_control_state.hpp
 * @brief rl policy runnning state
 * @author zhenglinqiang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_RL_CONTROL_STATE_HPP_
#define HUMANOID_RL_CONTROL_STATE_HPP_

#include "state_base.h"
#include "crx_model_inference_external.hpp"
#include "safe_controller/safe_controller.hpp"
#include <atomic>
#include <chrono>

using namespace std::chrono;
namespace humanoid {
    class RLControlState : public StateBase {
    private:
        RobotBasicState rbs_;
        std::mutex rbs_mutex_;
        std::atomic<int> state_run_cnt_{0};

        std::shared_ptr<ModelInferenceBase> policy_ptr_;
        std::shared_ptr<CRXModelInferenceExternal> crx_policy_runner_;
        RobotMotionState feedback_state_ = RobotMotionState::RLControlMode;
        StateName own_state_name_ = StateName::kRLControl;

        std::thread run_policy_thread_;
        std::atomic<bool> start_flag_{true};

        MatXf joint_cmd;
        bool heading_mode_ = false;
        bool is_first_enter_ = true;
        float yaw_target_ = 0.0f;
        float yaw_delta_ = 0.0f;

        float policy_cost_time_ = 1;
        int acc_rot_count = 0;

        DrLogger logger_{"RLControl"};

        // Joint index ranges for upper-body fault handling
        int waist_start_, waist_end_;
        int larm_start_, larm_end_;
        int rarm_start_, rarm_end_;
        int neck_start_, neck_end_;

        // Upper body fault recovery observation
        static constexpr float kRecoveryStableTime = 3.0f;
        float upper_body_recovery_start_ = 0;
        bool upper_body_recovered_ = false;

        void InitJointIndexRanges() {
            int offset = 0;
            waist_start_ = offset;
            waist_end_ = offset + cp_ptr_->waist_dof_num_ - 1;
            offset += cp_ptr_->waist_dof_num_;

            larm_start_ = offset;
            larm_end_ = offset + cp_ptr_->arm_dof_num_ - 1;
            offset += cp_ptr_->arm_dof_num_;

            rarm_start_ = offset;
            rarm_end_ = offset + cp_ptr_->arm_dof_num_ - 1;
            offset += cp_ptr_->arm_dof_num_;

            // Skip legs: lleg + rleg
            offset += cp_ptr_->leg_dof_num_ * 2;

            neck_start_ = offset;
            neck_end_ = offset + cp_ptr_->neck_dof_num_ - 1;
        }

        // 上半身故障保护：清零故障关节除kd外的所有指令列（kp/pos_d/vel_d/tau_ff）
        void ApplyUpperBodyFaultProtection(MatXf& cmd) {
            uint32_t fault_mask = sc_ptr_->GetFaultMask();
            if (fault_mask == 0) return;

            BodyFaultGroup group = sc_ptr_->ClassifyFault(fault_mask);

            auto zero_except_kd = [&cmd](int start, int end) {
                for (int i = start; i <= end; i++) {
                    cmd(i, 0) = 0;  // kp
                    cmd(i, 1) = 0;  // pos_d
                    cmd(i, 3) = 0;  // vel_d
                    cmd(i, 4) = 0;  // tau_ff
                }
            };

            if (group == BodyFaultGroup::UPPER_LIMB) {
                zero_except_kd(larm_start_, larm_end_);
                zero_except_kd(rarm_start_, rarm_end_);
            }
            if (group == BodyFaultGroup::WAIST_Z) {
                zero_except_kd(waist_start_, waist_end_);
            }
            if (group == BodyFaultGroup::HEAD && cp_ptr_->neck_dof_num_ > 0) {
                zero_except_kd(neck_start_, neck_end_);
            }
        }

        void init_rbs_() {
            joint_cmd = Eigen::MatrixXf::Zero(cp_ptr_->dof_num_, 5);
            rbs_.flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
            rbs_.flt_joint_vel_mat = Eigen::MatrixXf::Zero(20, cp_ptr_->dof_num_);
            rbs_.flt_base_omega_mat = Eigen::MatrixXf::Zero(20, 3);
            rbs_.motion_idx = 0;
        }


        void UpdateRobotObservation() {
            rbs_.base_rpy = ri_ptr_->GetImuRpy();
            rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
            rbs_.base_omega = ri_ptr_->GetImuOmega();
            rbs_.base_acc = ri_ptr_->GetImuAcc();
            rbs_.joint_pos = ri_ptr_->GetJointPosition();
            rbs_.joint_vel = ri_ptr_->GetJointVelocity();
            rbs_.joint_tau = ri_ptr_->GetJointTorque();
            rbs_.hmap_vec  = ri_ptr_->GetHeightMapVec();

            // 储存
            rbs_.flt_base_acc_mat.row(acc_rot_count) = rbs_.base_acc.transpose();
            rbs_.flt_joint_vel_mat.row(acc_rot_count) = rbs_.joint_vel.transpose();
            rbs_.flt_base_omega_mat.row(acc_rot_count) = rbs_.base_omega.transpose();

            acc_rot_count += 1;
            acc_rot_count = acc_rot_count % 20;
        }

        void PolicyRunner() {
            int run_cnt_record = -1;
            int print_cnt = 0;
            auto next_time = high_resolution_clock::now();
            timespec start_timestamp, end_timestamp, last_start_timestamp;
            bool needFlush = false;
            std::shared_ptr<ModelInferenceBase> policy_ptr_tmp;
            UserCommand user_command_tmp;
            while (start_flag_.load(std::memory_order_acquire)) {
                    {
                        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
                        memcpy(&user_command_tmp, &user_command, sizeof(UserCommand));
                        policy_ptr_tmp = policy_ptr_;
                    }

                    if (state_run_cnt_.load(std::memory_order_relaxed) - run_cnt_record >= policy_ptr_tmp->decimation_) {
                        run_cnt_record = state_run_cnt_.load(std::memory_order_relaxed);
                        clock_gettime(CLOCK_MONOTONIC, &start_timestamp);
                        if(((start_timestamp.tv_sec - last_start_timestamp.tv_sec) * 1e3
                                            + (start_timestamp.tv_nsec - last_start_timestamp.tv_nsec) / 1e6) > 22)
                        {
                            std::cout << "common policy period:  " << ((start_timestamp.tv_sec - last_start_timestamp.tv_sec) * 1e3
                                            + (start_timestamp.tv_nsec - last_start_timestamp.tv_nsec) / 1e6) << " ms\n";
                        }
                        last_start_timestamp = start_timestamp;
                        // 拷贝 rbs_ 快照
                        RobotBasicState rbs_local;
                        {
                            std::lock_guard<std::mutex> lock(rbs_mutex_);
                            rbs_local = rbs_;
                        }
                        // 消费步态事件，并传递给策略切换逻辑
                        uint32_t new_gait = 0;
                        bool has_gait_event = uc_ptr_->ConsumeGaitEvent(new_gait);
                        policy_ptr_tmp->update_inner_policy(has_gait_event, new_gait, rbs_local);
                        // 将 update_inner_policy 可能修改的 motion_idx 回写到 rbs_，保证下次快照能拿到最新值
                        {
                            std::lock_guard<std::mutex> lock(rbs_mutex_);
                            rbs_.motion_idx = rbs_local.motion_idx;
                        }
                        auto start_time = high_resolution_clock::now();
                        auto ra = policy_ptr_tmp->GetRobotAction(rbs_local, user_command_tmp, joint_cmd);//actdim(12)*5
                        // std::cout<<"kp:"<<joint_cmd.col(0).transpose()<<std::endl;
                        // std::cout<<"kd:"<<joint_cmd.col(2).transpose()<<std::endl;
                        // joint_cmd.col(0).setZero();
                        // joint_cmd.col(2).setZero(); 
                        // std::cout<<"joint_cmd:"<<joint_cmd.transpose()<<std::endl;

                        clock_gettime(CLOCK_MONOTONIC, &end_timestamp);
                        policy_cost_time_ = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1e3
                                            + (end_timestamp.tv_nsec - start_timestamp.tv_nsec) / 1e6;
                        if(policy_cost_time_ > 2)
                        std::cout << "common policy cost_time:  " << policy_cost_time_ << " ms\n";
                        // 每 ~2s 打印一次当前策略
                        if (++print_cnt % 100 == 0) {
                            std::cout << "[" << state_name_ << "] 当前策略: " << crx_policy_runner_->GetCurrentPolicyName()
                                      << ", gait=0x" << std::hex << StateBase::msfb_.GetCurrentGait() << std::dec << std::endl;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

    public:
        /**
         * @brief 获取策略调度器，供 QiLiControlState 共享策略资源
         */
        std::shared_ptr<CRXModelInferenceExternal> GetCRXPolicyRunner() const { return crx_policy_runner_; }

        RLControlState(const RobotName &robot_name, const std::string &state_name,
                       std::shared_ptr<ControllerData> data_ptr,
                       const std::string &onnx_path = CRXModelInference::kPolicyOnnxPath,
                       RobotMotionState feedback_state = RobotMotionState::RLControlMode,
                       StateName own_state_name = StateName::kRLControl,
                       std::shared_ptr<CommonPolicyRunner> policy_runner = nullptr)
            : StateBase(robot_name, state_name, data_ptr),
              feedback_state_(feedback_state), own_state_name_(own_state_name) {
            std::memset(&rbs_, 0, sizeof(rbs_));
            // if(robot_name == RobotName::CR1STANDARD){
                if (policy_runner) {
                    crx_policy_runner_ = std::make_shared<CRXModelInferenceExternal>(
                        state_name, robot_name, std::move(policy_runner));
                } else {
                    crx_policy_runner_ = std::make_shared<CRXModelInferenceExternal>(
                        state_name, robot_name, onnx_path);
                }
            // }

            policy_ptr_ = crx_policy_runner_;
            if (!policy_ptr_)
            {
                std::cerr << "policy_ptr_: error policy" << std::endl;
                exit(0);
            }
            policy_ptr_->GetRobotParameters(cp_ptr_);
            InitJointIndexRanges();
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            init_rbs_();
        }

        ~RLControlState() {
            if (run_policy_thread_.joinable()) {//确保线程在对象销毁时能够正确退出
                run_policy_thread_.join();
            }
        }

        virtual void OnEnter() {
            
                state_run_cnt_.store(-1, std::memory_order_relaxed);
                {
                    std::lock_guard<std::mutex> lock(rbs_mutex_);
                    UpdateRobotObservation();
                }
                ds_ptr_->InsertScopeData(8, policy_cost_time_);
                start_flag_.store(true, std::memory_order_release);
                {
                    std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
                    policy_ptr_->OnEnter();
                    memcpy(&user_command, uc_ptr_->GetUserCommand(), sizeof(UserCommand));
                }
                StateBase::msfb_.UpdateCurrentState(feedback_state_);
                run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
        };

        virtual void OnExit() {
            start_flag_.store(false, std::memory_order_release);
            policy_ptr_->OnExit();
            run_policy_thread_.join();
            state_run_cnt_.store(-1, std::memory_order_relaxed);
            //TODO外面切换motion时清理gait
            StateBase::msfb_.UpdateCurrentGait(0);
        }

        virtual void Run() {
            {
                std::lock_guard<std::mutex> lock(rbs_mutex_);
                UpdateRobotObservation();
            }
                ds_ptr_->InsertScopeData(8, policy_cost_time_);
                state_run_cnt_.fetch_add(1, std::memory_order_relaxed);
                {
                    std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
                    memcpy(&user_command, uc_ptr_->GetUserCommand(), sizeof(UserCommand));
                    // 上半身故障保护
                    // std::cout<<"joint_cmd: "<<joint_cmd.transpose()<<std::endl;
                    ApplyUpperBodyFaultProtection(joint_cmd);
                    // std::cout<<"joint_cmd_after: "<<joint_cmd.transpose()<<std::endl;
                    ri_ptr_->SetJointCommand(joint_cmd);


                    //heading 模式，本来应该在接收轴指令的地方直接对指令做闭环，但那里是5hz的低频，因此只能移到1khz高频的这里写
                    if(heading_mode_){
                        if(is_first_enter_){
                            yaw_target_ = rbs_.base_rpy[2];
                            is_first_enter_ = false;
                        }
                        if (fabs(user_command.turnning_vel_scale) > 0.1)yaw_target_ = rbs_.base_rpy[2] + user_command.turnning_vel_scale * 0.25;
                        if (yaw_target_ > M_PI) yaw_target_ -= 2 * M_PI;
                        else if (yaw_target_ < -M_PI) yaw_target_ += 2 * M_PI;
                        yaw_delta_ = yaw_target_ - rbs_.base_rpy[2];
                        if (yaw_delta_ > M_PI) yaw_delta_ -= 2 * M_PI;
                        else if (yaw_delta_ < -M_PI)yaw_delta_ += 2 * M_PI;
                        user_command.turnning_vel_scale = 4.0 * yaw_delta_;
                        if(user_command.turnning_vel_scale > 1.0) user_command.turnning_vel_scale = 1.0;
                        else if(user_command.turnning_vel_scale < -1.0) user_command.turnning_vel_scale = -1.0;
                    }else{
                        is_first_enter_ = true;//确保每次手柄切换至heading模式，都初始化为当前imuyaw角
                    }
                }
        }

        virtual bool LoseControlJudge() {
            if (user_command.target_mode == (RobotMotionState::JointDamping)) return true;
            return false;
        }

        virtual StateName GetNextStateName() {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            // 用户主动请求急停，直接切入 JointDamping
            if (user_command.target_mode == RobotMotionState::JointDamping) {
                return StateName::kJointDamping;
            }

            if(user_command.safe_control_mode!=0){
                
                // 分级降级判定（仅 CR1PRO 和 CR1STANDARD 支持）
                if (robot_name_ == RobotName::CR1PRO || robot_name_ == RobotName::CR1STANDARD) {
                    uint32_t current_gait = msfb_.GetCurrentGait();
                    // 获取故障掩码并分组判定
                    uint32_t fault_mask = sc_ptr_->GetFaultMask();
                    if (fault_mask != 0) {
                        // 故障存在 → 取消恢复观测
                        upper_body_recovery_start_ = 0;
                        upper_body_recovered_ = false;

                        BodyFaultGroup group = sc_ptr_->ClassifyFault(fault_mask);
                        
                        // 下半身/整机级故障 → 软急停
                        if (group == BodyFaultGroup::LEG || 
                            group == BodyFaultGroup::WAIST_XY || 
                            group == BodyFaultGroup::IMU_BODY) {
                            logger_.error("[FAULT] Lower/whole body fault (group=%d, mask=0x%x) -> JointDamping",
                                         static_cast<int>(group), fault_mask);
                            return StateName::kJointDamping;
                        }
                        
                        // 上半身故障 → 告警，不切换状态（Run()中清零对应关节指令）
                        if (group == BodyFaultGroup::HEAD ||
                            group == BodyFaultGroup::IMU_HEAD ||
                            group == BodyFaultGroup::UPPER_LIMB ||
                            group == BodyFaultGroup::WAIST_Z) {
                            static auto last_upper_warn = std::chrono::steady_clock::now();
                            static BodyFaultGroup last_upper_group = BodyFaultGroup::NONE;
                            auto now = std::chrono::steady_clock::now();
                            if (last_upper_group != group) {
                                logger_.error("[FAULT] Upper body fault detected (group=%d, mask=0x%x), zeroing joints",
                                             static_cast<int>(group), fault_mask);
                                last_upper_group = group;
                            } else if (std::chrono::duration_cast<std::chrono::seconds>(now - last_upper_warn).count() >= 1) {
                                logger_.warn("[FAULT] Upper body fault ongoing (group=%d, mask=0x%x)",
                                             static_cast<int>(group), fault_mask);
                                last_upper_warn = now;
                            }
                            return own_state_name_;
                        }
                    } else {
                        // 故障已清除 → 恢复观测（3s 稳定后方可退出）
                        if (!upper_body_recovered_) {
                            float now = ri_ptr_->GetInterfaceTimeStamp();
                            if (upper_body_recovery_start_ == 0) {
                                upper_body_recovery_start_ = now;
                                logger_.info("Upper body fault cleared, recovery observation started");
                            } else if (now - upper_body_recovery_start_ >= kRecoveryStableTime) {
                                logger_.info("Recovery stable for %.1fs, resuming normal operation",
                                             now - upper_body_recovery_start_);
                                upper_body_recovered_ = true;
                                upper_body_recovery_start_ = 0;
                            }
                        }
                        return own_state_name_;  // 观测中或已恢复，保持当前状态
                    }
                }
                
                return StateName::kJointDamping;
            }
            if (user_command.target_mode == (RobotMotionState::WaitingForStand))
                return StateName::kIdle;
            if(user_command.target_mode == (RobotMotionState::SuspendedStand)) return StateName::kSuspendedStand;
            if(user_command.target_mode == (RobotMotionState::Ysc)) return StateName::kYsc;
            if(user_command.target_mode == (RobotMotionState::Mofang)) return StateName::kMofang;
            if(user_command.target_mode == (RobotMotionState::ZeroPos)) return StateName::kZeroPos;
            if(user_command.target_mode == (RobotMotionState::RLControlMode)) return StateName::kRLControl;
            if(user_command.target_mode == (RobotMotionState::ReStand)) return StateName::kReStand;
            return own_state_name_;
        }
    };
};//namespace humanoid;

#endif
