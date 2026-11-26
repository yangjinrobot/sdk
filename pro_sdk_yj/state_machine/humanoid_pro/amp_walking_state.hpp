#ifndef HUMANOID_PRO_AMP_WALKING_STATE_HPP_
#define HUMANOID_PRO_AMP_WALKING_STATE_HPP_

#include <chrono>
#include <atomic>
#include <stdexcept>

#include "amp_walking_policy_runner.hpp"
#include "state_base.hpp"

using namespace std::chrono;
namespace humanoid_pro {
class AmpWalkingState : public StateBase {
private:
    RobotBasicState rbs_{};
    std::mutex rbs_mutex_;
    std::atomic<int> state_run_cnt_{-1};
    std::atomic<bool> policy_available_{false};
    bool policy_entered_ = false;

    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    std::shared_ptr<AmpWalkingPolicyRunner> common_policy_;

    UserCommand user_command{};
    std::mutex user_command_policy_mutex_;

    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};

    int acc_rot_count = 0;

    void InitRbs() {
        rbs_.base_rpy.setZero();
        rbs_.base_quat.setZero();
        rbs_.base_rot_mat.setIdentity();
        rbs_.base_omega.setZero();
        rbs_.base_acc.setZero();
        rbs_.joint_pos = VecXf::Zero(cp_ptr_->dof_num_);
        rbs_.joint_vel = VecXf::Zero(cp_ptr_->dof_num_);
        rbs_.joint_tau = VecXf::Zero(cp_ptr_->dof_num_);
        rbs_.flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        rbs_.flt_joint_vel_mat = Eigen::MatrixXf::Zero(20, cp_ptr_->dof_num_);
        rbs_.flt_base_omega_mat = Eigen::MatrixXf::Zero(20, 3);
    }

    void UpdateRobotObservation() {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        rbs_.base_rpy = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.base_omega = ri_ptr_->GetImuOmega();
        rbs_.base_acc = ri_ptr_->GetImuAcc();
        rbs_.joint_pos = ri_ptr_->GetJointPosition();
        rbs_.joint_vel = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau = ri_ptr_->GetJointTorque();

        rbs_.flt_base_acc_mat.row(acc_rot_count) = rbs_.base_acc.transpose();
        rbs_.flt_joint_vel_mat.row(acc_rot_count) = rbs_.joint_vel.transpose();
        rbs_.flt_base_omega_mat.row(acc_rot_count) = rbs_.base_omega.transpose();

        acc_rot_count += 1;
        acc_rot_count = acc_rot_count % 20;
    }

    void PolicyRunner() {
        try {
        int run_cnt_record = -1;
        MatXf res;
        MatXf joint_cmd(cp_ptr_->dof_num_, 5);
        MatXf leg_joint_cmd(2 * cp_ptr_->leg_dof_num_, 5);
        MatXf arm_joint_cmd(2 * cp_ptr_->arm_dof_num_, 5);
        MatXf waist_joint_cmd(cp_ptr_->waist_dof_num_, 5);
        MatXf neck_joint_cmd(cp_ptr_->neck_dof_num_, 5);

        UserCommand user_command_tmp;
        RobotBasicState rbs_snapshot;
        while (start_flag_) {
            {
                std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
                memcpy(&user_command_tmp, &user_command, sizeof(UserCommand));
            }

            if (state_run_cnt_ - run_cnt_record >= policy_ptr_->decimation_) {
                {
                    std::lock_guard<std::mutex> lock(rbs_mutex_);
                    rbs_snapshot = rbs_;
                }

                auto ra = policy_ptr_->GetRobotAction(rbs_snapshot, user_command_tmp);  // actdim(12)*5
                if (!policy_ptr_->IsReady())
                    throw std::runtime_error("policy inference is no longer ready");
                MatXf res = ra.ConvertToMat();
                if (res.rows() != 29 || res.cols() != 5)
                    throw std::runtime_error("PRO policy must produce a 29x5 command");

                waist_joint_cmd.setZero();
                arm_joint_cmd.setZero();
                neck_joint_cmd.setZero();
                leg_joint_cmd.setZero();

                // arm_joint_cmd.col(0) = cp_ptr_->arm_kp_.replicate(2, 1);
                // arm_joint_cmd.col(2) = cp_ptr_->arm_kd_.replicate(2, 1);
                // waist_joint_cmd.col(0) = cp_ptr_->waist_kp_;
                // waist_joint_cmd.col(2) = cp_ptr_->waist_kd_;

                // // Only replace joints covered by the policy output; keep PD lock for the rest
                // waist_joint_cmd.block<1, 5>(0, 0) = res.block<1, 5>(0, 0);
                // arm_joint_cmd.block<4, 5>(0, 0) = res.block<4, 5>(1, 0);
                // arm_joint_cmd.block<4, 5>(7, 0) = res.block<4, 5>(5, 0);
                // leg_joint_cmd = res.block<12, 5>(9, 0);

                // 29DOF 策略映射（按 robot_order 前29个关节的顺序）
                // 腰部 3 个关节
                waist_joint_cmd = res.block<3, 5>(0, 0);
                // 左臂 7 个关节
                arm_joint_cmd.block<7, 5>(0, 0) = res.block<7, 5>(3, 0);
                // 右臂 7 个关节
                arm_joint_cmd.block<7, 5>(7, 0) = res.block<7, 5>(10, 0);
                // 左腿 6 个关节
                leg_joint_cmd.block<6, 5>(0, 0) = res.block<6, 5>(17, 0);
                // 右腿 6 个关节
                leg_joint_cmd.block<6, 5>(6, 0) = res.block<6, 5>(23, 0);


                joint_cmd << waist_joint_cmd, arm_joint_cmd, leg_joint_cmd, neck_joint_cmd;

                ri_ptr_->SetJointCommand(joint_cmd);
                run_cnt_record = state_run_cnt_;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        } catch (const std::exception& error) {
            start_flag_ = false;
            policy_available_ = false;
            std::cerr << "[amp_walking_state.hpp] worker failure: " << error.what() << std::endl;
        } catch (...) {
            start_flag_ = false;
            policy_available_ = false;
            std::cerr << "[amp_walking_state.hpp] worker failure" << std::endl;
        }
    }

public:
    AmpWalkingState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {
        common_policy_ = std::make_shared<AmpWalkingPolicyRunner>("amp_walking", robot_name);
        common_policy_->SetCmdMaxVel(Vec3f(1.2f, 0.6f, 1.0f));
        common_policy_->DisplayPolicyInfo();

        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
        policy_ptr_ = common_policy_;
        if (!policy_ptr_) {
            std::cerr << "error policy" << std::endl;
            exit(0);
        }
        InitRbs();
    }

    ~AmpWalkingState() override {
        start_flag_ = false;
        if (run_policy_thread_.joinable()) run_policy_thread_.join();
    }

    void OnEnter() override {
        try {
        state_run_cnt_ = -1;
        policy_available_ = false;
        policy_entered_ = false;
        if (!policy_ptr_) throw std::runtime_error("Missing policy");
        UpdateRobotObservation();
        if (motion_state_store_->GetState() == RobotMotionState::RLControl_StandSalute_Mode ||
            motion_state_store_->GetState() == RobotMotionState::RLControl_StandSaluteLeftArm_Mode ||
            motion_state_store_->GetState() == RobotMotionState::RLControl_StandPlease_Mode ||
            motion_state_store_->GetState() == RobotMotionState::RLControl_StandTalking_Mode ||
            motion_state_store_->GetState() == RobotMotionState::RLControl_StandPlease2_Mode)
            common_policy_->SetArmReleaseFrom(rbs_.joint_pos.segment<7>(3), rbs_.joint_pos.segment<7>(10));
        policy_ptr_->OnEnter();
        policy_entered_ = true;
        policy_available_ = policy_ptr_->IsReady();
        if (!policy_available_) throw std::runtime_error("Policy is not ready");
        {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            user_command = uc_ptr_->GetUserCommandSnapshot();
        }
        start_flag_ = true;
        motion_state_store_->SetState(RobotMotionState::AmpControl_Walk_Mode);
        run_policy_thread_ = std::thread(std::bind(&AmpWalkingState::PolicyRunner, this));

        } catch (const std::exception& error) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
            std::cerr << "Policy lifecycle failure: " << error.what() << std::endl;
        } catch (...) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        }
    }

    void OnExit() override {
        try {
        start_flag_ = false;
        if (run_policy_thread_.joinable()) {
            run_policy_thread_.join();
        }
        if (policy_entered_) {
            policy_ptr_->OnExit();
            policy_entered_ = false;
        }
        state_run_cnt_ = -1;

        } catch (const std::exception& error) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
            std::cerr << "Policy lifecycle failure: " << error.what() << std::endl;
        } catch (...) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        }
    }

    void Run() override {
        UpdateRobotObservation();

        state_run_cnt_++;
        {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            user_command = uc_ptr_->GetUserCommandSnapshot();
        }
    }

    bool LoseControlJudge() override {
        if (!policy_available_) return true;
        if (user_command.target_mode == (RobotMotionState::JointDamping)) return true;
        return false;
    }

    StateName GetNextStateName() override {
        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
        if (!policy_available_) {
            return StateName::kJointDamping;
        }
        if (user_command.safe_control_mode != 0 || user_command.target_mode == (RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        }
        if (user_command.target_mode == RobotMotionState::AmpWalkingZeroPos)
            return StateName::kAmpWalkingZeroPos;
        if (user_command.target_mode == (RobotMotionState::RLControl_Stand_Mode)) {
            return StateName::kRLStandControl;
        }
        if (user_command.target_mode == RobotMotionState::RLControl_StandSalute_Mode)
            return StateName::kRLStandSalute;
        if (user_command.target_mode == RobotMotionState::RLControl_StandSaluteLeftArm_Mode)
            return StateName::kRLStandSaluteLeftArm;
        if (user_command.target_mode == RobotMotionState::RLControl_StandPlease_Mode)
            return StateName::kRLStandPlease;
        if (user_command.target_mode == RobotMotionState::RLControl_StandTalking_Mode)
            return StateName::kRLStandTalking;
        if (user_command.target_mode == RobotMotionState::RLControl_StandPlease2_Mode)
            return StateName::kRLStandPlease2;
        if (user_command.target_mode == RobotMotionState::AmpControl_Walk2Run_Mode)
            return StateName::kRLControl;
        // Dance policies are only reachable from AMP walking.
        if (user_command.target_mode == RobotMotionState::MimicControl_ChairDown_Mode)
            return StateName::kRLMimic12Control;
        if (user_command.target_mode == RobotMotionState::MimicControl_BoxFight_Mode)
            return StateName::kRLMimic2Control;
        return StateName::kAmpWalkingControl;
    }
};
}  // namespace humanoid_pro

#endif
