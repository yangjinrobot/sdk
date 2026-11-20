#ifndef HUMANOID_PRO_AMP_MOE_15DOF_STATE_HPP_
#define HUMANOID_PRO_AMP_MOE_15DOF_STATE_HPP_

#include "amp_moe_15dof_policy_runner.hpp"
#include "state_base.hpp"
#include <atomic>
#include <thread>

namespace humanoid_pro {
class AmpMoe15DofState : public StateBase {
private:
    std::shared_ptr<AmpMoe15DofPolicyRunner> policy_ptr_;
    RobotBasicState rbs_{};
    UserCommand user_command{};
    std::mutex rbs_mutex_;
    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false}, policy_available_{false};
    std::atomic<int> state_run_cnt_{0};
    bool policy_entered_ = false;

    RobotBasicState ReadObservation() {
        RobotBasicState state{};
        state.base_rpy = ri_ptr_->GetImuRpy();
        state.base_rot_mat = RpyToRm(state.base_rpy);
        state.base_omega = ri_ptr_->GetImuOmega();
        state.base_acc = ri_ptr_->GetImuAcc();
        state.joint_pos = ri_ptr_->GetJointPosition();
        state.joint_vel = ri_ptr_->GetJointVelocity();
        state.joint_tau = ri_ptr_->GetJointTorque();
        return state;
    }
    void InferAndSend(const RobotBasicState& state, const UserCommand& command) {
        auto action = policy_ptr_->GetRobotAction(state, command);
        if (!policy_ptr_->IsReady()) throw std::runtime_error("AMP MoE inference failed");
        const MatXf output = action.ConvertToMat();
        if (output.rows() != 29 || output.cols() != 5 || !output.allFinite())
            throw std::runtime_error("AMP MoE requires finite 29x5 output");
        MatXf full = MatXf::Zero(cp_ptr_->dof_num_, 5);
        full.topRows(29) = output;
        ri_ptr_->SetJointCommand(full);
    }
    void PolicyRunner() {
        try {
            int run_cnt_record = 0;  // OnEnter already sent the t=0 action.
            while (start_flag_) {
                RobotBasicState state;
                UserCommand command;
                int tick;
                {
                    std::lock_guard<std::mutex> lock(rbs_mutex_);
                    tick = state_run_cnt_.load();
                    if (tick - run_cnt_record >= policy_ptr_->decimation_) {
                        state = rbs_; command = user_command;
                    }
                }
                if (tick - run_cnt_record >= policy_ptr_->decimation_) {
                    InferAndSend(state, command);
                    run_cnt_record = tick;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        } catch (const std::exception& error) {
            std::cerr << "AMP MoE worker failure: " << error.what() << std::endl;
            policy_available_ = false; start_flag_ = false;
        } catch (...) { policy_available_ = false; start_flag_ = false; }
    }

public:
    AmpMoe15DofState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr), policy_ptr_(std::make_shared<AmpMoe15DofPolicyRunner>("amp_moe_15dof", robot_name)) {}
    ~AmpMoe15DofState() override { OnExit(); }
    void OnEnter() override {
        try {
            state_run_cnt_ = 0;
            rbs_ = ReadObservation();
            if (rbs_.joint_pos.size() != 31 || !rbs_.joint_pos.allFinite())
                throw std::runtime_error("AMP MoE state requires finite 31-joint feedback");
            Vec7f left = rbs_.joint_pos.segment<7>(3);
            Vec7f right = rbs_.joint_pos.segment<7>(10);
            const MatXf previous = ri_ptr_->GetJointCommand();
            if (previous.rows() == 31 && previous.cols() == 5 && previous.allFinite()) {
                left = previous.block<7, 1>(3, 1); right = previous.block<7, 1>(10, 1);
            }
            policy_ptr_->SetInitialArmTargets(left, right);
            policy_ptr_->OnEnter();
            policy_entered_ = true;
            policy_available_ = policy_ptr_->IsReady();
            if (!policy_available_) throw std::runtime_error("AMP MoE policy is not ready");
            user_command = uc_ptr_->GetUserCommandSnapshot();
            // Match simtosim: initialize history from the entry observation and
            // apply its action before advancing physics, then hold it for 20 ms.
            InferAndSend(rbs_, user_command);
            motion_state_store_->SetState(RobotMotionState::AmpMoe15Dof);
            start_flag_ = true;
            run_policy_thread_ = std::thread(&AmpMoe15DofState::PolicyRunner, this);
        } catch (const std::exception& error) {
            std::cerr << "AMP MoE entry failure: " << error.what() << std::endl;
            OnExit(); policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        } catch (...) {
            OnExit(); policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        }
    }
    void OnExit() override {
        start_flag_ = false;
        if (run_policy_thread_.joinable()) run_policy_thread_.join();
        if (policy_entered_) { policy_ptr_->OnExit(); policy_entered_ = false; }
    }
    void Run() override {
        {
            std::lock_guard<std::mutex> lock(rbs_mutex_);
            rbs_ = ReadObservation(); user_command = uc_ptr_->GetUserCommandSnapshot();
            ++state_run_cnt_;
        }
    }
    bool LoseControlJudge() override {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        return !policy_available_ || user_command.safe_control_mode != 0 || user_command.target_mode == RobotMotionState::JointDamping;
    }
    StateName GetNextStateName() override {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        if (!policy_available_ || user_command.safe_control_mode != 0 || user_command.target_mode == RobotMotionState::JointDamping)
            return StateName::kJointDamping;
        if (user_command.target_mode == RobotMotionState::RLStandControlMode) return StateName::kRLStandControl;
        if (user_command.target_mode == RobotMotionState::RLStandSaluteMode) return StateName::kRLStandSalute;
        if (user_command.target_mode == RobotMotionState::RLStandSaluteLeftArmMode) return StateName::kRLStandSaluteLeftArm;
        if (user_command.target_mode == RobotMotionState::RLStandPleaseMode) return StateName::kRLStandPlease;
        if (user_command.target_mode == RobotMotionState::RLStandTalkingMode) return StateName::kRLStandTalking;
        if (user_command.target_mode == RobotMotionState::RLStandPlease2Mode) return StateName::kRLStandPlease2;
        if (user_command.target_mode == RobotMotionState::AmpWalkingControl) return StateName::kAmpWalkingControl;
        if (user_command.target_mode == RobotMotionState::AmpWalkingWAQ) return StateName::kAmpWalkingWAQ;
        return StateName::kAmpMoe15Dof;
    }
};
}  // namespace humanoid_pro

#endif
