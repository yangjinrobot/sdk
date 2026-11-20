#ifndef HUMANOID_PRO_COPIED_POLICY_CONTROL_STATE_HPP_
#define HUMANOID_PRO_COPIED_POLICY_CONTROL_STATE_HPP_

#include "state_base.hpp"
#include "policy_runner_base.hpp"
#include "humanoid_pro/bounded_policy_command.hpp"
#include <atomic>
#include <optional>
#include <thread>

namespace humanoid_pro {
// Shared lifecycle for the imported AMP and sit/stand policies. The worker only
// reads snapshots; OnExit joins it before touching the policy's runtime buffers.
class CopiedPolicyControlState : public StateBase {
private:
    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    BoundedPolicyCommand bounded_command_;
    RobotMotionState mode_;
    std::vector<RobotMotionState> exits_;
    RobotBasicState rbs_{};
    UserCommand user_command{};
    std::mutex rbs_mutex_;
    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false}, policy_available_{false};
    int state_run_cnt_ = 0;
    bool entered_ = false;
    std::optional<RobotMotionState> smooth_entry_source_;
    std::optional<RobotMotionState> completion_source_;
    unsigned completion_steps_;
    unsigned successful_steps_ = 0;
    bool completion_gate_active_ = false;
    MatXf entry_command_;
    MatXf policy_command_;
    double entry_start_time_ = 0.0;
    std::atomic<bool> entry_blend_active_{false};
    static constexpr double kEntryBlendSeconds = 1.0;

    RobotBasicState ReadObservation() const {
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
    void SendDamping() {
        MatXf cmd = MatXf::Zero(cp_ptr_->dof_num_, 5);
        cmd.col(2).setOnes();
        ri_ptr_->SetJointCommand(cmd);
    }
    MatXf BlendedCommandLocked() {
        if (!entry_blend_active_.load(std::memory_order_acquire))
            return policy_command_;
        const double elapsed = ri_ptr_->GetInterfaceTimeStamp() - entry_start_time_;
        const float s = static_cast<float>(
            std::clamp(elapsed / kEntryBlendSeconds, 0.0, 1.0));
        const float weight = s * s * (3.0f - 2.0f * s);
        MatXf output = (1.0f - weight) * entry_command_ + weight * policy_command_;
        if (s >= 1.0f)
            entry_blend_active_.store(false, std::memory_order_release);
        return output;
    }
    void PublishCommandLocked() {
        ri_ptr_->SetJointCommand(BlendedCommandLocked());
    }
    void InferAndSend(const RobotBasicState& state, const UserCommand& command) {
        auto action = policy_ptr_->GetRobotAction(state, command);
        if (!policy_ptr_->IsReady()) throw std::runtime_error("Imported policy inference failed");
        const MatXf output = action.ConvertToMat();
        if (output.rows() != 29 || output.cols() != 5 || !output.allFinite())
            throw std::runtime_error("Imported policy requires finite 29x5 output");
        MatXf full = MatXf::Zero(31, 5);
        full.topRows(29) = output;
        const MatXf bounded = bounded_command_.Apply(full, state);
        {
            std::lock_guard<std::mutex> lock(rbs_mutex_);
            policy_command_ = bounded;
            PublishCommandLocked();
            if (successful_steps_ < completion_steps_) ++successful_steps_;
        }
    }
    void Fail() {
        policy_available_ = false;
        start_flag_ = false;
        entry_blend_active_.store(false, std::memory_order_release);
        SendDamping();
    }
    void PolicyRunner() {
        try {
            int run_cnt_record = 0;
            while (start_flag_) {
                RobotBasicState state;
                UserCommand command;
                int tick;
                {
                    std::lock_guard<std::mutex> lock(rbs_mutex_);
                    tick = state_run_cnt_;
                    if (tick - run_cnt_record >= policy_ptr_->decimation_) {
                        state = rbs_;
                        command = user_command;
                    }
                }
                if (tick - run_cnt_record >= policy_ptr_->decimation_) {
                    InferAndSend(state, command);
                    run_cnt_record = tick;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        } catch (const std::exception& error) {
            std::cerr << state_name_ << " worker failure: " << error.what() << std::endl;
            Fail();
        } catch (...) { Fail(); }
    }

protected:
    CopiedPolicyControlState(RobotName robot_name, const std::string& state_name,
                             std::shared_ptr<ControllerData> data_ptr,
                             std::shared_ptr<PolicyRunnerBase> policy,
                             RobotMotionState mode, std::vector<RobotMotionState> exits,
                             std::optional<RobotMotionState> smooth_entry_source = std::nullopt,
                             std::optional<RobotMotionState> completion_source = std::nullopt,
                             unsigned completion_steps = 0)
        : StateBase(robot_name, state_name, data_ptr), policy_ptr_(std::move(policy)),
          bounded_command_(*cp_ptr_), mode_(mode), exits_(std::move(exits)),
          smooth_entry_source_(smooth_entry_source), completion_source_(completion_source),
          completion_steps_(completion_steps) {
        if (robot_name != RobotName::CR1PRO || cp_ptr_->dof_num_ != 31)
            throw std::invalid_argument("Imported policies require CR1PRO / 31 DOF");
    }

public:
    ~CopiedPolicyControlState() override { OnExit(); }
    void OnEnter() override {
        OnExit();
        policy_available_ = false;
        try {
            state_run_cnt_ = 0;
            successful_steps_ = 0;
            rbs_ = ReadObservation();
            user_command = uc_ptr_->GetUserCommandSnapshot();
            const RobotMotionState source = motion_state_store_->GetState();
            completion_gate_active_ = completion_source_ && source == *completion_source_;
            entry_blend_active_.store(false, std::memory_order_release);
            if (smooth_entry_source_ == source) {
                const MatXf previous = ri_ptr_->GetJointCommand();
                if (previous.rows() == 31 && previous.cols() == 5 && previous.allFinite()) {
                    entry_command_ = previous;
                    entry_start_time_ = ri_ptr_->GetInterfaceTimeStamp();
                    entry_blend_active_.store(true, std::memory_order_release);
                } else {
                    throw std::runtime_error(
                        state_name_ + " requires a finite previous 31x5 command for entry blending");
                }
            }
            policy_ptr_->OnEnter();
            entered_ = true;
            if (!policy_ptr_->IsReady() || user_command.safe_control_mode ||
                user_command.target_mode == RobotMotionState::JointDamping)
                throw std::runtime_error("Imported policy cannot enter");
            // Initialize reference t=0 and immediately send the first action.
            InferAndSend(rbs_, user_command);
            motion_state_store_->SetState(mode_);
            policy_available_ = true;
            start_flag_ = true;
            run_policy_thread_ = std::thread(&CopiedPolicyControlState::PolicyRunner, this);
        } catch (const std::exception& error) {
            std::cerr << state_name_ << " entry failure: " << error.what() << std::endl;
            OnExit();
            Fail();
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        } catch (...) {
            OnExit();
            Fail();
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        }
    }
    void OnExit() override {
        start_flag_ = false;
        if (run_policy_thread_.joinable()) run_policy_thread_.join();
        if (entered_) { policy_ptr_->OnExit(); entered_ = false; }
        policy_available_ = false;
        entry_blend_active_.store(false, std::memory_order_release);
    }
    void Run() override {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        rbs_ = ReadObservation();
        user_command = uc_ptr_->GetUserCommandSnapshot();
        const bool blending = entry_blend_active_.load(std::memory_order_acquire);
        if (!blending) ++state_run_cnt_;
        if (blending && policy_available_ && !user_command.safe_control_mode &&
            user_command.target_mode != RobotMotionState::JointDamping) {
            try {
                PublishCommandLocked();
            } catch (const std::exception& error) {
                std::cerr << state_name_ << " entry blend failure: " << error.what() << std::endl;
                Fail();
            }
        }
        if (completion_steps_ && successful_steps_ >= completion_steps_ && policy_available_ &&
            !user_command.safe_control_mode && user_command.target_mode != RobotMotionState::JointDamping)
            motion_state_store_->MarkHandsetReady();
    }
    bool LoseControlJudge() override {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        return !policy_available_ || user_command.safe_control_mode || user_command.target_mode == RobotMotionState::JointDamping;
    }
    StateName GetNextStateName() override {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        if (!policy_available_ || user_command.safe_control_mode || user_command.target_mode == RobotMotionState::JointDamping)
            return StateName::kJointDamping;
        if (completion_gate_active_ && successful_steps_ < completion_steps_)
            return static_cast<StateName>(mode_);
        for (const auto exit : exits_)
            if (user_command.target_mode == exit) return static_cast<StateName>(exit);
        return static_cast<StateName>(mode_);
    }
};
}  // namespace humanoid_pro

#endif
