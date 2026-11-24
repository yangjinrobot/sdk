#ifndef HUMANOID_STD_MJRUN_STATE_HPP_
#define HUMANOID_STD_MJRUN_STATE_HPP_

#include "state_base.h"
#include "mjrun_policy_runner.h"
#include "arm_entry_transition.hpp"

#include <chrono>

namespace humanoid_std {

class MjRunState final : public StateBase {
public:
    static constexpr const char* kDefaultPolicyPath = MjRunPolicyRunner::kDefaultPolicyPath;

    MjRunState(const RobotName& robot_name, const std::string& state_name,
               std::shared_ptr<ControllerData> data_ptr,
               const std::string& policy_path = kDefaultPolicyPath)
        : StateBase(robot_name, state_name, data_ptr), policy_(robot_name, policy_path) {
        policy_.DisplayPolicyInfo();
    }

    void OnEnter() override {
        policy_.OnEnter();
        policy_tick_ = 0;
        stopped_frames_ = 0;
        waiting_for_neutral_ = true;
        failed_ = false;
        try {
            last_time_ = ri_ptr_->GetInterfaceTimeStamp();
            joint_cmd_ = ri_ptr_->GetJointCommand();
            if (joint_cmd_.rows() != MjRunPolicyRunner::kJoints || !std::isfinite(last_time_))
                throw std::runtime_error("Invalid entry joint command or timestamp");
            // Carry over the last joint command and PD gains from the source state.
            entry_transition_.Reset(joint_cmd_, last_time_);
        } catch (const std::exception& error) {
            Fail(error);
        }
    }

    void OnExit() override {
        entry_transition_.Cancel();
        policy_.OnExit();
    }

    void Run() override {
        if (LoseControlJudge()) return;
        try {
            const double now = ri_ptr_->GetInterfaceTimeStamp();
            if (!std::isfinite(now) || now < last_time_) throw std::runtime_error("Invalid interface timestamp");
            last_time_ = now;
            if (policy_tick_ == 0) UpdatePolicyCommand();
            MatXf output = joint_cmd_;
            entry_transition_.Apply(output, 0, now);
            if (!output.allFinite()) throw std::runtime_error("Invalid joint command");
            if (LoseControlJudge()) return;
            ri_ptr_->SetJointCommand(output);
            policy_tick_ = (policy_tick_ + 1) % policy_.decimation_;

            const uint32_t source = msfb_.GetCurrentState();
            if (source != RobotMotionState::MjRun) {
                msfb_.UpdateCurrentGait(0);
                msfb_.UpdateCurrentState(RobotMotionState::MjRun);
                std::cout << "[mjrun] entered from 0x" << std::hex << source << std::dec << std::endl;
            }
        } catch (const std::exception& error) {
            Fail(error);
        }
    }

    bool LoseControlJudge() override {
        const auto& command = *uc_ptr_->GetUserCommand();
        return failed_ || command.safe_control_mode != 0 || command.target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        switch (uc_ptr_->GetUserCommand()->target_mode) {
            case RobotMotionState::Restand: return StateName::kRestand;
            case RobotMotionState::Run: return StateName::kRun;
            case RobotMotionState::DunXia: return StateName::kDunXia;
            case RobotMotionState::Stand2SitMimic: return StateName::kStand2SitMimic;
            case RobotMotionState::RLControlMode:
                return stopped_frames_ >= kStoppedFrames && policy_tick_ == 0
                    ? StateName::kRLControl : StateName::kMjRun;
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            default: return StateName::kMjRun;
        }
    }

private:
    void UpdatePolicyCommand() {
        RobotBasicState state{};
        state.base_rpy = ri_ptr_->GetImuRpy();
        state.base_rot_mat = RpyToRm(state.base_rpy);
        state.base_omega = ri_ptr_->GetImuOmega();
        state.joint_pos = ri_ptr_->GetJointPosition();
        state.joint_vel = ri_ptr_->GetJointVelocity();

        UserCommand command = *uc_ptr_->GetUserCommand();
        const Vec3f axes(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
        if (!axes.allFinite()) throw std::runtime_error("Invalid joystick input");
        const bool returning = command.target_mode == RobotMotionState::RLControlMode;
        if (entry_transition_.active() || waiting_for_neutral_ || returning) {
            // Accept joystick input only after a centered sample following the full-body blend.
            if (!entry_transition_.active() && axes.cwiseAbs().maxCoeff() <= kJoystickNeutralThreshold)
                waiting_for_neutral_ = false;
            command.forward_vel_scale = command.side_vel_scale = command.turnning_vel_scale = 0.0f;
        }
#ifndef MOTION_MASTER_MUJOCO_LOCKSTEP
        const auto start = std::chrono::steady_clock::now();
#endif
        auto action = policy_.GetRobotAction(state, command);
#ifndef MOTION_MASTER_MUJOCO_LOCKSTEP
        if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(20))
            throw std::runtime_error("Inference exceeded 20 ms");
#endif
        joint_cmd_ = action.ConvertToMat();
        stopped_frames_ = returning && policy_.CommandVelocity().isZero()
            ? stopped_frames_ + 1 : 0;
    }

    void Fail(const std::exception& error) {
        failed_ = true;
        std::cerr << "[mjrun] " << error.what() << "; entering damping\n";
    }

    static constexpr int kStoppedFrames = 13;
    static constexpr float kJoystickNeutralThreshold = 0.05f;
    JointEntryTransition entry_transition_;
    MjRunPolicyRunner policy_;
    MatXf joint_cmd_;
    double last_time_ = 0.0;
    int policy_tick_ = 0;
    int stopped_frames_ = 0;
    bool waiting_for_neutral_ = true;
    bool failed_ = false;
};

}  // namespace humanoid_std

#endif
