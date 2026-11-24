#ifndef HUMANOID_PRO_SQUAT_PREPARATION_STATE_HPP_
#define HUMANOID_PRO_SQUAT_PREPARATION_STATE_HPP_

#include "state_base.hpp"
#include "humanoid_pro/bounded_policy_command.hpp"
#include <optional>

namespace humanoid_pro {
class SquatPreparationState : public StateBase {
private:
    static constexpr int kBodyJoints = 29;
    static constexpr double kPreparationSeconds = 3.0;
    static constexpr double kSettleSeconds = 0.15;
    static constexpr float kPositionTolerance = 0.15f;
    static constexpr float kVelocityTolerance = 0.3f;
    BoundedPolicyCommand bounded_command_;
    VecXf start_, goal_, kp_, kd_;
    double start_time_ = 0;
    std::optional<double> settled_since_;
    bool ready_ = false, failed_ = false;

public:
    SquatPreparationState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr), bounded_command_(*cp_ptr_),
          goal_(VecXf::Zero(31)), kp_(VecXf::Zero(31)), kd_(VecXf::Zero(31)) {
        // SDK order: waist, left arm, right arm, left leg, right leg, neck.
        goal_.head(kBodyJoints) <<
            0.f, 0.f, 0.f,
            -.296706f, .122173f, -.122173f, .558505f, -.226893f, -.052360f, .191986f,
            -.383972f, -.261799f, .069813f, .575959f, .139626f, -.017453f, -.226893f,
            -1.431170f, 0.f, 0.f, 1.884956f, -.349066f, 0.f,
            -1.431170f, 0.f, 0.f, 1.884956f, -.349066f, 0.f;
        kp_.head(kBodyJoints) << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_,
            cp_ptr_->leg_kp_, cp_ptr_->leg_kp_;
        kd_.head(kBodyJoints) << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_,
            cp_ptr_->leg_kd_, cp_ptr_->leg_kd_;
    }

    void OnEnter() override {
        start_ = ri_ptr_->GetJointPosition();
        start_time_ = ri_ptr_->GetInterfaceTimeStamp();
        failed_ = start_.size() != 31 || !start_.allFinite();
        settled_since_.reset();
        ready_ = false;
        motion_state_store_->SetState(SquatPreparation);
        motion_state_store_->SetHandsetReady(false);
    }
    void OnExit() override {}

    void Run() override {
        if (LoseControlJudge()) return;
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        const float t = static_cast<float>(std::clamp((now - start_time_) / kPreparationSeconds, 0.0, 1.0));
        const float weight = t * t * (3.f - 2.f * t);
        const float rate = 6.f * t * (1.f - t) / kPreparationSeconds;
        MatXf command = MatXf::Zero(31, 5);
        command.col(0) = weight * kp_;
        command.col(1) = start_ + weight * (goal_ - start_);
        command.col(2) = weight * kd_;
        command.col(3) = rate * (goal_ - start_);
        RobotBasicState observation{};
        observation.joint_pos = ri_ptr_->GetJointPosition();
        observation.joint_vel = ri_ptr_->GetJointVelocity();
        try {
            ri_ptr_->SetJointCommand(bounded_command_.Apply(command, observation));
        } catch (const std::exception& error) {
            std::cerr << state_name_ << " preparation failed: " << error.what() << std::endl;
            failed_ = true;
            motion_state_store_->SetHandsetReady(false);
            return;
        }

        const bool at_goal = t >= 1.f &&
            (observation.joint_pos - goal_).head(kBodyJoints).cwiseAbs().maxCoeff() <= kPositionTolerance &&
            observation.joint_vel.head(kBodyJoints).cwiseAbs().maxCoeff() <= kVelocityTolerance;
        if (!at_goal) settled_since_.reset();
        else if (!settled_since_) settled_since_ = now;
        const bool ready = settled_since_ && now - *settled_since_ >= kSettleSeconds;
        if (ready_ != ready) {
            ready_ = ready;
            motion_state_store_->SetHandsetReady(ready_);
        }
    }

    bool LoseControlJudge() override {
        const auto command = uc_ptr_->GetUserCommandSnapshot();
        return failed_ || command.safe_control_mode ||
               command.target_mode == RobotMotionState::JointDamping;
    }
    StateName GetNextStateName() override {
        if (LoseControlJudge()) return kJointDamping;
        return ready_ && uc_ptr_->GetUserCommandSnapshot().target_mode == RLControlMimic13Mode
            ? kRLMimic13Control : kSquatPreparation;
    }
};
}  // namespace humanoid_pro

#endif
