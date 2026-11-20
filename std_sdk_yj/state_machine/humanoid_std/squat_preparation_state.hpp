#pragma once

#include "state_base.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace humanoid_std {

class SquatPreparationState final : public StateBase {
public:
    SquatPreparationState(RobotName robot, const std::string& name, std::shared_ptr<ControllerData> data)
        : StateBase(robot, name, data) {
        if (robot != RobotName::CR1STANDARD || cp_ptr_->dof_num_ != kJoints ||
            cp_ptr_->joint_vel_limit_.size() != kJoints || cp_ptr_->torque_limit_.size() != kJoints)
            throw std::invalid_argument("Squat preparation requires CR1STANDARD / 21 DOF");

        // CR1PRO SDK indices: waist 0, left arm 3-6, right arm 10-13, legs 17-28.
        // Keep the same angles for these joints in CR1 Standard SDK order.
        goal_ << 0.f,
            -.296706f, .122173f, -.122173f, .558505f,
            -.383972f, -.261799f, .069813f, .575959f,
            -1.431170f, 0.f, 0.f, 1.884956f, -.349066f, 0.f,
            -1.431170f, 0.f, 0.f, 1.884956f, -.349066f, 0.f;
        kp_ << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_,
            cp_ptr_->leg_kp_, cp_ptr_->leg_kp_;
        kd_ << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_,
            cp_ptr_->leg_kd_, cp_ptr_->leg_kd_;
        lower_ << cp_ptr_->waist_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->arm_joint_lower_,
            cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_;
        upper_ << cp_ptr_->waist_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->arm_joint_upper_,
            cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_;
        // Same mirrored roll/yaw bounds as the Standard hardware checker.
        for (const auto& [right, left] : {std::pair{6, 2}, std::pair{7, 3},
                                         std::pair{16, 10}, std::pair{17, 11}}) {
            lower_(right) = -upper_(left);
            upper_(right) = -lower_(left);
        }
        if (!lower_.allFinite() || !upper_.allFinite() || !kp_.allFinite() || !kd_.allFinite() ||
            !cp_ptr_->joint_vel_limit_.allFinite() || !cp_ptr_->torque_limit_.allFinite() ||
            (kp_.array() < 0).any() || (kd_.array() < 0).any() ||
            (goal_.array() < lower_.array()).any() || (goal_.array() > upper_.array()).any() ||
            (cp_ptr_->joint_vel_limit_.array() <= 0).any() || (cp_ptr_->torque_limit_.array() <= 0).any())
            throw std::invalid_argument("Invalid Standard squat posture or joint limits");
    }

    void OnEnter() override {
        const VecXf measured = ri_ptr_->GetJointPosition();
        start_time_ = ri_ptr_->GetInterfaceTimeStamp();
        failed_ = measured.size() != kJoints || !measured.allFinite() || !std::isfinite(start_time_);
        if (!failed_) start_ = measured;
        ready_ = false;
        settled_since_.reset();
        msfb_.UpdateCurrentState(RobotMotionState::SquatPreparation);
        msfb_.SetHandsetReady(false);
        msfb_.UpdateCurrentGait(0);
    }

    void OnExit() override { msfb_.SetHandsetReady(false); }

    void Run() override {
        if (LoseControlJudge()) return;
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        const VecXf measured = ri_ptr_->GetJointPosition();
        const VecXf measured_velocity = ri_ptr_->GetJointVelocity();
        if (!std::isfinite(now) || now < start_time_ ||
            measured.size() != kJoints || measured_velocity.size() != kJoints ||
            !measured.allFinite() || !measured_velocity.allFinite()) {
            failed_ = true;
            msfb_.SetHandsetReady(false);
            return;
        }

        const float t = static_cast<float>(std::clamp((now - start_time_) / kPreparationSeconds, 0.0, 1.0));
        const float weight = t * t * (3.f - 2.f * t);
        const float rate = 6.f * t * (1.f - t) / kPreparationSeconds;
        Eigen::Matrix<float, kJoints, 5> command;
        for (int i = 0; i < kJoints; ++i) {
            const double q = measured(i), dq = measured_velocity(i);
            const double kp = weight * kp_(i), kd = weight * kd_(i);
            const double limit = cp_ptr_->torque_limit_(i);
            const double velocity_limit = cp_ptr_->joint_vel_limit_(i);
            const double desired = start_(i) + weight * (goal_(i) - start_(i));
            const double desired_velocity = rate * (goal_(i) - start_(i));
            const double target = std::clamp(desired, double(lower_(i)), double(upper_(i)));
            const double speed = std::clamp(desired_velocity, -velocity_limit, velocity_limit);
            const double effort = std::clamp(kp * (desired - q) + kd * (desired_velocity - dq), -limit, limit);
            const double pd = kp * (target - q) + kd * (speed - dq);
            double scale = 1.0;
            if (std::abs(effort - pd) > limit && std::abs(pd) > 1e-12)
                scale = std::clamp((effort + std::copysign(limit, pd)) / pd, 0.0, 1.0);
            command(i, 0) = float(kp * scale);
            command(i, 1) = float(target);
            command(i, 2) = float(kd * scale);
            command(i, 3) = float(speed);
            // Bound the feedforward and the combined effort with the transmitted PD fields.
            const double actual_pd = command(i, 0) * (command(i, 1) - q) + command(i, 2) * (command(i, 3) - dq);
            command(i, 4) = float(std::clamp(effort - actual_pd, -limit, limit));
        }
        if (!command.allFinite()) {
            failed_ = true;
            msfb_.SetHandsetReady(false);
            return;
        }
        ri_ptr_->SetJointCommand(command);

        const bool at_goal = t >= 1.f && (measured - goal_).cwiseAbs().maxCoeff() <= kPositionTolerance &&
            measured_velocity.cwiseAbs().maxCoeff() <= kVelocityTolerance;
        if (!at_goal) settled_since_.reset();
        else if (!settled_since_) settled_since_ = now;
        ready_ = settled_since_ && now - *settled_since_ >= kSettleSeconds;
        msfb_.SetHandsetReady(ready_);
    }

    bool LoseControlJudge() override {
        const auto& user = *uc_ptr_->GetUserCommand();
        return failed_ || user.safe_control_mode != 0 || user.target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        return ready_ && uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::Sit2StandMimic
            ? StateName::kSit2StandMimic : StateName::kSquatPreparation;
    }

private:
    static constexpr int kJoints = 21;
    static constexpr double kPreparationSeconds = 1.0;
    static constexpr double kSettleSeconds = .15;
    static constexpr float kPositionTolerance = .15f;
    static constexpr float kVelocityTolerance = .3f;
    using JointVector = Eigen::Matrix<float, kJoints, 1>;
    JointVector start_, goal_, kp_, kd_, lower_, upper_;
    double start_time_ = 0;
    std::optional<double> settled_since_;
    bool ready_ = false;
    bool failed_ = false;
};

}  // namespace humanoid_std
