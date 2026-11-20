#pragma once

#include "common_types.h"
#include <algorithm>
#include <chrono>
#include <stdexcept>

// Blend the last published joint command into the latest policy command.
// The row and start offset are configurable, so the same helper can handle
// either the 8 arm joints or the full 21-joint body.
class JointEntryTransition {
public:
    using Clock = std::chrono::steady_clock;

    void Reset(const types::MatXf& previous_joints, Clock::time_point now = Clock::now()) {
        active_ = false;
        if (previous_joints.rows() == 0 || previous_joints.cols() != 5 || !previous_joints.allFinite())
            throw std::runtime_error("Invalid previous joint command");
        entry_command_ = previous_joints;
        start_ = now;
        active_ = true;
    }

    void Reset(const types::MatXf& previous_joints, double now_seconds) {
        active_ = false;
        if (previous_joints.rows() == 0 || previous_joints.cols() != 5 || !previous_joints.allFinite())
            throw std::runtime_error("Invalid previous joint command");
        entry_command_ = previous_joints;
        start_seconds_ = now_seconds;
        active_ = true;
    }

    void Apply(types::MatXf& command, int first_joint,
               Clock::time_point now = Clock::now()) {
        if (!active_) return;
        const double elapsed = std::chrono::duration<double>(now - start_).count();
        ApplyElapsed(command, first_joint, elapsed);
    }

    void Apply(types::MatXf& command, int first_joint, double now_seconds) {
        if (!active_) return;
        ApplyElapsed(command, first_joint, now_seconds - start_seconds_);
    }

    void Cancel() { active_ = false; }
    bool active() const { return active_; }

    static constexpr double kDurationSeconds = 1.0;

private:
    void ApplyElapsed(types::MatXf& command, int first_joint, double elapsed) {
        if (!active_) return;
        if (first_joint < 0 || command.rows() < first_joint + entry_command_.rows() || command.cols() != 5) {
            active_ = false;
            return;
        }
        const float s = static_cast<float>(std::clamp(elapsed / kDurationSeconds, 0.0, 1.0));
        const float weight = s * s * (3.0f - 2.0f * s);
        // Columns: kp, target position, kd, target velocity, feedforward torque.
        command.block(first_joint, 0, entry_command_.rows(), 5) =
            (1.0f - weight) * entry_command_ +
            weight * command.block(first_joint, 0, entry_command_.rows(), 5);
        if (s >= 1.0f) active_ = false;
    }

    types::MatXf entry_command_;
    Clock::time_point start_;
    double start_seconds_ = 0.0;
    bool active_ = false;
};

// Backward-compatible name for existing arm-only call sites and tests.
using ArmEntryTransition = JointEntryTransition;

