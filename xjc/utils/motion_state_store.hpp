#pragma once

#include <atomic>
#include "custom_types.h"
#include "motion_state_feedback.hpp"

// The imported state graph uses this store to inspect the state it entered from.
// Keep the existing telemetry feedback synchronized with the same state changes.
class MotionStateStore {
public:
    explicit MotionStateStore(MotionStateFeedback* feedback = nullptr)
        : feedback_(feedback) {}

    void SetState(types::RobotMotionState state) {
        current_state_.store(state, std::memory_order_release);
        if (feedback_) feedback_->UpdateCurrentState(static_cast<int>(state));
    }

    types::RobotMotionState GetState() const {
        return current_state_.load(std::memory_order_acquire);
    }

private:
    std::atomic<types::RobotMotionState> current_state_{types::RobotMotionState::Idle};
    MotionStateFeedback* feedback_;
};
