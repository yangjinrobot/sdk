#pragma once

#include <atomic>

#include "custom_types.h"

class MotionStateStore {
public:
    MotionStateStore() : current_state_(types::RobotMotionState::Idle) {}

    void SetState(types::RobotMotionState state) { current_state_.store(state, std::memory_order_release); }

    types::RobotMotionState GetState() const { return current_state_.load(std::memory_order_acquire); }

private:
    std::atomic<types::RobotMotionState> current_state_;
};
