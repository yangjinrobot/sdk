#pragma once
#include <atomic>
#include <mutex>
#include "common_types.h"

// Updated by the controller/safety threads and read by the telemetry threads.
class MotionStateFeedback {
    std::atomic<uint32_t> current_state_{0}, current_gait_{0}, current_action_{0}, current_pose_{0};
    std::atomic<uint32_t> last_state_{0}, last_gait_{0}, last_action_{0}, error_code_{0};
    mutable std::mutex velocity_mutex_;
    types::Vec3f current_vel_ = types::Vec3f::Zero();
public:
    void UpdateCurrentError(uint32_t code) { error_code_.store(code); }
    void UpdateCurrentState(int state) {
        const auto next = static_cast<uint32_t>(state);
        const auto previous = current_state_.exchange(next);
        if (previous != next) last_state_.store(previous);
    }
    void UpdateCurrentGait(uint32_t gait) {
        const auto previous = current_gait_.exchange(gait);
        if (previous != gait) last_gait_.store(previous);
    }
    void UpdateCurrentAction(uint32_t action) {
        const auto previous = current_action_.exchange(action);
        if (previous != action) last_action_.store(previous);
    }
    void UpdateCurrentPose(uint32_t pose) { current_pose_.store(pose); }
    void UpdateCurrentVel(types::Vec3f velocity) {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        current_vel_ = velocity;
    }
    void ClearMotionError() { error_code_.store(0); }
    void SetJointPosLimitError() { error_code_.fetch_or(1u); }
    void SetPostureLimitError() { error_code_.fetch_or(2u); }
    uint32_t GetCurrentState() const { return current_state_.load(); }
    uint32_t GetCurrentAction() const { return current_action_.load(); }
    uint32_t GetCurrentPose() const { return current_pose_.load(); }
    uint32_t GetCurrentGait() const { return current_gait_.load(); }
    uint32_t GetLastState() const { return last_state_.load(); }
    uint32_t GetLastGait() const { return last_gait_.load(); }
    float GetCurrentVelX() const {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        return current_vel_[0];
    }
    float GetCurrentVelY() const {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        return current_vel_[1];
    }
};
