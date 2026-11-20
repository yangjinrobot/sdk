#ifndef USER_COMMAND_INTERFACE_H_
#define USER_COMMAND_INTERFACE_H_

#include <atomic>
#include <mutex>
#include <utility>
#include "common_types.h"
#include "custom_types.h"
using namespace types;
#include "motion_state_feedback.hpp"
#include "motion_state_store.hpp"

class UdpNode;

namespace interface {
class UserCommandInterface {
public:
    explicit UserCommandInterface(RobotName robot_name = RobotName::CR1PRO,
                                  ControlMode mode = ControlMode::kWholeBody)
        : robot_name_(robot_name), control_mode_(mode), usr_cmd_(new UserCommand{}) {}
    virtual ~UserCommandInterface() { delete usr_cmd_; }
    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void EnableDdsInput(bool = true) {}

    // Compatibility view for existing read-only data streaming code.
    virtual UserCommand* GetUserCommand() { return usr_cmd_; }
    UserCommand GetUserCommandSnapshot() const {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        return *usr_cmd_;
    }
    template<class Update> decltype(auto) UpdateCommand(Update&& update) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        return std::forward<Update>(update)(*usr_cmd_);
    }
    void SetSafeControlMode(uint32_t mode) {
        UpdateCommand([mode](UserCommand& command) { command.safe_control_mode = mode; });
    }
    void ResetUserCommand() {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        UpdateCommand([](UserCommand& command) { command = UserCommand{}; });
        emergency_damping_pending_ = false;
        ResetGaitPending();
        ResetActionPending();
    }
    // Keep an emergency request until the FSM acknowledges a passive state.
    // Keyboard, gamepad, UDP and DDS must all use this mode-request boundary.
    bool SetTargetModeRequest(uint32_t state) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (!IsKnownRobotMotionState(state)) return false;
        const bool pending = EmergencyDampingPending();
        if (pending && state != RobotMotionState::JointDamping) return false;
        usr_cmd_->target_mode = state;
        if (state == RobotMotionState::JointDamping) {
            const auto current = CurrentMotionState();
            emergency_damping_pending_ = current != RobotMotionState::Idle && current != RobotMotionState::JointDamping;
            usr_cmd_->forward_vel_scale = usr_cmd_->side_vel_scale = usr_cmd_->turnning_vel_scale = 0.f;
        }
        return true;
    }
    bool EmergencyDampingPending() {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        const auto current = CurrentMotionState();
        if (current == RobotMotionState::Idle || current == RobotMotionState::JointDamping)
            emergency_damping_pending_ = false;
        return emergency_damping_pending_;
    }
    virtual void SetMotionStateFeedback(MotionStateFeedback* feedback) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        msfb_ = feedback;
    }
    virtual void SetMotionStateStore(const std::shared_ptr<MotionStateStore>& store) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        motion_state_store_ = store;
    }
    virtual void SetUdpRemote(UdpNode* remote) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        udp_remote_ = remote;
    }
    void SetControlMode(ControlMode mode) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        control_mode_ = mode;
    }
    void SetTargetGait(uint32_t gait) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        usr_cmd_->target_gait = gait;
        gait_pending_.store(true, std::memory_order_release);
    }
    bool ConsumeGaitEvent(uint32_t& gait) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (!gait_pending_.exchange(false, std::memory_order_acq_rel)) return false;
        gait = usr_cmd_->target_gait;
        return true;
    }
    void ResetGaitPending() { gait_pending_.store(false, std::memory_order_release); }
    void SetTargetAction(uint32_t action) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        usr_cmd_->target_action = action;
        action_pending_.store(true, std::memory_order_release);
    }
    bool ConsumeActionEvent(uint32_t& action) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (!action_pending_.exchange(false, std::memory_order_acq_rel)) return false;
        action = usr_cmd_->target_action;
        return true;
    }
    void ResetActionPending() { action_pending_.store(false, std::memory_order_release); }

    mutable std::recursive_mutex command_mutex_;
    MotionStateFeedback* msfb_ = nullptr;
    RobotName robot_name_;
    ControlMode control_mode_;
    std::shared_ptr<MotionStateStore> motion_state_store_;
    UdpNode* udp_remote_ = nullptr;
    UserCommand* usr_cmd_;
    std::atomic<bool> gait_pending_{false};
    std::atomic<bool> action_pending_{false};

private:
    RobotMotionState CurrentMotionState() const {
        if (motion_state_store_) return motion_state_store_->GetState();
        return msfb_ ? static_cast<RobotMotionState>(msfb_->GetCurrentState()) : RobotMotionState::Idle;
    }
    bool emergency_damping_pending_ = false;  // Protected by command_mutex_.
};
}  // namespace interface
#endif
