#pragma once
// Isolated services for the actual migrated state headers. No DDS or hardware.
#include "basic_function.hpp"
#include "parameters/control_parameters.h"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
using namespace types;

class MotionStateStore {
    std::atomic<RobotMotionState> state_{RobotMotionState::Idle};
public:
    void SetState(RobotMotionState state) { state_ = state; }
    RobotMotionState GetState() const { return state_.load(); }
};
struct ProTestRobot {
    mutable std::mutex mutex;
    VecXf q = VecXf::Zero(31), dq = VecXf::Zero(31), tau = VecXf::Zero(31);
    MatXf command = MatXf::Zero(31, 5);
    std::atomic<double> time{0};
    std::atomic<int> packets{0};
    std::atomic<int> driver_resets{0};
    std::vector<uint64_t> zero_masks, offset_masks;
    std::vector<std::string> zero_events;
    MatXf command_at_zero;
    std::chrono::steady_clock::time_point zero_time, offset_time;
    std::function<void()> on_set_zero, on_wrap_offset;
    VecXf GetJointPosition() const { std::lock_guard<std::mutex> lock(mutex); return q; }
    VecXf GetJointVelocity() const { std::lock_guard<std::mutex> lock(mutex); return dq; }
    VecXf GetJointTorque() const { std::lock_guard<std::mutex> lock(mutex); return tau; }
    Vec3f GetImuRpy() const { return Vec3f::Zero(); }
    Vec3f GetImuOmega() const { return Vec3f::Zero(); }
    Vec3f GetImuAcc() const { return Vec3f(0, 0, 9.815f); }
    double GetInterfaceTimeStamp() const { return time.load(); }
    void ResetJointError() { ++driver_resets; }
    void SetJointZero(uint64_t mask) {
        zero_masks.push_back(mask);
        zero_events.push_back("zero");
        command_at_zero = GetJointCommand();
        zero_time = std::chrono::steady_clock::now();
        if (on_set_zero) on_set_zero();
    }
    void InitPositionOffset(uint64_t mask) {
        offset_masks.push_back(mask);
        zero_events.push_back("offset");
        offset_time = std::chrono::steady_clock::now();
    }
    void WrapPositionOffset() {
        zero_events.push_back("wrap");
        if (on_wrap_offset) on_wrap_offset();
    }
    MatXf GetJointCommand() const { std::lock_guard<std::mutex> lock(mutex); return command; }
    void SetJointCommand(const MatXf& value) {
        std::lock_guard<std::mutex> lock(mutex);
        command = value;
        ++packets;
    }
};
struct ProTestUser {
    mutable std::mutex mutex;
    UserCommand command{};
    UserCommand GetUserCommandSnapshot() const { std::lock_guard<std::mutex> lock(mutex); return command; }
    void Set(uint32_t target, uint32_t safe = 0) {
        std::lock_guard<std::mutex> lock(mutex);
        command.target_mode = target;
        command.safe_control_mode = safe;
    }
};
class UdpNode;
struct ControllerData {
    std::shared_ptr<ProTestRobot> ri_ptr = std::make_shared<ProTestRobot>();
    std::shared_ptr<ProTestUser> uc_ptr = std::make_shared<ProTestUser>();
    std::shared_ptr<ControlParameters> cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1PRO);
    std::shared_ptr<MotionStateStore> motion_state_store = std::make_shared<MotionStateStore>();
    ControlMode control_mode = ControlMode::kWholeBody;
    UdpNode* udp_node = nullptr;
};
class StateBase {
public:
    RobotName robot_name_;
    std::string state_name_;
    std::shared_ptr<ControllerData> data_ptr_;
    std::shared_ptr<ProTestRobot> ri_ptr_;
    std::shared_ptr<ProTestUser> uc_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<MotionStateStore> motion_state_store_;
    ControlMode control_mode_;
    StateBase(RobotName robot, const std::string& name, std::shared_ptr<ControllerData> data)
        : robot_name_(robot), state_name_(name), data_ptr_(data), ri_ptr_(data->ri_ptr),
          uc_ptr_(data->uc_ptr), cp_ptr_(data->cp_ptr), motion_state_store_(data->motion_state_store),
          control_mode_(data->control_mode) {}
    virtual ~StateBase() = default;
    virtual void OnEnter() = 0;
    virtual void OnExit() = 0;
    virtual void Run() = 0;
    virtual bool LoseControlJudge() = 0;
    virtual StateName GetNextStateName() = 0;
};
