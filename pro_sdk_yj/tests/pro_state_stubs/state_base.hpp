#pragma once
// Isolated services for the actual migrated state headers. No DDS or hardware.
#include "basic_function.hpp"
#include "parameters/control_parameters.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
using namespace types;

class MotionStateStore {
    std::atomic<RobotMotionState> state_{RobotMotionState::Idle};
public:
    std::atomic<bool> handset_ready{false};
    void SetState(RobotMotionState state) { state_ = state; handset_ready = false; }
    RobotMotionState GetState() const { return state_.load(); }
    void MarkHandsetReady() { handset_ready = true; }
    void SetHandsetReady(bool ready) { handset_ready = ready; }
};
struct ProTestRobot {
    mutable std::mutex mutex;
    VecXf q = VecXf::Zero(31), dq = VecXf::Zero(31), tau = VecXf::Zero(31);
    MatXf command = MatXf::Zero(31, 5);
    std::atomic<double> time{0};
    std::atomic<int> packets{0};
    std::atomic<int> driver_resets{0};
    std::atomic<int> joint_zero_calls{0};
    std::atomic<int> position_offset_calls{0};
    std::atomic<int> wrap_offset_calls{0};
    std::atomic<uint64_t> last_zero_mask{0};
    std::atomic<uint64_t> last_offset_mask{0};
    VecXf GetJointPosition() const { std::lock_guard<std::mutex> lock(mutex); return q; }
    VecXf GetJointVelocity() const { std::lock_guard<std::mutex> lock(mutex); return dq; }
    VecXf GetJointTorque() const { std::lock_guard<std::mutex> lock(mutex); return tau; }
    Vec3f GetImuRpy() const { return Vec3f::Zero(); }
    Vec3f GetImuOmega() const { return Vec3f::Zero(); }
    Vec3f GetImuAcc() const { return Vec3f(0, 0, 9.815f); }
    double GetInterfaceTimeStamp() const { return time.load(); }
    void ResetJointError() { ++driver_resets; }
    void SetJointZero(uint64_t mask) {
        last_zero_mask = mask;
        ++joint_zero_calls;
    }
    void InitPositionOffset(uint64_t mask) {
        last_offset_mask = mask;
        ++position_offset_calls;
    }
    void WrapPositionOffset() { ++wrap_offset_calls; }
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
