#pragma once
// Isolated adapters for exercising the real GunMimicState without DDS/hardware.
// Only the test executable adds this directory before the production include path.
#include "basic_function.hpp"
#include "motion_state_feedback.hpp"

struct TestRobot {
    VecXf q = VecXf::Zero(21), dq = VecXf::Zero(21);
    Vec3f rpy = Vec3f::Zero(), omega = Vec3f::Zero();
    MatXf command;
    double time = 0;
    int publishes = 0;
    VecXf GetJointPosition() { return q; }
    VecXf GetJointVelocity() { return dq; }
    Vec3f GetImuRpy() { return rpy; }
    Vec3f GetImuOmega() { return omega; }
    double GetInterfaceTimeStamp() { return time; }
    MatXf GetJointCommand() { return command; }
    void SetJointCommand(const MatXf& value) { command = value; ++publishes; }
};
struct TestUser {
    UserCommand command{};
    UserCommand* GetUserCommand() { return &command; }
};
struct ControllerData {
    std::shared_ptr<TestRobot> ri_ptr;
    std::shared_ptr<TestUser> uc_ptr;
    std::shared_ptr<ControlParameters> cp_ptr;
};
class StateBase {
public:
    RobotName robot_name_;
    std::shared_ptr<TestRobot> ri_ptr_;
    std::shared_ptr<TestUser> uc_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    inline static MotionStateFeedback msfb_{};
    StateBase(RobotName robot, const std::string&, std::shared_ptr<ControllerData> data)
        : robot_name_(robot), ri_ptr_(data->ri_ptr), uc_ptr_(data->uc_ptr), cp_ptr_(data->cp_ptr) {}
    virtual ~StateBase() = default;
    virtual void OnEnter() = 0;
    virtual void OnExit() = 0;
    virtual void Run() = 0;
    virtual bool LoseControlJudge() = 0;
    virtual StateName GetNextStateName() = 0;
};
