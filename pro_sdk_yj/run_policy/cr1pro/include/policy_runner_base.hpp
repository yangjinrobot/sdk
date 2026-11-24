#pragma once

#include "common_types.h"
#include "custom_types.h"
#include "basic_function.hpp"
#include "policy_paths.hpp"
#include "policy_validation.hpp"
#include <atomic>
#include <cmath>
#include <mutex>

using namespace types;

class PolicyRunnerBase {
public:
    PolicyRunnerBase(std::string policy_name, RobotName robot_name);
    virtual ~PolicyRunnerBase();
    virtual void DisplayPolicyInfo() = 0;
    virtual void OnEnter() = 0;
    virtual void OnExit() = 0;
    virtual bool IsReady() const;
    virtual void SetDecimation(int decimation);

    // Worker threads use this boundary: failures latch an error and produce
    // damping for the state machine to switch into JointDamping safely.
    RobotAction GetRobotAction(const RobotBasicState& state, const UserCommand& command);

    bool HasInferenceFault() const;
    std::string GetLastError() const;

    const std::string policy_name_;
    const RobotName robot_name_;
    int decimation_ = 1;
    int run_cnt_ = 0;
    int run_cnt_record_ = 0;
    Vec3f vel_delta_const_, cmd_vel_input_;

protected:
    virtual RobotAction ComputeRobotAction(const RobotBasicState&, const UserCommand&) = 0;
    static RobotAction DampingAction();
    void ResetInferenceFault();
    void SetInferenceFault(const std::string& error);

    // Commands below 0.1 are held at zero in policy observations. Keep the
    // filter state unchanged so a later valid command can still ramp normally.
    static float ApplyCommandDeadband(float velocity) {
        return std::abs(velocity) < 0.1f ? 0.0f : velocity;
    }

    static Vec3f ApplyCommandDeadband(const Vec3f& velocity) {
        return Vec3f(ApplyCommandDeadband(velocity.x()),
                     ApplyCommandDeadband(velocity.y()),
                     ApplyCommandDeadband(velocity.z()));
    }

private:
    std::atomic<bool> inference_fault_{false};
    mutable std::mutex error_mutex_;
    std::string last_error_;
};
