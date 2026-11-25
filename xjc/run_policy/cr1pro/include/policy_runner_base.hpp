#pragma once

#include "common_types.h"
#include "custom_types.h"
#include "basic_function.hpp"
#include "policy_paths.hpp"
#include "policy_validation.hpp"
#include <atomic>
#include <mutex>

using namespace types;

class PolicyRunnerBase {
public:
    PolicyRunnerBase(std::string policy_name, RobotName robot_name)
        : policy_name_(std::move(policy_name)), robot_name_(robot_name) {
        if (robot_name != RobotName::CR1PRO)
            throw std::invalid_argument("CR1PRO policies require RobotName::CR1PRO");
        vel_delta_const_ << 0.3, 0.2, 0.3;
        cmd_vel_input_.setZero();
    }
    virtual ~PolicyRunnerBase() = default;
    virtual void DisplayPolicyInfo() = 0;
    virtual void OnEnter() = 0;
    virtual void OnExit() = 0;
    virtual bool IsReady() const { return !HasInferenceFault(); }
    virtual void SetDecimation(int decimation) {
        if (decimation <= 0) throw std::invalid_argument("policy decimation must be positive");
        decimation_ = decimation;
    }

    // Worker threads use this boundary: failures latch an error and produce
    // damping for the state machine to switch into JointDamping safely.
    RobotAction GetRobotAction(const RobotBasicState& state, const UserCommand& command) {
        if (HasInferenceFault()) return DampingAction();
        try {
            if (!IsReady()) throw std::runtime_error("policy is not ready: " + policy_name_);
            pro_policy::ValidateRobotState(state, command);
            RobotAction action = ComputeRobotAction(state, command);
            pro_policy::ValidateRobotAction(action);
            return action;
        } catch (const std::exception& error) {
            SetInferenceFault(error.what());
        } catch (...) {
            SetInferenceFault("unknown policy inference failure");
        }
        return DampingAction();
    }

    bool HasInferenceFault() const { return inference_fault_.load(std::memory_order_acquire); }
    std::string GetLastError() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return last_error_;
    }

    const std::string policy_name_;
    const RobotName robot_name_;
    int decimation_ = 1;
    int run_cnt_ = 0;
    int run_cnt_record_ = 0;
    Vec3f vel_delta_const_, cmd_vel_input_;

protected:
    virtual RobotAction ComputeRobotAction(const RobotBasicState&, const UserCommand&) = 0;
    static RobotAction DampingAction() {
        RobotAction damping;
        damping.goal_joint_pos = VecXf::Zero(29);
        damping.goal_joint_vel = VecXf::Zero(29);
        damping.kp = VecXf::Zero(29);
        damping.kd = VecXf::Ones(29);
        damping.tau_ff = VecXf::Zero(29);
        return damping;
    }
    void ResetInferenceFault() {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
        inference_fault_.store(false, std::memory_order_release);
    }
    void SetInferenceFault(const std::string& error) {
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = error;
        }
        if (!inference_fault_.exchange(true, std::memory_order_acq_rel))
            std::cerr << "[CR1PRO policy] " << policy_name_ << ": " << error << std::endl;
    }

private:
    std::atomic<bool> inference_fault_{false};
    mutable std::mutex error_mutex_;
    std::string last_error_;
};
