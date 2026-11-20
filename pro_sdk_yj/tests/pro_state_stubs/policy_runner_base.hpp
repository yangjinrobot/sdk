#pragma once
#include "state_base.hpp"
#include <stdexcept>

// Only inference is substituted: state lifecycle, transition and command assembly
// come from deployment headers. Fault injection exercises their worker boundary.
struct ProPolicyFaults {
    inline static std::atomic<bool> ready{true};
    inline static std::atomic<int> failure{0}; // 1=throw, 2=IsReady false, 3=bad shape
    inline static std::atomic<int> active_calls{0};
    inline static std::atomic<bool> unsafe_exit{false};
    inline static std::atomic<bool> salute_lowering_requested{false}, salute_lowering_complete{false};
    static void Reset() {
        ready = true; failure = 0; unsafe_exit = false;
        salute_lowering_requested = salute_lowering_complete = false;
    }
};
class PolicyRunnerBase {
public:
    int decimation_ = 20;
    PolicyRunnerBase(const std::string&, RobotName) {}
    virtual ~PolicyRunnerBase() = default;
    bool inference_fault_ = false;
    bool IsReady() const { return ProPolicyFaults::ready.load() && !inference_fault_; }
    void SetCmdMaxVel(const Vec3f&) {}
    void DisplayPolicyInfo() {}
    void SetInitialArmTargets(const Vec7f&, const Vec7f&) {}
    VecXf GetDefaultJointPosition() const { return VecXf::Zero(29); }
    void OnEnter() { inference_fault_ = false; }
    void OnExit() {
        if (ProPolicyFaults::active_calls.load() != 0) ProPolicyFaults::unsafe_exit = true;
    }
    RobotAction GetRobotAction(const RobotBasicState& observation, const UserCommand&) {
        if (observation.joint_pos.size() != 31 || !observation.joint_pos.allFinite())
            throw std::runtime_error("incoherent PRO observation");
        struct ActiveCall {
            ActiveCall() { ++ProPolicyFaults::active_calls; }
            ~ActiveCall() { --ProPolicyFaults::active_calls; }
        } call;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const int failure = ProPolicyFaults::failure.load();
        if (failure == 1) throw std::runtime_error("injected inference exception");
        if (failure == 2) inference_fault_ = true;
        const int rows = failure == 3 ? 28 : 29;
        RobotAction action;
        action.goal_joint_pos = VecXf::Constant(rows, .12f);
        action.goal_joint_vel = VecXf::Zero(rows);
        action.kp = VecXf::Constant(rows, 3);
        action.kd = VecXf::Constant(rows, 4);
        action.tau_ff = VecXf::Zero(rows);
        return action;
    }
};
