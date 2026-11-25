#pragma once

#include "state_base.hpp"
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>

class AmpMoe15DofPolicyRunner;

namespace humanoid {

class AmpMoe15DofState final : public StateBase {
public:
    AmpMoe15DofState(const RobotName& robot_name, const std::string& state_name,
                     std::shared_ptr<ControllerData> data_ptr);
    ~AmpMoe15DofState() override;
    void OnEnter() override;
    void OnExit() override;
    void Run() override;
    bool LoseControlJudge() override;
    StateName GetNextStateName() override;

private:
    RobotBasicState ReadObservation();
    void InferAndSend(const RobotBasicState& state, const UserCommand& command);
    void PolicyRunner();

    std::shared_ptr<AmpMoe15DofPolicyRunner> policy_ptr_;
    RobotBasicState rbs_{};
    UserCommand user_command_{};
    std::mutex policy_input_mutex_;
    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<bool> policy_available_{false};
    std::atomic<int> state_run_cnt_{0};
    bool policy_entered_ = false;
};

} // namespace humanoid
