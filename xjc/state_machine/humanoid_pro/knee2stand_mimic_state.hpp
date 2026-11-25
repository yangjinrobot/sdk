#pragma once

#include "state_base.hpp"
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include "humanoid_pro/bounded_policy_command.hpp"

class Knee2StandPolicyRunner;

namespace humanoid {

class Knee2StandMimicState final : public StateBase {
public:
    Knee2StandMimicState(const RobotName& robot_name, const std::string& state_name,
                         std::shared_ptr<ControllerData> data_ptr);
    ~Knee2StandMimicState() override;
    void OnEnter() override;
    void OnExit() override;
    void Run() override;
    bool LoseControlJudge() override;
    StateName GetNextStateName() override;

private:
    RobotBasicState ReadObservation() const;
    void SendDamping();
    void InferAndSend(const RobotBasicState& state, const UserCommand& command);
    void Fail();
    void PolicyRunner();

    std::shared_ptr<Knee2StandPolicyRunner> policy_ptr_;
    BoundedPolicyCommand bounded_command_;
    RobotBasicState rbs_{};
    UserCommand user_command_{};
    std::mutex policy_input_mutex_;
    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<bool> policy_available_{false};
    int state_run_cnt_ = 0;
    bool policy_entered_ = false;
};

} // namespace humanoid
