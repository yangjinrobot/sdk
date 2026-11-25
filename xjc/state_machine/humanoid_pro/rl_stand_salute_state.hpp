#pragma once

#include "state_base.hpp"
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>

class RLStandSalutePolicyRunner;

namespace humanoid {

class RLStandSaluteState final : public StateBase {
public:
    RLStandSaluteState(const RobotName& robot_name, const std::string& state_name,
                       std::shared_ptr<ControllerData> data_ptr);
    RLStandSaluteState(const RobotName& robot_name, const std::string& state_name,
                       std::shared_ptr<ControllerData> data_ptr,
                       std::shared_ptr<RLStandSalutePolicyRunner> policy_ptr);
    ~RLStandSaluteState() override;
    void OnEnter() override;
    void OnExit() override;
    void Run() override;
    bool LoseControlJudge() override;
    StateName GetNextStateName() override;

private:
    void InitializeObservation();
    void UpdateRobotObservation();
    void PolicyRunner();

    RobotBasicState rbs_{};
    std::mutex rbs_mutex_;
    std::atomic<int> state_run_cnt_{-1};

    std::shared_ptr<RLStandSalutePolicyRunner> policy_ptr_;

    UserCommand user_command_{};
    std::mutex user_command_mutex_;

    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<bool> policy_available_{false};
    bool policy_entered_ = false;

    int observation_history_index_ = 0;
    bool walking_requested_ = false;
    bool gesture_transition_ = false;
};

} // namespace humanoid
