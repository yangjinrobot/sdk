/**
 * @file upper_body_control_state.hpp
 * @brief upper body control state
 * @author DeepRobotics
 * @version 1.0
 * @date 2026-04-03
 */
#pragma once

#include "state_base.hpp"
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>

namespace humanoid {

class UpperBodyControlState final : public StateBase {
public:
    UpperBodyControlState(const RobotName& robot_name, const std::string& state_name,
                          std::shared_ptr<ControllerData> data_ptr);
    ~UpperBodyControlState() override;
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

    UserCommand user_command_{};
    std::mutex user_command_mutex_;

    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<bool> worker_failed_{false};

    int observation_history_index_ = 0;
};

} // namespace humanoid
