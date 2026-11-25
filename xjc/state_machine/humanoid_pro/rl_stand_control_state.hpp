/**
 * @file rl_stand_control_state.hpp
 * @brief rl policy runnning state
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */
#pragma once

#include "state_base.hpp"
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>

class RLStandPolicyRunner;

namespace humanoid {

class RLStandControlState final : public StateBase {
public:
    RLStandControlState(const RobotName& robot_name, const std::string& state_name,
                        std::shared_ptr<ControllerData> data_ptr);
    ~RLStandControlState() override;
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

    std::shared_ptr<RLStandPolicyRunner> policy_ptr_;

    UserCommand user_command_{};
    std::mutex user_command_mutex_;

    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<bool> policy_available_{false};
    bool policy_entered_ = false;

    int observation_history_index_ = 0;
};

} // namespace humanoid
