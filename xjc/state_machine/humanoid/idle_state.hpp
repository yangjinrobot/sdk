/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-12-12
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once

#include "state_base.hpp"
#include <memory>
#include "drdds/core/drdds_core.h"
#include <cstdint>
#include <mutex>

namespace humanoid {

class IdleState final : public StateBase {
public:
    IdleState(const RobotName& robot_name, const std::string& state_name,
              std::shared_ptr<ControllerData> data_ptr);
    ~IdleState() override;
    void OnEnter() override;
    void OnExit() override;
    void Run() override;
    bool LoseControlJudge() override;
    StateName GetNextStateName() override;

private:
    void HandlerMarkZero(const std_msgs::msg::UInt64* data);
    void ProcessMarkZero();
    void GetProprioceptiveData();
    void RecordJointData();

    VecXf joint_pos_init_, joint_vel_init_, kp_, kd_, joint_pos_, joint_vel_, joint_tau_;
    MatXf joint_cmd_;
    static constexpr double kDriverResetIntervalSeconds = 3.0;
    double last_driver_reset_time_ = 0.0;
    std::mutex mark_zero_mutex_;
    uint64_t pending_mark_zero_mask_ = 0;
    bool mark_zero_enabled_ = false;
    // Destroy the subscriber before the callback's mutex and pending state.
    std::unique_ptr<ChannelUInt64> mark_zero_sub_;
};

} // namespace humanoid
