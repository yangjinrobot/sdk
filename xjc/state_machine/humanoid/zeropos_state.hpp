/**
 * @file ZeroPosState.hpp
 * @brief
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

namespace humanoid {

class ZeroPosState final : public StateBase {
public:
    ZeroPosState(const RobotName& robot_name, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr);
    ~ZeroPosState() override;
    void OnEnter() override;
    void OnExit() override;
    void Run() override;
    bool LoseControlJudge() override;
    StateName GetNextStateName() override;

private:
    void GetRobotJointValue();
    void RecordJointData();

    float state_enter_time_, run_time_;
    VecXf joint_pos_init_, joint_vel_init_, joint_pos_, joint_vel_;
    VecXf kp_, kd_, joint_pos_goal_;
    MatXf joint_cmd_;
};

} // namespace humanoid
