/**
 * @file joint_damping_state.hpp
 * @brief joint passive control state
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

class JointDampingState final : public StateBase {
public:
    JointDampingState(const RobotName& robot_name, const std::string& state_name,
                      std::shared_ptr<ControllerData> data_ptr);
    ~JointDampingState() override;
    void OnEnter() override;
    void OnExit() override;
    void Run() override;
    bool LoseControlJudge() override;
    StateName GetNextStateName() override;

private:
    float state_enter_time_, run_time_;
    MatXf joint_cmd_;
};

} // namespace humanoid
