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
#ifndef HUMANOID_PRO_JOINT_DAMPING_STATE_HPP_
#define HUMANOID_PRO_JOINT_DAMPING_STATE_HPP_

#include "state_base.hpp"

namespace humanoid_pro {
class JointDampingState : public StateBase {
 private:
  float time_record_, run_time_;
  MatXf joint_cmd_;

 public:
  JointDampingState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
      : StateBase(robot_name, state_name, data_ptr) {
    VecXf kd_ = VecXf::Zero(cp_ptr_->dof_num_);
    if(robot_name_ == RobotName::CR1PRO)
      kd_<<cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_, cp_ptr_->neck_kd_;

    joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
    joint_cmd_.col(2) = kd_;
  }

  ~JointDampingState() {}

  void OnEnter() override {
    time_record_ = ri_ptr_->GetInterfaceTimeStamp();
    run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    motion_state_store_->SetState(RobotMotionState::JointDamping);
  }

  void OnExit() override {}

  void Run() override {
    run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    ri_ptr_->SetJointCommand(joint_cmd_);
  }

  bool LoseControlJudge() override { return false; }

  StateName GetNextStateName() override {
    if (run_time_ - time_record_ < 10.) {
      return StateName::kJointDamping;
    }
    return StateName::kIdle;
  }
};
}  // namespace humanoid_pro

#endif
