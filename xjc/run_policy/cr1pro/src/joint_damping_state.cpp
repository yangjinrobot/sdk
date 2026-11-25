#include "humanoid/joint_damping_state.hpp"

namespace humanoid {

JointDampingState::JointDampingState(const RobotName& robot_name, const std::string& state_name,
                                     std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr) {
    VecXf joint_kd = VecXf::Zero(cp_ptr_->dof_num_);
    if (robot_name_ == RobotName::CR1PRO)
        joint_kd << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_,
            cp_ptr_->leg_kd_, cp_ptr_->neck_kd_;

    joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
    joint_cmd_.col(2) = joint_kd;
}

JointDampingState::~JointDampingState() = default;

void JointDampingState::OnEnter() {
    state_enter_time_ = ri_ptr_->GetInterfaceTimeStamp();
    run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    motion_state_store_->SetState(RobotMotionState::JointDamping);
}

void JointDampingState::OnExit() {}

void JointDampingState::Run() {
    run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    ri_ptr_->SetJointCommand(joint_cmd_);
}

bool JointDampingState::LoseControlJudge() {
    return false;
}

StateName JointDampingState::GetNextStateName() {
    if (run_time_ - state_enter_time_ < 10.) {
        return StateName::kJointDamping;
    }
    return StateName::kIdle;
}

} // namespace humanoid
