/**
 * @file joint_damping_state.hpp
 * @brief joint passive control state
 * @author zhenglinqiang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_JOINT_DAMPING_STATE_HPP_
#define HUMANOID_JOINT_DAMPING_STATE_HPP_

#include "state_base.h"

namespace humanoid{
class JointDampingState : public StateBase{
private:
    float time_record_, run_time_;
    MatXf joint_cmd_;
public:
    JointDampingState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            VecXf kd_ = VecXf::Zero(cp_ptr_->dof_num_);
            if(cp_ptr_->dof_num_ == 31 || cp_ptr_->dof_num_ == 30) kd_<<1.5*cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, 2*cp_ptr_->leg_kd_, 2*cp_ptr_->leg_kd_, cp_ptr_->neck_kd_;
            else if(cp_ptr_->dof_num_ == 21) kd_<<1.5*cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, 2*cp_ptr_->leg_kd_, 2*cp_ptr_->leg_kd_;
            else kd_<<cp_ptr_->leg_kd_ , cp_ptr_->leg_kd_;
            // std::cout<<"kd_:"<<kd_.transpose()<<std::endl;
            joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
            joint_cmd_.col(2) = kd_;
        }
    ~JointDampingState(){}

    virtual void OnEnter() {
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        //记录故障快照
        ri_ptr_->WriteSnapShot();
        msfb_.UpdateCurrentState(RobotMotionState::JointDamping);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_record_ < 10.){
            return StateName::kJointDamping;
        }        
        uc_ptr_->GetUserCommand()->target_mode = RobotMotionState::WaitingForStand;
        return StateName::kIdle;
    }
};


};//namespace humanoid;


#endif