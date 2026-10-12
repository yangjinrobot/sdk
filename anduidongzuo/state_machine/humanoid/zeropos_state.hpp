/**
 * @file zeropos_state.hpp
 * @brief from sit state to stand state
 * @author zhenglinqiang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_ZEROPOS_STATE_HPP_
#define HUMANOID_ZEROPOS_STATE_HPP_

#include "state_base.h"
#include <fstream>
#include <cmath>
#include <ctime>

namespace humanoid{
class ZeroPosState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, kp_, kd_;
    MatXf joint_cmd_;
    float stand_duration_ = 2.;
    timespec start_timestamp, end_timestamp;

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }

public:
    ZeroPosState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            stand_duration_ = cp_ptr_->stand_duration_;
        }
    ~ZeroPosState(){}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::ZeroPos);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();        
        VecXf planning_joint_pos = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf planning_joint_vel = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf planning_joint_tau = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf kp_ = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf kd_ = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf amplitude_ = VecXf::Zero(cp_ptr_->dof_num_);
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        goal_joint_pos_ = VecXf::Zero(cp_ptr_->dof_num_);
        if (cp_ptr_->dof_num_ == 21) {
            // CR1STANDARD hardware order: waist, L-arm(4), R-arm(4), L-leg(6), R-leg(6).
            goal_joint_pos_ <<
                    0.0,
                    0.0,  0.15, 0.0, -0.2,
                    0.0, -0.15, 0.0, -0.2,
                   -0.2, 0.0,  0.0, 0.4, -0.2, 0.0,
                   -0.2, 0.0,  0.0, 0.4, -0.2, 0.0;
        } else {
            goal_joint_pos_(2) = 0.; goal_joint_pos_(6) = -0.0;
            goal_joint_pos_(4) = 0.0; goal_joint_pos_(8) = 0.0;
        }
        //-------------------关节测试-----------------//
        float init_time = 3.;
        float t = run_time_ - time_stamp_record_-init_time;
        if(run_time_ - time_stamp_record_ <= init_time){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, init_time);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, init_time);
            }
        }else{
            for(int i=0;i<current_joint_pos_.rows();++i){planning_joint_pos(i) = goal_joint_pos_(i); planning_joint_vel.setZero();}
        }
        //-------------------关节测试-----------------//

        if(cp_ptr_->dof_num_ == 31 || cp_ptr_->dof_num_ == 30) {
            kp_<<cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_, cp_ptr_->neck_kp_;
            kd_<<cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_, cp_ptr_->neck_kd_;
        }else if(cp_ptr_->dof_num_ == 21) {
            kp_<<cp_ptr_->waist_kp_,cp_ptr_->arm_kp_,cp_ptr_->arm_kp_, cp_ptr_->leg_kp_,cp_ptr_->leg_kp_;
            kd_<<cp_ptr_->waist_kd_,cp_ptr_->arm_kd_,cp_ptr_->arm_kd_, cp_ptr_->leg_kd_,cp_ptr_->leg_kd_;
        }else{
            kp_<<cp_ptr_->leg_kp_,cp_ptr_->leg_kp_;
            kd_<<cp_ptr_->leg_kd_,cp_ptr_->leg_kd_;            
        }

        joint_cmd_.col(0) = 1.0*kp_;
        joint_cmd_.col(2) = 1.0*kd_;  
        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(4) = planning_joint_tau;
        // clock_gettime(CLOCK_MONOTONIC, &start_timestamp);

        // clock_gettime(CLOCK_MONOTONIC, &end_timestamp);
        // std::cout << "solve cost_time:  " << (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1e3 + (end_timestamp.tv_nsec - start_timestamp.tv_nsec) / 1e6 << " ms\n";
        // std::cout<<"joint_cmd_:"<<joint_cmd_.transpose()<<std::endl;
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        // A fallen robot sets kSafePostureNormal (bit 4).  ZeroPos is the
        // explicit staging state for the stand-up policy, so keep it active
        // for that posture-only condition while still damping on every other
        // safety fault.
        const uint32_t non_posture_safety =
            uc_ptr_->GetUserCommand()->safe_control_mode & ~(1u << kSafePostureNormal);
        if(non_posture_safety != 0 || uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::JointDamping)){
            std::cout << "safe_control_mode:" <<std::dec<< uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::RLControlMode)) return StateName::kRLControl;
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::SuspendedStand)) return StateName::kSuspendedStand;
        return StateName::kZeroPos;
    }
};

};//namespace humanoid

#endif
