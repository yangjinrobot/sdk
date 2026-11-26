/**
 * @file suspended_standup_state.hpp
 * @brief 悬架起立状态控制器：机器人在悬挂/机架上从坐姿过渡到站立姿态
 * @author zhenglinqiang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_SUSPENDED_STANDUP_STATE_HPP_
#define HUMANOID_SUSPENDED_STANDUP_STATE_HPP_

#include "state_base.h"

namespace humanoid{
class SuspendedStandUpState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, goal_joint_pos_next_, kp_, kd_;
    MatXf joint_cmd_;
    float stand_duration_;
    std::string state_name_;

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
    SuspendedStandUpState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            stand_duration_ = cp_ptr_->stand_duration_;
            state_name_ = state_name;
        }
    ~SuspendedStandUpState(){}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::SuspendedStand);
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
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        goal_joint_pos_ = VecXf::Zero(cp_ptr_->dof_num_);
        goal_joint_pos_next_ = VecXf::Zero(cp_ptr_->dof_num_);

        Vec3f goal_xyz, goal_xyz_next;

        float ankley_pos,ankley_vel;
        float ankle_distance_x = 0.06f;
        float ankle_distance_x_next = 0.06f;
        goal_xyz << ankle_distance_x,cp_ptr_->leg_link_len_(0),cp_ptr_->pre_height_;
        goal_xyz_next << ankle_distance_x_next,cp_ptr_->leg_link_len_(0),cp_ptr_->stand_height_;

        //默认脚踝在髋正下方，脚掌要平行于地面的话，ankleY的活动范围会有点不够
        //first
        // ankley_pos = fabs(LegInverseKinematic(goal_xyz, cp_ptr_)(1)) - LegInverseKinematic(goal_xyz, cp_ptr_)(2);
        // LimitNumber(ankley_pos, cp_ptr_->leg_joint_lower_(4), cp_ptr_->leg_joint_upper_(4));
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+4) = -0.17;//ankley_pos;
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+10) = -0.17;//ankley_pos;
        //hipy
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_) = -0.17;//LegInverseKinematic(goal_xyz)(1);
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+6) = -0.17;//LegInverseKinematic(goal_xyz)(1);
        //hipx
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1) = 0.;//LegInverseKinematic(goal_xyz)(0);
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+7) = 0.;//-LegInverseKinematic(goal_xyz)(0);
        //knee
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3) = 0.34;//LegInverseKinematic(goal_xyz)(2);
        goal_joint_pos_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+9) = 0.34;//LegInverseKinematic(goal_xyz)(2);

        //second
        // ankley_pos = fabs(LegInverseKinematic(goal_xyz_next, cp_ptr_)(1)) - LegInverseKinematic(goal_xyz_next, cp_ptr_)(2);
        // LimitNumber(ankley_pos, cp_ptr_->leg_joint_lower_(4), cp_ptr_->leg_joint_upper_(4));
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+4) = -0.17;//ankley_pos;
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+10) = -0.17;//ankley_pos;
        //hipy
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_) = -0.17;//LegInverseKinematic(goal_xyz_next)(1);
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+6) = -0.17;//LegInverseKinematic(goal_xyz_next)(1);
        //hipx
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1) = 0.;//LegInverseKinematic(goal_xyz_next)(0);
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+7) = 0.;//-LegInverseKinematic(goal_xyz_next)(0);
        //knee
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3) = 0.34;//LegInverseKinematic(goal_xyz_next)(2);
        goal_joint_pos_next_(2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+9) = 0.34;//LegInverseKinematic(goal_xyz_next)(2);
        
        //shoulderx
        goal_joint_pos_(cp_ptr_->waist_dof_num_+1) = 0.13;
        goal_joint_pos_(cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1) = -0.13;
        goal_joint_pos_next_(cp_ptr_->waist_dof_num_+1) = 0.13;
        goal_joint_pos_next_(cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1) = -0.13;
        //elbow
        goal_joint_pos_(cp_ptr_->waist_dof_num_+3) = 1.2;
        goal_joint_pos_(cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3) = 1.2;
        goal_joint_pos_next_(cp_ptr_->waist_dof_num_+3) = 1.2;
        goal_joint_pos_next_(cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3) = 1.2;

        if (cp_ptr_->dof_num_ == 21) {
            // CR1 Standard suspended-stand initial posture uses 1.2 rad elbows.
            goal_joint_pos_ <<
                    0.0,
                    0.0,  0.15, 0.0, 1.2,
                    0.0, -0.15, 0.0, 1.2,
                   -0.2, 0.0,  0.0, 0.4, -0.2, 0.0,
                   -0.2, 0.0,  0.0, 0.4, -0.2, 0.0;
            goal_joint_pos_next_ = goal_joint_pos_;
        }
        
        if(run_time_ - time_stamp_record_ <= stand_duration_/2){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_/2);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_/2);
            }
        }else if(run_time_ - time_stamp_record_ <= stand_duration_){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(goal_joint_pos_(i), 0., goal_joint_pos_next_(i), 0, 
                                                run_time_ - time_stamp_record_ - stand_duration_/2, stand_duration_/2);
                planning_joint_vel(i) = GetCubicSplineVel(goal_joint_pos_(i), 0., goal_joint_pos_next_(i), 0, 
                                                run_time_ - time_stamp_record_ - stand_duration_/2, stand_duration_/2);
            }
        }else{
            for(int i=0;i<current_joint_pos_.rows();++i){planning_joint_pos(i) = goal_joint_pos_next_(i); planning_joint_vel.setZero();}
        }

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

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;  
        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(4) = planning_joint_tau;

        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if((uc_ptr_->GetUserCommand()->safe_control_mode &= ~(1 << kSafePostureNormal)) !=0 || uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::JointDamping)){
            std::cout << "safe_control_mode:" <<std::dec<< uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::AmpControl_NiRenWalk_Mode) &&
           !(uc_ptr_->GetUserCommand()->safe_control_mode & (1 << kSafePostureNormal)))
            return StateName::kAmpControlNiRenWalkMode;
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::AmpControl_GunShootWalk_Mode) && !(uc_ptr_->GetUserCommand()->safe_control_mode&(1 << kSafePostureNormal))) return StateName::kAmpControlGunShootWalkMode;
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::ZeroPos)) return StateName::kZeroPos;
        if(robot_name_ == RobotName::CR1STANDARD &&
           uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::RLControl_Stand_Mode) return StateName::kRLControlStandMode;
        return StateName::kSuspendedStand;
    }
};

};//namespace humanoid

#endif
