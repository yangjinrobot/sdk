/**
 * @file zero_calibration_state.hpp
 * @brief Move to the limit position and perform zero calibration
 * @author zhenglinqiang
 * @version 1.0
 * @date 2024-11-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_ZERO_CALIBRATION_STATE_HPP_
#define HUMANOID_ZERO_CALIBRATION_STATE_HPP_

#include "state_base.hpp"
#include "humanoid/standard_motion_state.hpp"

namespace humanoid{
class ZeroCalibrationState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;
    VecXf kp_, kd_;
    MatXf joint_cmd_;
    MatXf goal_joint_vel_;
    VecXf calibration_time_;
    int calibration_state_ = 0;
    int num_state_ = 3;
    bool exit_flag = false;
    bool sethome_flag = false;

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
    ZeroCalibrationState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){}
    ~ZeroCalibrationState(){}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        calibration_state_ = 0;
        exit_flag = false;
        sethome_flag = false;
        motion_state_store_->SetState(static_cast<RobotMotionState>(StandardMotionState::ZeroCalibration));
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);  // 与原始维度一致
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();   
        VecXf planning_joint_vel = VecXf::Zero(cp_ptr_->dof_num_);     
        kd_ = VecXf::Zero(cp_ptr_->dof_num_);
        joint_cmd_.setZero();
        goal_joint_vel_ = MatXf::Zero(num_state_, cp_ptr_->dof_num_);
        calibration_time_ = VecXf::Zero(num_state_);
        calibration_time_ << 8., 1., 1.;
        float sethome_cost_time = 1.;
        float set_vel = 0.6;
        //第一阶段
        goal_joint_vel_(0,0) = -set_vel;//waistz

        goal_joint_vel_(0,cp_ptr_->waist_dof_num_) = set_vel;//shoulder y
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+cp_ptr_->arm_dof_num_) = set_vel;  

        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+3) = -set_vel;//elbow
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+cp_ptr_->arm_dof_num_+3) = -set_vel; 

        // goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+1) = -set_vel; //left hipx

        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+2) = -set_vel; //hipz
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+2) = set_vel;

        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+3) = -set_vel; //knee
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+3) = -set_vel;

        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+4) = set_vel; //ankley
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+4) = set_vel;
        
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+5) = set_vel; //anklex
        goal_joint_vel_(0,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+5) = -set_vel;
        //第二阶段
        // goal_joint_vel_(1,cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+1) = set_vel;//right hipx    

        if(run_time_ - time_stamp_record_ <= calibration_time_[calibration_state_]){
            planning_joint_vel = goal_joint_vel_.row(calibration_state_);
            // std::cout<<planning_joint_vel<<std::endl;
        }else if(run_time_ - time_stamp_record_ <= calibration_time_[calibration_state_]+sethome_cost_time){
            planning_joint_vel = goal_joint_vel_.row(calibration_state_);
            if(!sethome_flag){
                sethome_flag = true;
                uint64_t jointMask = 0;
                jointMask |= 1 << 0;

                jointMask |= 1 << cp_ptr_->waist_dof_num_;
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+cp_ptr_->arm_dof_num_);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+3);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+cp_ptr_->arm_dof_num_+3);

                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+2);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+2);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+3);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+3);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+4);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+4);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+5);
                jointMask |= 1 << (cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_+5);
                ri_ptr_->SetJointZero(jointMask);//TODO只能执行一次
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
                ri_ptr_->InitPositionOffset(jointMask);
                ri_ptr_->WrapPositionOffset();
            }
        }else{
            if(calibration_state_ < num_state_-1){
                RecordJointData();
                calibration_state_++;
                std::cout<<"go to next calibration state_:"<<calibration_state_<<std::endl;
            }else{
                planning_joint_vel.setZero();
                std::cout<<"exit calibration"<<std::endl;
                exit_flag = true;
            }
        }

        if(cp_ptr_->dof_num_ == 31 || cp_ptr_->dof_num_ == 30) {
            kd_<<2*cp_ptr_->waist_kd_, 5*cp_ptr_->arm_kd_, 5*cp_ptr_->arm_kd_, 3*cp_ptr_->leg_kd_, 3*cp_ptr_->leg_kd_, cp_ptr_->neck_kd_;
        }else if(cp_ptr_->dof_num_ == 21) {
            kd_<<2*cp_ptr_->waist_kd_,5*cp_ptr_->arm_kd_,5*cp_ptr_->arm_kd_, 3*cp_ptr_->leg_kd_,3*cp_ptr_->leg_kd_;
        }else{
            kd_<<cp_ptr_->leg_kd_,cp_ptr_->leg_kd_;            
        }

        joint_cmd_.col(2) = kd_;  
        joint_cmd_.col(3) = planning_joint_vel;

        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand()->target_mode == (StandardMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(/*uc_ptr_->GetUserCommand()->safe_control_mode!=0 ||*/ uc_ptr_->GetUserCommand()->target_mode == (StandardMotionState::JointDamping)){
            std::cout << "safe_control_mode:" <<std::dec<< uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }
        if(exit_flag) {uc_ptr_->GetUserCommand()->target_mode = StandardMotionState::WaitingForStand; return StateName::kIdle;}
        return kZeroCalibration;
    }
};

};//namespace humanoid

#endif