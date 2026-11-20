/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author zhenglinqiang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_IDLE_STATE_HPP_
#define HUMANOID_IDLE_STATE_HPP_


#include "state_base.h"
#include "drdds/core/drdds_core.h"

namespace humanoid{
class IdleState : public StateBase{
private:
    bool joint_normal_flag_ = false, imu_normal_flag_ = false;
    bool first_enter_flag_ = true;
    int cnt=0;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    Vec3f rpy_body_, acc_body_, omg_body_;
    Vec3f rpy_leg_, acc_leg_, omg_leg_;
    float enter_state_time_ = 10000.;
    float time_record_, run_time_;
    std::unique_ptr<ChannelUInt64> mark_zero_sub_;
    uint64_t MarkZeroMask = 0;

    void GetProprioceptiveData(){
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
        rpy_leg_ = ri_ptr_->GetImuRpy();
        acc_leg_ = ri_ptr_->GetImuAcc();
        omg_leg_ = ri_ptr_->GetImuOmega();
        rpy_body_ = ri_ptr_->GetImuBodyRpy();
        acc_body_ = ri_ptr_->GetImuBodyAcc();
        omg_body_ = ri_ptr_->GetImuBodyOmega();
    }

    bool JointDataNormalCheck(){
        VecXf joint_pos_lower(cp_ptr_->dof_num_), joint_pos_upper(cp_ptr_->dof_num_);
        if(cp_ptr_->dof_num_ == 31 || cp_ptr_->dof_num_ == 30) {
            joint_pos_lower << cp_ptr_->waist_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_, cp_ptr_->neck_joint_lower_;
            joint_pos_upper << cp_ptr_->waist_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_, cp_ptr_->neck_joint_upper_;
            joint_pos_lower(11) = -cp_ptr_->arm_joint_upper_(1);
            joint_pos_upper(11) = -cp_ptr_->arm_joint_lower_(1);
            joint_pos_lower(12) = -cp_ptr_->arm_joint_upper_(2);
            joint_pos_upper(12) = -cp_ptr_->arm_joint_lower_(2);
            joint_pos_lower(24) = -cp_ptr_->leg_joint_upper_(1);
            joint_pos_upper(24) = -cp_ptr_->leg_joint_lower_(1);
            joint_pos_lower(25) = -cp_ptr_->leg_joint_upper_(2);
            joint_pos_upper(25) = -cp_ptr_->leg_joint_lower_(2);
        }else if(cp_ptr_->dof_num_ == 21) {
            joint_pos_lower << cp_ptr_->waist_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_;
            joint_pos_upper << cp_ptr_->waist_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_;
            joint_pos_lower(6) = -cp_ptr_->arm_joint_upper_(1);
            joint_pos_upper(6) = -cp_ptr_->arm_joint_lower_(1);
            joint_pos_lower(7) = -cp_ptr_->arm_joint_upper_(2);
            joint_pos_upper(7) = -cp_ptr_->arm_joint_lower_(2);
            joint_pos_lower(16) = -cp_ptr_->leg_joint_upper_(1);
            joint_pos_upper(16) = -cp_ptr_->leg_joint_lower_(1);
            joint_pos_lower(17) = -cp_ptr_->leg_joint_upper_(2);
            joint_pos_upper(17) = -cp_ptr_->leg_joint_lower_(2);
        }else{
            joint_pos_lower << cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_;
            joint_pos_upper << cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_;
            joint_pos_lower(7) = -cp_ptr_->leg_joint_upper_(1);
            joint_pos_upper(7) = -cp_ptr_->leg_joint_lower_(1);
            joint_pos_lower(8) = -cp_ptr_->leg_joint_upper_(2);
            joint_pos_upper(8) = -cp_ptr_->leg_joint_lower_(2);
        }

        // std::cout<<"joint_pos_lower:"<<joint_pos_lower.transpose()<<std::endl;
        // std::cout<<"joint_pos_upper:"<<joint_pos_upper.transpose()<<std::endl;
        //---正负待根据实际安装确认---//
        for(int i=0;i<cp_ptr_->dof_num_;++i){
            if(std::isnan(joint_pos_lower(i)) || joint_pos_(i) > joint_pos_upper(i)+0.1 || joint_pos_(i) < joint_pos_lower(i)-0.1) {
                // std::cout << "joint pos " << i << " : " << joint_pos_(i) << " | " 
                //                                         << joint_pos_lower(i) << " " << joint_pos_upper(i) << std::endl;
                std::cout << "joint "<<i<< "is out of range! "<<std::endl<<"upper: "<<joint_pos_upper(i)+0.1<<",lower: " << joint_pos_lower(i)-0.1<<",now:"<<joint_pos_(i)<<std::endl;
                return false;
            }
            if(std::isnan(joint_vel_(i)) || (joint_vel_(i)) > cp_ptr_->joint_vel_limit_(i) + 0.1) {
                // std::cout << "joint vel " << i << " : " << joint_vel_(i) << " | " << cp_ptr_->joint_vel_limit_(i%3) << std::endl;
                return false;
            }
        }
        return true;
    }
    bool ImuDataNormalCheck(){
        for(int i=0;i<3;++i){
            if(std::isnan(rpy_leg_(i)) || fabs(rpy_leg_(i)) > M_PI){
                // std::cout << "rpy_ " << i << " : " << rpy_(i) << std::endl;
                return false;
            } 
            if(std::isnan(omg_leg_(i)) || fabs(omg_leg_(i)) > M_PI){
                // std::cout << "omg_ " << i << " : " << omg_(i) << std::endl;
                return false;
            }
        }
        if(acc_leg_.norm() < 0.1*gravity || acc_leg_.norm() > 3.0*gravity){
            // std::cout << "acc " << " : " << acc.transpose() << std::endl;
            return false;
        }
        return true;
    }

    void DisplayProprioceptiveInfo(){
        std::cout << "Joint Data: \n";
        // std::cout << "pos: " << joint_pos_.transpose()*57.2957795 << std::endl;
        std::cout << "Waist pos: " << joint_pos_.transpose().segment(0,cp_ptr_->waist_dof_num_)*57.2957795 << std::endl;

        std::cout << "L arm pos: " << joint_pos_.transpose().segment(cp_ptr_->waist_dof_num_,cp_ptr_->arm_dof_num_)*57.2957795 << std::endl;
        std::cout << "R arm pos: " << joint_pos_.transpose().segment(cp_ptr_->waist_dof_num_+cp_ptr_->arm_dof_num_,cp_ptr_->arm_dof_num_)*57.2957795 << std::endl;

        std::cout << "L leg pos: " << joint_pos_.transpose().segment(cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_,cp_ptr_->leg_dof_num_)*57.2957795 << std::endl;
        std::cout << "R leg pos: " << joint_pos_.transpose().segment(cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+cp_ptr_->leg_dof_num_,cp_ptr_->leg_dof_num_)*57.2957795 << std::endl;
        std::cout << "neck pos: " << joint_pos_.transpose().segment(cp_ptr_->waist_dof_num_+2*cp_ptr_->arm_dof_num_+2*cp_ptr_->leg_dof_num_,cp_ptr_->neck_dof_num_)*57.2957795 << std::endl;

        // std::cout << "vel: " << joint_vel_.transpose() << std::endl;
        // std::cout << "tau: " << joint_tau_.transpose() << std::endl;
        std::cout << "Imu for Leg Data: \n";
        std::cout << "rpy: " << rpy_leg_.transpose() << std::endl;
        std::cout << "acc: " << acc_leg_.transpose() << std::endl;
        std::cout << "omg: " << omg_leg_.transpose() << std::endl;
        std::cout << "Imu in Body Data: \n";
        std::cout << "rpy: " << rpy_body_.transpose() << std::endl;
        std::cout << "acc: " << acc_body_.transpose() << std::endl;
        std::cout << "omg: " << omg_body_.transpose() << std::endl;
    }

    void DisplayAxisValue(){
        // UserCommand* uc_cmd = uc_ptr_->GetUserCommand();
        auto cmd = uc_ptr_->GetUserCommand();
        std::cout << "User Command Input: \n";
        std::cout << "axis value:  " << cmd->forward_vel_scale << " " 
                                     << cmd->side_vel_scale << " "
                                     << cmd->turnning_vel_scale << std::endl;
        std::cout << "target mode: " << int(cmd->target_mode) << std::endl;
    }

    void HandlerMarkZero(const std_msgs::msg::UInt64 *data)
    {
        MarkZeroMask = data->data();
        std::cout<< " HandlerMarkZero" <<MarkZeroMask << std::endl;
    }

public:
    IdleState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            mark_zero_sub_ = std::make_unique<ChannelUInt64>(std::bind(&IdleState::HandlerMarkZero, this, std::placeholders::_1),"/JOINTS_MARK_ZERO", 0);
        }
    ~IdleState(){}

    virtual void OnEnter() {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::WaitingForStand);
        //临时措施，进入空闲，步态、motion、action清零
        uc_ptr_->GetUserCommand()->target_mode = 0;
        uc_ptr_->ResetGaitPending();
        uc_ptr_->GetUserCommand()->target_action = 0;
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        enter_state_time_ = ri_ptr_->GetInterfaceTimeStamp();
        MarkZeroMask = 0;
    };
    virtual void OnExit() {
        first_enter_flag_ = false;
    }
    virtual void Run() {
        GetProprioceptiveData();

        // std::cout << ri_ptr_->GetInterfaceTimeStamp() << " | " << enter_state_time_ << " " << first_enter_flag_ << std::endl;
        cnt++;

        // if(first_enter_flag_ && ri_ptr_->GetInterfaceTimeStamp() - enter_state_time_ > 2.){//to confirm right state input
            // DisplayProprioceptiveInfo();
            // DisplayAxisValue();
        // }
        MatXf cmd = MatXf::Zero(cp_ptr_->dof_num_, 5);
        // cmd(6,4)=1.2;
        if(cnt%2000==0){
            joint_normal_flag_ = JointDataNormalCheck();
            imu_normal_flag_ = ImuDataNormalCheck();
            DisplayProprioceptiveInfo();
        }
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        // std::cout<<"cmd:"<<cmd.transpose()<<std::endl;
        if(MarkZeroMask != 0)
        {
            ri_ptr_->SetJointZero(MarkZeroMask);
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
            ri_ptr_->InitPositionOffset(MarkZeroMask);
            ri_ptr_->WrapPositionOffset();
            MarkZeroMask = 0;
        }
        ri_ptr_->SetJointCommand(cmd);
    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(uc_ptr_->GetUserCommand()->safe_control_mode&(1<<kSafeDriverStatus)){
            if(run_time_ - time_record_ >= 3.){
                ri_ptr_->ResetJointError();
                time_record_ = run_time_;
            }
        }
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::SuspendedStand)) return StateName::kSuspendedStand;
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::ZeroPos)) return StateName::kZeroPos;
        if(uc_ptr_->GetUserCommand()->target_mode == (RobotMotionState::ZeroCalibration)) return StateName::kZeroCalibration;
        const auto& user = *uc_ptr_->GetUserCommand();
        if (robot_name_ == RobotName::CR1STANDARD && user.safe_control_mode == 0 &&
            user.target_mode == RobotMotionState::SquatPreparation) return StateName::kSquatPreparation;
        return StateName::kIdle;
    }
};

};//namespace humanoid


#endif
