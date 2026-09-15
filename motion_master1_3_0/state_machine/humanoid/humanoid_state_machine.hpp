/**
 * @file humanoid_state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_STATE_MACHINE_HPP_
#define HUMANOID_STATE_MACHINE_HPP_

#include "state_base.h"
#include "state_machine_base.h"
#include "humanoid/idle_state.hpp"
#include "humanoid/zeropos_state.hpp"
#include "humanoid/joint_damping_state.hpp"
#include "humanoid/rl_control_state.hpp"
#include "humanoid/restand_state.hpp"
#include "humanoid/suspended_standup_state.hpp"
#include "humanoid/stand_policy_state.hpp"
#include "humanoid/stand_shooting_state.hpp"
#include "humanoid/ysc_state.hpp"
#include "humanoid/mofang_state.hpp"
#include "humanoid/zero_calibration_state.hpp"

#include "skydroid_gamepad_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "dds_interface.hpp"
#include "keyboard_interface.hpp"

#ifdef USE_SIMULATION
    #include "simulation/pybullet_interface.hpp"
#else
    #include "hardware/cr1std_hardware_interface.hpp"
    #include "hardware/cr1pro_hardware_interface.hpp"
#endif
#include "data_streaming.hpp"
#include "safe_controller/safe_controller.hpp"

namespace humanoid{
class HumanoidStateMachine : public StateMachineBase{
private:
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> zeropos_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> suspended_standup_controller_;
    std::shared_ptr<StateBase> stand_policy_controller_;
    std::shared_ptr<StateBase> stand_shooting_controller_;
    std::shared_ptr<StateBase> ysc_controller_;
    std::shared_ptr<StateBase> mofang_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> restand_controller_;
    std::shared_ptr<StateBase> zero_calibration_controller_;

    StateName last_state_name_, current_state_name_, next_state_name_;

public:
    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;    

    HumanoidStateMachine(RobotName robot_name, RemoteCommandType rct):StateMachineBase(RobotType::HumanoidLeg),
    robot_name_(robot_name),
    remote_cmd_type_(rct) {
        // robot_name_ = robot_name;
        // remote_cmd_type_ = rct;
    }
    ~HumanoidStateMachine(){}

    void Start(){

        const std::string activation_key = "~/raisim/activation.raisim";
        std::string urdf_path = "";
        if(remote_cmd_type_ == RemoteCommandType::kRetroid){
            uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121, robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kSkydroid){
            uc_ptr_ = std::make_shared<SkydroidGamepadInterface>(12121, robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kDDS){
            uc_ptr_ = std::make_shared<DdsInterface>(robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kKeyboard){
            uc_ptr_ = std::make_shared<KeyboardInterface>(robot_name_);
        }
        else{
            std::cerr << "error user command interface! " << std::endl;
            exit(0);
        }
        uc_ptr_->SetMotionStateFeedback(&StateBase::msfb_);

        if(robot_name_ == RobotName::CR1STANDARD){
            #ifdef USE_SIMULATION
                ri_ptr_ = std::make_shared<PybulletInterface>("CR1STANDARD");
            #else
                ri_ptr_ = std::make_shared<CR1StdHardwareInterface>("CR1STANDARD");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);  
        }else if(robot_name_ == RobotName::CR1PRO){
            #ifdef USE_SIMULATION
                ri_ptr_ = std::make_shared<PybulletInterface>("CR1PRO");
            #else
                ri_ptr_ = std::make_shared<CR1ProHardwareInterface>("CR1PRO");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);  
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;

        ds_ptr_ = std::make_shared<DataStreaming>();
        ds_ptr_->SetRobotDataSource(ri_ptr_);
        ds_ptr_->SetUserCommandDataSource(uc_ptr_);
        data_ptr->ds_ptr = ds_ptr_;

        sc_ptr_ = std::make_shared<SafeController>(HumanoidStandard, "");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);
        sc_ptr_->SetControlParamInfaceSource(cp_ptr_);
        data_ptr->sc_ptr = sc_ptr_;

        // 初始化关节保护检查器（复用 SafeController 的 fault_client）
        #ifndef USE_SIMULATION
        {
            auto* ecan_ri = dynamic_cast<EcanHardwareInterface*>(ri_ptr_.get());
            if (ecan_ri) {
                ecan_ri->initJointProtection(cp_ptr_.get(), sc_ptr_->GetFaultClient());
                ecan_ri->SetSafeControllerSource(sc_ptr_);
            }
        }
        #endif

        du_ptr_ = std::make_shared<MotionDataUpload>();
        du_ptr_->SetMotionStateFeedbackSource(&StateBase::msfb_);
        du_ptr_->SetRobotInterfaceSource(ri_ptr_);
        du_ptr_->SetUserCommandInfaceSource(uc_ptr_);
        #ifndef USE_SIMULATION
            dp_ptr_ = std::make_shared<MotionDataPublish>();
            dp_ptr_->SetRobotInterfaceSource(ri_ptr_);
            dp_ptr_->SetMotionStateFeedbackSource(&StateBase::msfb_);
            dp_ptr_->SetSafeControllerSource(sc_ptr_);
        #endif

        idle_controller_ = std::make_shared<IdleState>(robot_name_, "idle_state", data_ptr);
        zeropos_controller_ = std::make_shared<ZeroPosState>(robot_name_, "zeropos_state", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name_, "joint_damping", data_ptr);
        suspended_standup_controller_ = std::make_shared<SuspendedStandUpState>(robot_name_, "suspended_standup", data_ptr);
        if (robot_name_ == RobotName::CR1STANDARD) {
            stand_policy_controller_ = std::make_shared<StandPolicyState>(robot_name_, "stand_policy", data_ptr);
            stand_shooting_controller_ = std::make_shared<StandShootingState>(robot_name_, "stand_shooting", data_ptr);
            ysc_controller_ = std::make_shared<YscState>(robot_name_, "ysc", data_ptr);
            mofang_controller_ = std::make_shared<MofangState>(robot_name_, "mofang", data_ptr);
        }
        rl_controller_ = std::make_shared<RLControlState>(robot_name_, "rl_control", data_ptr);
        if (robot_name_ == RobotName::CR1STANDARD) {
            restand_controller_ = std::make_shared<ReStandState>(robot_name_, "restand", data_ptr);
        }
        zero_calibration_controller_ = std::make_shared<ZeroCalibrationState>(robot_name_, "zero_calibration_state", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;

        std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety 

        ri_ptr_->Start();
        std::cout << "1" << std::endl;
        uc_ptr_->Start();
        std::cout << "2" << std::endl;
        ds_ptr_->Start();
        std::cout << "3" << std::endl;
        du_ptr_->Start();
        std::cout << "4" << std::endl;
        #ifndef USE_SIMULATION
            dp_ptr_->Start();
            std::cout << "5" << std::endl;
        #endif

        #ifndef USE_RAISIM
            sc_ptr_->Start();
        #endif

        // 脖子接管配置加载（代码下沉至 utils::NeckTakeover）
        ri_ptr_->LoadNeckTakeoverConfig();
        
        current_controller_->OnEnter();  
        std::cout<<"start finish"<<std::endl;
    }

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kSuspendedStand:{
                return suspended_standup_controller_;
            }
            case StateName::kStandPolicy:{
                return stand_policy_controller_ ? stand_policy_controller_ : joint_damping_controller_;
            }
            case StateName::kStandShooting:{
                return stand_shooting_controller_ ? stand_shooting_controller_ : joint_damping_controller_;
            }
            case StateName::kYsc:{
                return ysc_controller_ ? ysc_controller_ : joint_damping_controller_;
            }
            case StateName::kMofang:{
                return mofang_controller_ ? mofang_controller_ : joint_damping_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kReStand:{
                return restand_controller_ ? restand_controller_ : joint_damping_controller_;
            }
            case StateName::kZeroPos:{
                return zeropos_controller_;
            }
            case StateName::kZeroCalibration:{
                return zero_calibration_controller_;
            }
            
            default:{
                std::cerr << "error state name" << std::endl;
                return joint_damping_controller_;
            }
        }
        return nullptr;
    }


    void Stop(){
        uc_ptr_->Stop();
        ri_ptr_->Stop();
        ds_ptr_->Stop();
    }

};
};//namespace humanoid
#endif
