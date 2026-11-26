#ifndef SKYDROID_GAMEPAD_INTERFACE_HPP_
#define SKYDROID_GAMEPAD_INTERFACE_HPP_

#include "user_command_interface.h"
#include "skydroid_gamepad.h"
#include "custom_types.h"
#include "gamepad_motion_command.hpp"

using namespace interface;
using namespace types;

class SkydroidGamepadInterface : public UserCommandInterface{
private:
    std::shared_ptr<SkydroidGamepad> gamepad_ptr_;
    SkydroidKeys sd_keys_record_, sd_keys_;
    // UserCommand* usr_cmd_;
    bool IsKeysEqual(const SkydroidKeys& a, const SkydroidKeys& b){
        if(a.keys_value != b.keys_value) return false;
        for(int i=0;i<kAxisChannlSize;++i){
            if(a.axis_values[i] != b.axis_values[i]) return false;
        }
        for(int i=0;i<kSwitchKeysSize;++i){
            if(a.switch_keys[i] != b.switch_keys[i]) return false;
        }
        return true;
    }


    bool transform_cmd_flag_ = true;
    std::thread transform_thread_;
    bool first_flag_ = true;
    
    
public:
    SkydroidGamepadInterface(int port, RobotName robot_name=RobotName::CR1STANDARD):UserCommandInterface(robot_name){
        std::cout << "Using Skydroid Gamepad Command Interface" << std::endl;
        gamepad_ptr_ = std::make_shared<SkydroidGamepad>(port);
    }
    ~SkydroidGamepadInterface(){}

    virtual void Start(){
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
        gamepad_ptr_->StartDataThread();
        transform_thread_ = std::thread(std::bind(&SkydroidGamepadInterface::TransformSkydroidToUserCommand, this));
        //
    }
    virtual void Stop(){
        gamepad_ptr_->StopDataThread();
        transform_cmd_flag_ = false;
        transform_thread_.join();
    }
    virtual UserCommand* GetUserCommand(){
        return usr_cmd_;
    }

    

    void TransformSkydroidToUserCommand();
    
    void PrintGamepadData(SkydroidKeys *data){
        std::cout << "\nAxis value: \t";
        for(auto i : data->axis_values) std::cout << i << ",\t";
        std::cout << "\nSW keys: \t";
        std::cout << bool(data->sw1) << ",\t" << bool(data->sw2) << ",\t" << bool(data->sw3) << ",\t" << bool(data->sw4) << ",\t";
        std::cout << "\nDownside keys: \t";
        std::cout << bool(data->A) << ",\t" << bool(data->B) << ",\t" << bool(data->C) << ",\t" << bool(data->D) << ",\t";
        std::cout << bool(data->E) << ",\t" << bool(data->F) << ",\t" << bool(data->right) << ",\t";
        std::cout << "\n";  
    }


};

void SkydroidGamepadInterface::TransformSkydroidToUserCommand(){
    while (transform_cmd_flag_) {
        sd_keys_ = gamepad_ptr_->GetKeys();
        // PrintGamepadData(&sd_keys_);
        if(first_flag_) {
            sd_keys_record_ = sd_keys_;
            first_flag_ = false;
            continue;
        }
        // std::cout << "axis: " << int(sd_keys_.A) << std::endl;
        usr_cmd_->time_stamp = sd_keys_.time_stamp;
        if (msfb_->GetCurrentState() != RobotMotionState::AmpControl_NiRenWalk_Mode &&
            msfb_->GetCurrentState() != RobotMotionState::AmpControl_GunShootWalk_Mode &&
            msfb_->GetCurrentState() != RobotMotionState::AmpControl_Walk2Run_Mode) {
            usr_cmd_->forward_vel_scale = usr_cmd_->side_vel_scale = usr_cmd_->turnning_vel_scale = 0.0f;
        }
        // usr_cmd_->forward_vel_scale = sd_keys_.left_axis_y;
        // usr_cmd_->side_vel_scale = -sd_keys_.left_axis_x;

        if (!IsKeysEqual(sd_keys_, sd_keys_record_)) {
            switch (msfb_->GetCurrentState()){

            case RobotMotionState::AmpControl_Walk2Run_Mode:
            case RobotMotionState::AmpControl_NiRenWalk_Mode:
            case RobotMotionState::AmpControl_GunShootWalk_Mode:
                usr_cmd_->forward_vel_scale = sd_keys_.left_axis_y;
                usr_cmd_->side_vel_scale = -sd_keys_.left_axis_x;
                usr_cmd_->turnning_vel_scale = -sd_keys_.right_axis_x;
                if (msfb_->GetCurrentState() == RobotMotionState::AmpControl_NiRenWalk_Mode ||
                    msfb_->GetCurrentState() == RobotMotionState::AmpControl_GunShootWalk_Mode)
                    SetTargetGait(HumanWALK);
                break;

            default:
                break;
            }
            if(sd_keys_.A != sd_keys_record_.A) {
                usr_cmd_->target_mode = RobotMotionState::ZeroPos;
            }
            if(sd_keys_.B != sd_keys_record_.B) {
                usr_cmd_->target_mode = RobotMotionState::SuspendedStand;
            }
            if(sd_keys_.right && !sd_keys_record_.right &&
               msfb_->GetCurrentState() != RobotMotionState::MimicControl_ChairDown_Mode){
                usr_cmd_->target_mode = RobotMotionState::AmpControl_NiRenWalk_Mode;
            }
            if(sd_keys_.sw1 != 0 && sd_keys_.sw4 != 0){
                usr_cmd_->target_mode = RobotMotionState::JointDamping;
            }
            // C sits, D rises, right selects AmpControl_NiRenWalk_Mode,
            // E selects AmpControl_Walk2Run_Mode.
            // Legacy running key edges are ignored.
            usr_cmd_->target_mode = GamepadMotionTarget(robot_name_, msfb_->GetCurrentState(), usr_cmd_->target_mode,
                sd_keys_.C && !sd_keys_record_.C, sd_keys_.D && !sd_keys_record_.D,
                sd_keys_.right && !sd_keys_record_.right, sd_keys_.E && !sd_keys_record_.E,
                sd_keys_.sw1 != 0 && sd_keys_.sw4 != 0);
            sd_keys_record_ = sd_keys_;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
}

#endif
