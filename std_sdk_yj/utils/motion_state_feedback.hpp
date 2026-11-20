/**
 * @file motion_state_feedback.hpp
 * @brief motion state feedback 
 * @author deeprobotics
 * @version 1.0
 * @date 2024-08-27
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef MOTION_STATE_FEEDBACK_HPP
#define MOTION_STATE_FEEDBACK_HPP
#include <atomic>

#include "common_types.h"

class MotionStateFeedback
{
private:
    uint32_t current_state_ = 0;
    uint32_t current_gait_ = 0;
    uint32_t current_action_ = 0;
    uint32_t current_pose_ = 0;
    uint32_t last_state_ = 0;
    uint32_t last_gait_ = 0;
    uint32_t last_action_ = 0;
    uint32_t error_code_ = 0;
    std::atomic<uint32_t> handset_status_{0};

    Vec3f current_vel_ = Vec3f::Zero();
    Vec3f cmd_vel_ = Vec3f::Zero();
    Vec3f max_vel_ = Vec3f::Zero();

    union{
        struct{
            uint32_t joint_pos_limit : 1;
            uint32_t posture_limit : 1;
            uint32_t reserved : 30;
        }error_code_bit_;
        uint32_t old_error_code_ = 0;
    };
public:
    static constexpr uint32_t kHandsetReadyFlag = 0x40000000u;

    MotionStateFeedback(/* args */){
        std::cout << "Motion State Feedback" << std::endl;
    }
    ~MotionStateFeedback(){

    }

    void UpdateCurrentError(uint32_t error_code){
        error_code_ = error_code;
    }

    void UpdateCurrentState(int state){
        if(uint32_t(state) != current_state_){
            last_state_ = current_state_;
            current_state_ = uint32_t(state);
            handset_status_.store(current_state_);
        }
    }

    void SetHandsetReady(bool ready) {
        if (ready) handset_status_.fetch_or(kHandsetReadyFlag);
        else handset_status_.fetch_and(~kHandsetReadyFlag);
    }

    uint32_t GetHandsetStatus() const { return handset_status_.load(); }

    void UpdateCurrentGait(uint32_t gait){
        if(gait != current_gait_){
            last_gait_ = current_gait_;
            current_gait_ = gait;
        }
    }
    void UpdateCurrentAction(uint32_t action){
        if(action != current_action_){
            last_action_ = current_action_;
            current_action_ = action;
        }
    }
    void UpdateCurrentPose(uint32_t pose){
        current_pose_ = pose;
    }

    void UpdateCurrentVel(Vec3f vel){
        current_vel_ = vel;
    }
    
    void ClearMotionError(){error_code_ = 0;}
    void SetJointPosLimitError(){
        error_code_bit_.joint_pos_limit = 1;
    }
    void SetPostureLimitError(){
        error_code_bit_.posture_limit = 1;
    }

    inline uint32_t GetCurrentState(){return current_state_;}
    inline uint32_t GetCurrentAction(){return current_action_;}
    inline uint32_t GetCurrentPose(){return current_pose_;}
    inline uint32_t GetCurrentGait(){return current_gait_;}
    inline uint32_t GetLastState(){return last_state_;}
    inline uint32_t GetLastGait(){return last_gait_;}

    
    inline float GetCurrentVelX(){return current_vel_[0];}
    inline float GetCurrentVelY(){return current_vel_[1];}
};

#endif
