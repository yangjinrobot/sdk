/**
 * @file user_command_interface.h
 * @brief this file is used for robot's user command input
 * @author deeprobotics
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef USER_COMMAND_INTERFACE_H_
#define USER_COMMAND_INTERFACE_H_

#include <atomic>
#include "common_types.h"
#include "custom_types.h"
#include "motion_state_feedback.hpp"

using namespace types;

namespace interface{

class UserCommandInterface{
private:
    /* data */
public:
    UserCommandInterface(RobotName robot_name=CR1STANDARD){
        robot_name_ = robot_name;
        usr_cmd_ = new UserCommand();
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }
    virtual ~UserCommandInterface(){
        delete usr_cmd_;
    }

    /**
     * @brief start the thread to process user command
     */
    virtual void Start() = 0;

    /**
     * @brief stop the thread 
     */
    virtual void Stop() = 0;

    /**
     * @brief return your user command
     * @return UserCommand ptr
     */
    virtual UserCommand* GetUserCommand() = 0; 

    /**
     * @brief set the motion state feedback 
     * @param  msfb         motion state feedback
     */
    virtual void SetMotionStateFeedback(MotionStateFeedback* msfb){
        msfb_ = msfb;
    }

    /**
     * @brief 生产端：设置目标步态（事件型，仅消费一次）
     */
    void SetTargetGait(uint32_t gait) {
        usr_cmd_->target_gait = gait;
        gait_pending_.store(true, std::memory_order_release);
    }

    /**
     * @brief 消费端：获取步态事件，消费后自动清除
     * @param[out] gait 输出目标步态值
     * @return true=有新事件，false=无新事件
     */
    bool ConsumeGaitEvent(uint32_t &gait) {
        if (!gait_pending_.exchange(false, std::memory_order_acq_rel)) {
            return false;
        }
        gait = usr_cmd_->target_gait;
        return true;
    }

    /**
     * @brief 重置步态事件标志位（用于状态机清零场景）
     */
    void ResetGaitPending() {
        gait_pending_.store(false, std::memory_order_release);
    }

    /**
     * @brief 生产端：设置目标动作（事件型，仅消费一次）
     */
    void SetTargetAction(uint32_t action) {
        usr_cmd_->target_action = action;
        action_pending_.store(true, std::memory_order_release);
    }

    /**
     * @brief 消费端：获取动作事件，消费后自动清除
     * @param[out] action 输出目标动作值
     * @return true=有新事件，false=无新事件
     */
    bool ConsumeActionEvent(uint32_t &action) {
        if (!action_pending_.exchange(false, std::memory_order_acq_rel)) {
            return false;
        }
        action = usr_cmd_->target_action;
        return true;
    }

    /**
     * @brief 重置动作事件标志位
     */
    void ResetActionPending() {
        action_pending_.store(false, std::memory_order_release);
    }

    MotionStateFeedback *msfb_;
    RobotName robot_name_;
    UserCommand *usr_cmd_;
    std::atomic<bool> gait_pending_{false};
    std::atomic<bool> action_pending_{false};
};
};

#endif