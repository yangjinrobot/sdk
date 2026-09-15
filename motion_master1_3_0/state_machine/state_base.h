/**
 * @file state_base.h
 * @brief state base class for robot state control
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef STATE_BASE_H_
#define STATE_BASE_H_

#include "common_types.h"
#include "custom_types.h"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "data_streaming.hpp"
#include "motion_state_feedback.hpp"
#include "basic_function.hpp"
#include "parameters/control_parameters.h"
#include  <mutex>

using namespace types;
using namespace interface;

class SafeController;

struct ControllerData{
    std::shared_ptr<RobotInterface> ri_ptr;
    std::shared_ptr<UserCommandInterface> uc_ptr;
    std::shared_ptr<ControlParameters> cp_ptr;
    std::shared_ptr<DataStreaming> ds_ptr;
    std::shared_ptr<SafeController> sc_ptr;
};


class StateBase{
private:

public:
    StateBase(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):
        robot_name_(robot_name),
        state_name_(state_name),
        data_ptr_(data_ptr){
            ri_ptr_ = data_ptr_->ri_ptr;
            uc_ptr_ = data_ptr_->uc_ptr;
            cp_ptr_ = data_ptr_->cp_ptr;
            ds_ptr_ = data_ptr_->ds_ptr;
            sc_ptr_ = data_ptr_->sc_ptr;
            std::memset(&msfb_, 0, sizeof(msfb_));
        }

    ~StateBase(){}

    /**
     * @brief execute function when enter state
     */
    virtual void OnEnter() = 0;

    /**
     * @brief execute function when exit state
     */
    virtual void OnExit() = 0;

    /**
     * @brief run this function when in this state
     */
    virtual void Run() = 0;

    /**
     * @brief lose control judgement in every time stamp
     * @return true     robot is losing control and going to switch state
     * @return false    robot is in normal state
     */
    virtual bool LoseControlJudge() = 0;

    /**
     * @brief get the next state name if you want to switch state
     * @return StateName    next state name
     */
    virtual StateName GetNextStateName() = 0;

    /**
     * @brief 请求当前状态启动平滑退出（默认空实现）
     *        子类可 override 在内部启动过渡逻辑；未启用该机制的状态保持为空。
     * @param next 过渡完成后要切到的目标状态
     */
    virtual void RequestSmoothExit(StateName /*next*/) {}

    /**
     * @brief 查询当前状态是否正处于平滑退出过渡中
     */
    virtual bool IsSmoothExiting() const { return false; }

    /**
     * @brief state name for print 
     */
    std::string state_name_;

    /**
     * @brief robot_name for print
     */
    RobotName robot_name_;

    std::shared_ptr<ControllerData> data_ptr_;      //control data pointer 
    std::shared_ptr<RobotInterface> ri_ptr_;        //robot interface pointer to control your robot
    std::shared_ptr<UserCommandInterface> uc_ptr_;  //control your robot by design user command
    std::shared_ptr<ControlParameters> cp_ptr_;     //control parameters
    std::shared_ptr<DataStreaming> ds_ptr_;         //for data record
    std::shared_ptr<SafeController> sc_ptr_;        //safe controller pointer
    UserCommand user_command;                       //当前计算周期内的user_command
    std::mutex user_command_policy_mutex_;    
    static MotionStateFeedback msfb_;               //robot current motion state
};



#endif