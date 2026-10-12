/**
 * @file state_machine_base.h
 * @brief state machine base class for different robot state machine control
 * @author deeprobotics
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef STATE_MACHINE_BASE_H_
#define STATE_MACHINE_BASE_H_

#include <bitset>
#include "state_base.h"
#include "safe_controller/safe_controller.hpp"
#include "motion_data_upload.hpp" 
#include "dr_logger.hpp"
#ifndef USE_SIMULATION
    #include "motion_data_publish.hpp"
#endif
// #include "log4cplus_dr/dr_logger.hpp"

class StateMachineBase
{
private:
    /* data */
    DrLogger logger_temp, logger_pos, logger_vel, logger_torq, logger_imu, logger_bat, logger_stat, logger_error;

public:
    StateMachineBase(const RobotType& robot_type):robot_type_(robot_type),
        logger_temp("Temp"), logger_pos("Pos"), logger_vel("Vel"), logger_error("Error"),
        logger_torq("Torq"), logger_imu("Imu"), logger_bat("Bat"), logger_stat("Stat")
    {
    }
    virtual ~StateMachineBase(){}

    virtual void Start() = 0;
    void GetNextStateName()
    {
        if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
        else next_state_name_ = current_controller_ -> GetNextStateName();                    
    }
    virtual void Run() {
        int cnt = 0;
        set_timer.time_init(floor(_dt * 1e3 + 1e-8));
        while(true){

            // std::cout<<ri_ptr_->GetInterfaceTimeStamp()<<std::endl;
            if(!set_timer.time_interrupt()){
                cnt++;
                ri_ptr_->RefreshRobotData();
                time_record_ = ri_ptr_->GetInterfaceTimeStamp();
                // 推送脖子接管指令给硬件接口（开关未启用时内部 short-circuit）
                if (uc_ptr_) {
                    auto* uc = uc_ptr_->GetUserCommand();
                    if (uc) {
                        ri_ptr_->SetNeckTakeoverCmd(uc->neck_yaw_cmd, uc->neck_pitch_cmd);
                    }
                }
                current_controller_->Run();
                
                GetNextStateName();

                // std::cout << current_controller_ -> state_name_ << std::endl;
                // mdr = uint8_t(next_state_name_);
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    logger_stat.info("%s  ------------> %s",current_controller_ -> state_name_.c_str(), GetStateControllerPtr(next_state_name_)->state_name_.c_str());
                    current_controller_ = GetStateControllerPtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                ++run_cnt_;
                ds_ptr_->InsertScopeData(0, StateBase::msfb_.GetCurrentState());
                ds_ptr_->InsertScopeData(1, StateBase::msfb_.GetCurrentGait());
                ds_ptr_->InsertScopeData(2, sc_ptr_->GetFaultSnapshot().error_code );
                ds_ptr_->InsertScopeData(3, uc_ptr_->GetUserCommand()->target_gait);
            }
            // 500us
            // std::this_thread::sleep_for(std::chrono::microseconds(500));

            // 0.5s
            if(sc_ptr_->GetFaultSnapshot().error_code > 0 && (cnt % 1000 == 0)){
                // cnt = 0;
                LogRecord();
            }
        }
    }
    virtual void Stop() = 0;

    // 将Eigen::VectorXf转换成字符串
    template <typename T>
    std::string vectorToString(T vec) {
        std::stringstream ss;
        for (int i = 0; i < vec.size(); ++i) {
            ss << vec[i];
            if (i < vec.size() - 1) {
                ss << ", "; // 在元素之间添加空格
            }
        }
        return ss.str();
    }

    std::string intToBinary(int num) {
        std::bitset<32> bits(num); // 假设 int 是 32 位
        std::string binary = bits.to_string();
        // 去掉前导零
        size_t first_one = binary.find('1');
        if (first_one != std::string::npos) {
            binary = binary.substr(first_one);
        } else {
            binary = "0"; // 如果全是0
        }
        return binary;
    }

    void LogRecord(){
        // info
        logger_pos.info(vectorToString(ri_ptr_->GetJointPosition()));
        logger_vel.info(vectorToString(ri_ptr_->GetJointVelocity()));
        logger_torq.info(vectorToString(ri_ptr_->GetJointTorque()));
        logger_temp.info(vectorToString(ri_ptr_->GetMotorTemperture()));
        logger_imu.info(vectorToString(ri_ptr_->GetImuRpy()));
        logger_bat.info(vectorToString(ri_ptr_->GetBatteryData()));
        logger_stat.info(std::to_string(StateBase::msfb_.GetCurrentState()));
        logger_error.info(intToBinary(sc_ptr_->GetFaultSnapshot().error_code));
    }



    virtual std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) = 0;
    const RobotType robot_type_;
    
    // const RobotName robot_name_;
    // const RemoteCommandType remote_cmd_type_;    
    double _dt = 0.001, startTime;
    TimeTool set_timer{};
    int run_cnt_ = 0;
    double time_record_ = 0;
    
    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<DataStreaming> ds_ptr_;
    std::shared_ptr<SafeController> sc_ptr_;
    std::shared_ptr<MotionDataUpload> du_ptr_;
    #ifndef USE_SIMULATION
        std::shared_ptr<MotionDataPublish> dp_ptr_;
    #endif

    std::shared_ptr<StateBase> current_controller_;
    StateName current_state_name_, next_state_name_;
};




#endif