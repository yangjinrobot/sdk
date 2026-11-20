/**
 * @file state_machine_base.hpp
 * @brief state machine base class for different robot state machine control
 * @author deeprobotics
 * @version 1.0
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */

#ifndef STATE_MACHINE_BASE_HPP_
#define STATE_MACHINE_BASE_HPP_

#include <csignal>
#include "state_base.hpp"
#include "safe_controller/safe_controller.hpp"
#include "motion_data_upload.hpp"
#include "motion_data_publish.hpp"
#include "dr_logger.hpp"
#include <bitset>

class StateMachineBase
{
private:
    DrLogger logger_temp, logger_pos, logger_vel, logger_torq, logger_imu, logger_bat, logger_stat, logger_error;

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

public:
    StateMachineBase(const RobotType& robot_type)
        : logger_temp("Temp"), logger_pos("Pos"), logger_vel("Vel"), logger_torq("Torq"),
          logger_imu("Imu"), logger_bat("Bat"), logger_stat("Stat"), logger_error("Error"),
          robot_type_(robot_type) {}
    virtual ~StateMachineBase() = default;

    inline static volatile std::sig_atomic_t stop_requested_ = 0;
    static void RequestStop(int = 0) {
        stop_requested_ = 1;
    }

    virtual void Start() = 0;
    void GetNextStateName() {
        next_state_name_ = current_controller_->LoseControlJudge()
            ? StateName::kJointDamping : current_controller_->GetNextStateName();
    }
    virtual void Run() {
        int cnt = 0;
        set_timer.time_init(floor(_dt * 1e3 + 1e-8));
        while (!stop_requested_) {
            if (!set_timer.time_interrupt()) {
                ++cnt;
                ri_ptr_->RefreshRobotData();
                time_record_ = ri_ptr_->GetInterfaceTimeStamp();
                if (uc_ptr_) {
                    const auto uc = uc_ptr_->GetUserCommandSnapshot();
                    ri_ptr_->SetNeckTakeoverCmd(uc.neck_yaw_cmd, uc.neck_pitch_cmd);
                }
                const auto command = uc_ptr_->GetUserCommandSnapshot();
                if (current_state_name_ != StateName::kIdle && current_state_name_ != StateName::kJointDamping &&
                    (command.safe_control_mode != 0 || command.target_mode == RobotMotionState::JointDamping)) {
                    SwitchState(StateName::kJointDamping);
                }
                current_controller_->Run();

                GetNextStateName();
                SwitchState(next_state_name_);
                ++run_cnt_;
                ds_ptr_->InsertScopeData(0, StateBase::msfb_.GetCurrentState());
                ds_ptr_->InsertScopeData(1, StateBase::msfb_.GetCurrentGait());
                ds_ptr_->InsertScopeData(2, sc_ptr_->GetFaultSnapshot().error_code);
                ds_ptr_->InsertScopeData(3, uc_ptr_->GetUserCommandSnapshot().target_gait);
            }

            if (sc_ptr_->GetFaultSnapshot().error_code > 0 && cnt % 1000 == 0) {
                LogRecord();
            }
        }
    }
    // Resolve the controller before exiting the current state. Entry failures
    // must replace the previous powered command immediately with damping.
    void SwitchState(StateName state_name) {
        while (state_name != current_state_name_) {
            auto next_controller = GetStateControllerPtr(state_name);
            if (!next_controller) {
                state_name = StateName::kJointDamping;
                next_controller = GetStateControllerPtr(state_name);
            }
            if (!next_controller) throw std::runtime_error("Missing joint damping controller");

            current_controller_->OnExit();
            logger_stat.info("%s  ------------> %s",
                current_controller_->state_name_.c_str(), next_controller->state_name_.c_str());
            std::cout << current_controller_->state_name_ << " ------------> " << next_controller->state_name_ << std::endl;
            current_controller_ = next_controller;
            current_state_name_ = next_state_name_ = state_name;

            try {
                current_controller_->OnEnter();
            } catch (const std::exception& error) {
                if (state_name == StateName::kJointDamping) throw;
                std::cerr << "State entry failed: " << error.what() << std::endl;
                state_name = StateName::kJointDamping;
                continue;
            }

            if (state_name == StateName::kJointDamping) {
                current_controller_->Run();
                return;
            }
            if (current_controller_->motion_state_store_->GetState() != RobotMotionState::JointDamping) return;
            state_name = StateName::kJointDamping;
        }
    }

    virtual void Stop() = 0;

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

    void LogRecord() {
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
    std::shared_ptr<MotionDataPublish> dp_ptr_;

    std::shared_ptr<StateBase> current_controller_;
    StateName current_state_name_, next_state_name_;
};

#endif
