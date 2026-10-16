/**
 * @file humanoid_state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-12-12
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once
#include "dr2_pro_interface.hpp"

#include "state_base.h"
#include "zeropos_state.hpp"
#include "idle_state.hpp"
#include "joint_damping_state.hpp"
#include "whole_body_control_state.hpp"
#include "upper_body_control_state.hpp"
#include "run_control_state.hpp"
#include "arm_control_state.hpp"
#include "rl_stand_control_state.hpp"
#include "amp_walking_state.hpp"
#include "depth_walk_state.hpp"
#include "ground_stand_control_state.hpp"

#include "safe_controller.hpp"
#include "user_command_interface.hpp"
namespace humanoid {
class HumanoidStateMachine {
private:
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> zeropos_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> whole_body_controller_;
    std::shared_ptr<StateBase> upper_body_controller_;
    std::shared_ptr<StateBase> run_control_controller_;
    std::shared_ptr<StateBase> arm_control_controller_;
    std::shared_ptr<StateBase> rl_stand_controller_;
    std::shared_ptr<StateBase> amp_walking_controller_;
    std::shared_ptr<StateBase> depth_walk_controller_;
    std::shared_ptr<StateBase> ground_stand_controller_;
    StateName last_state_name_;

public:
    int run_cnt_ = 0;
    TimeTool set_timer{};
    std::shared_ptr<MotionStateStore> motion_state_store_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<SafeController> sc_ptr_;
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<StateBase> current_controller_;
    StateName current_state_name_, next_state_name_;

    std::thread run_thread_;

    const RobotName robot_name_;
    const ControlMode control_mode_;

    HumanoidStateMachine(RobotName robot_name, ControlMode control_mode = ControlMode::kWholeBody)
        : robot_name_(robot_name),
          control_mode_(control_mode),
          motion_state_store_(std::make_shared<MotionStateStore>()) {}

    ~HumanoidStateMachine() {}

    void Start() {
        node_ = std::make_shared<rclcpp::Node>("sdk_deploy");
        uc_ptr_ = std::make_shared<UserCommandInterface>(control_mode_, node_);
        uc_ptr_->SetMotionStateStore(motion_state_store_);

        ri_ptr_ = std::make_shared<DR2ProInterface>(node_);
        cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);

        std::cout << "HumanoidInterface finish" << std::endl;

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        data_ptr->motion_state_store = motion_state_store_;
        data_ptr->control_mode = control_mode_;

        std::cout << "Control mode: "
                  << (control_mode_ == ControlMode::kUpperBody ? "upper_body" : "whole_body") << std::endl;

        sc_ptr_ = std::make_shared<SafeController>("");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);

        idle_controller_ = std::make_shared<IdleState>(robot_name_, "idle_state", data_ptr);
        zeropos_controller_ = std::make_shared<ZeroPosState>(robot_name_, "zeropos_state", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name_, "joint_damping", data_ptr);
        whole_body_controller_ = std::make_shared<WholeBodyControlState>(robot_name_, "whole_body_control", data_ptr);
        upper_body_controller_ = std::make_shared<UpperBodyControlState>(robot_name_, "upper_body_control", data_ptr);
        run_control_controller_ = std::make_shared<RunControlState>(robot_name_, "run_control_state", data_ptr);
        arm_control_controller_ = std::make_shared<ArmControlState>(robot_name_, "arm_control_state", data_ptr);
        rl_stand_controller_ =
            std::make_shared<RLStandControlState>(robot_name_, "rl_stand_state", data_ptr);
        amp_walking_controller_ =
            std::make_shared<AmpWalkingState>(robot_name_, "amp_walking_state", data_ptr);
        depth_walk_controller_ =
            std::make_shared<DepthWalkState>(robot_name_, "depth_walk_state", data_ptr);
        ground_stand_controller_ =
            std::make_shared<GroundStandControlState>(robot_name_, "ground_stand_0907_state", data_ptr);
        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;

        std::this_thread::sleep_for(std::chrono::seconds(3));  // for safety

        ri_ptr_->Start();
        uc_ptr_->Start();
#ifndef USE_SIMULATION
        sc_ptr_->Start();
#endif
        current_controller_->OnEnter();
        
        std::cout << "start finish" << std::endl;
    }

    void Run() {
        set_timer.time_init(1);  // old is 2 = 500Hz， now is 5 = 200Hz
        run_thread_ = std::thread(&HumanoidStateMachine::RunThread, this);
        
        while (rclcpp::ok()) {
            if (set_timer.time_interrupt()) {
                ri_ptr_->RefreshRobotData();

                current_controller_->Run();

                if (current_controller_->LoseControlJudge()) {
                    next_state_name_ = StateName::kJointDamping;
                } else {
                    next_state_name_ = current_controller_->GetNextStateName();
                }

                if (next_state_name_ != current_state_name_) {
                    current_controller_->OnExit();
                    std::cout << current_controller_->state_name_ << " ------------> ";
                    current_controller_ = GetStateControllerPtr(next_state_name_);
                    std::cout << current_controller_->state_name_ << std::endl;
                    current_controller_->OnEnter();
                    current_state_name_ = next_state_name_;
                }
                ++run_cnt_;
            }
        }
    }

    void RunThread() {
        if (rclcpp::ok()) {
            rclcpp::spin(node_);
        }
    }

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) {
        switch (state_name) {
            case StateName::kInvalid: {
                return nullptr;
            }
            case StateName::kIdle: {
                return idle_controller_;
            }
            case StateName::kJointDamping: {
                return joint_damping_controller_;
            }
            case StateName::kZeroPos: {
                return zeropos_controller_;
            }
            case StateName::kWholeBodyControl: {
                return whole_body_controller_;
            }
            case StateName::kUpperBodyControl: {
                return upper_body_controller_;
            }
            case StateName::kRunControl: {
                return run_control_controller_;
            }
            case StateName::kArmControl: {
                return arm_control_controller_;
            }
            case StateName::kRLStandControl: {
                return rl_stand_controller_;
            }
            case StateName::kAmpWalkingControl: {
                return amp_walking_controller_;
            }
            case StateName::kDepthWalkControl: {
                return depth_walk_controller_;
            }
            case StateName::kGroundStandControl: {
                return ground_stand_controller_;
            }
            default: {
                std::cerr << "error state name" << std::endl;
                return joint_damping_controller_;
            }
        }
        return nullptr;
    }

    void Stop() {
        if (run_thread_.joinable()) {
            run_thread_.join();
        }
        sc_ptr_->Stop();
        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }
};
};
