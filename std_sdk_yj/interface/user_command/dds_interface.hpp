#ifndef DDS_INTERFACE_HPP_
#define DDS_INTERFACE_HPP_

#include "user_command_interface.h"
#include "custom_types.h"
#include "dr_logger.hpp"
#include "drdds/core/drdds_core.h"

using namespace interface;
using namespace types;

class DdsInterface : public UserCommandInterface{
protected:
    std::unique_ptr<ChannelMotionState> state_data_sub_;
    std::unique_ptr<ChannelSteer> steer_data_sub_;
    std::unique_ptr<ChannelGait> gait_data_sub_;
    std::unique_ptr<ChannelStdMsgInt32> action_data_sub_;
    std::unique_ptr<ChannelSteer> vel_data_sub_;
    std::unique_ptr<ChannelSteer> neck_steer_data_sub_;

    std::thread monitor_thread_;
    bool monitor_flag_ = true;
    DrLogger logger_DDS_cmd;
public:
    DdsInterface(RobotName robot_name = RobotName::CR1STANDARD)
        : UserCommandInterface(robot_name), logger_DDS_cmd("DDSCMD") {
        std::cout << "Using Dds Command Interface" << std::endl;

        state_data_sub_ = std::make_unique<ChannelMotionState>(
            std::bind(&DdsInterface::HandlerState, this, std::placeholders::_1),
            "/MOTION_STATE", 0);

        steer_data_sub_ = std::make_unique<ChannelSteer>(
            std::bind(&DdsInterface::HandlerSteer, this, std::placeholders::_1),
            "/STEER", 0);
        vel_data_sub_ = std::make_unique<ChannelSteer>(
            std::bind(&DdsInterface::HandlerVel, this, std::placeholders::_1),
             "/REAL_STEER", 0);

        gait_data_sub_ = std::make_unique<ChannelGait>(
            std::bind(&DdsInterface::HandlerGait, this, std::placeholders::_1),
            "/GAIT", 0);

        action_data_sub_ = std::make_unique<ChannelStdMsgInt32>(
        std::bind(&DdsInterface::HandlerAction, this, std::placeholders::_1),
        "/ACTION", 0);

        neck_steer_data_sub_ = std::make_unique<ChannelSteer>(
            std::bind(&DdsInterface::HandlerNeckSteer, this, std::placeholders::_1),
            "/NECK_STEER", 0);
    }

    ~DdsInterface() {
        monitor_flag_ = false;
        if (monitor_thread_.joinable())
            monitor_thread_.join();

        // 智能指针自动析构
        gait_data_sub_.reset();
        steer_data_sub_.reset();
        state_data_sub_.reset();
        neck_steer_data_sub_.reset();
    }

    virtual UserCommand* GetUserCommand(){
        return usr_cmd_;
    }

    void MonitorSteer(){
        while(monitor_flag_){
            //2s
            if(!steer_data_sub_->IsUpdate(2000) && !vel_data_sub_->IsUpdate(2000)){
                usr_cmd_->forward_vel_scale = 0.0;
                usr_cmd_->side_vel_scale = 0.0;
                usr_cmd_->turnning_vel_scale = 0.0;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
    }

    // NiRenWalk、GunShootWalk、Walk2Run 和 RL Stand 响应 /REAL_STEER、/STEER。
    bool IsVelocityControlActive() const {
        const uint32_t state = msfb_->GetCurrentState();
        if (state == RobotMotionState::AmpControl_NiRenWalk_Mode) {
            return msfb_->GetCurrentGait() == RobotGaitType::HumanWALK;
        }
        if (state == RobotMotionState::AmpControl_GunShootWalk_Mode) {
            return msfb_->GetCurrentGait() == RobotGaitType::HumanWALK;
        }
        if (state == RobotMotionState::AmpControl_Walk2Run_Mode || state == RobotMotionState::MjlabControl_Walk2Run_Mode) return true;
        if (state == RobotMotionState::RLControl_Stand_Mode) return true;
        return false;
    }

    void HandlerVel(const drdds::msg::Steer* data) {
        if (!IsVelocityControlActive()) {
            usr_cmd_->forward_vel_scale = 0;
            usr_cmd_->side_vel_scale = 0;
            usr_cmd_->turnning_vel_scale = 0;
            return;
        }

        usr_cmd_->forward_vel_scale = RetLimitNumber(data->data().x(), -1.0, 1.0);
        usr_cmd_->side_vel_scale = RetLimitNumber((data->data().y() / 0.75), -1.0, 1.0);
        usr_cmd_->turnning_vel_scale = RetLimitNumber((data->data().yaw() / 2.25), -1.0, 1.0);
    }
    virtual void Start(){
        monitor_thread_ = std::thread(std::bind(&DdsInterface::MonitorSteer, this));
    }
    virtual void Stop(){
        monitor_flag_ = false;
        monitor_thread_.join();
    }

    void HandlerDeadTopic(const drdds::msg::NavCmd* data) {
        
    }
    // ================================================================
    // 状态/步态权限过滤（固定最严格策略 rt="0"）
    // ================================================================

    // robot_type 归一化：非 "0"/"1"/"2"/"3" 均按 "0" 处理（最严格策略）
    static const char* NormalizeRobotType(const char* rt) {
        if (!rt) return "0";
        if (std::strcmp(rt, "0") == 0 || std::strcmp(rt, "1") == 0 ||
            std::strcmp(rt, "2") == 0 || std::strcmp(rt, "3") == 0) return rt;
        return "0";
    }

    // cert 不可用时仅告警一次，避免高频话题每秒刷屏
    bool IsStateAllowedByCert(uint32_t state) {
        const char* rt = "0";  // 固定最严格策略
        const char* tp = "std";

        rt = NormalizeRobotType(rt);

        if (!tp) return false;

        // robot_type "0"(error) / "3"(shipping) 仅允许白名单内 state
        if (std::strcmp(rt, "0") == 0 || std::strcmp(rt, "3") == 0) {
            switch (state) {
                case RobotMotionState::WaitingForStand:      // 0x0
                case RobotMotionState::JointDamping:         // 0x2
                case RobotMotionState::AmpControl_NiRenWalk_Mode:       // 0x11
                case RobotMotionState::AmpControl_GunShootWalk_Mode:    // 0x1100B
                case RobotMotionState::RLControl_Stand_Mode:            // 0x1100C
                case RobotMotionState::AmpControl_Walk2Run_Mode:        // 0x1100D
                case RobotMotionState::MjlabControl_Walk2Run_Mode:
                case RobotMotionState::MimicControl_ChairDown_Mode:     // 0x1100E
                case RobotMotionState::MimicControl_ChairUp_Mode:       // 0x1100F
                case RobotMotionState::MimicControl_Kneel_Mode:         // 0x11010
                case RobotMotionState::MimicControl_Kneel_StandUp_Mode: // 0x11011
                case RobotMotionCommand::StandElbowMinus04:             // motion 8
                case RobotMotionCommand::StandElbowPlus12:              // motion 9
                case RobotMotionState::ZeroPos:              // 0x20001
                case RobotMotionState::ZeroCalibration:      // 0x20004
                case RobotMotionState::SuspendedStand:       // 0x20006
                case RobotMotionState::SquatPreparation:    // 0x20010
                    return robot_name_ == RobotName::CR1STANDARD;
                default:
                    return false;
            }
        }

        return true;
    }

    bool IsGaitAllowedByCert(uint32_t gait) {
        const char* rt = "0";  // 固定最严格策略
        const char* tp = "std";

        rt = NormalizeRobotType(rt);
        if (!tp) return false;

        // 边界校验：gait 范围 0x21000~0x23FFF，拒绝越界值
        if (gait < 0x21000 || gait > 0x23FFF) {
            logger_DDS_cmd.warn("Gait 0x%x rejected: out of range", gait);
            return false;
        }

        // robot_type "0"(error) / "3"(shipping) 仅允许白名单内 gait
        if (std::strcmp(rt, "0") == 0 || std::strcmp(rt, "3") == 0) {
            switch (gait) {
                case RobotGaitType::HumanWALK:  // 0x21000
                    return true;
                default:
                    return false;
            }
        }

        return true;
    }

    bool IsActionAllowedByCert(int32_t action) {
        const char* rt = "0";  // 固定最严格策略
        const char* tp = "std";

        rt = NormalizeRobotType(rt);
        if (!tp) return false;

        // 边界校验：action 范围 0x3000~0x3004，拒绝越界及负值
        if (action < 0x3000 || action > 0x3004) {
            logger_DDS_cmd.warn("Action %d rejected: out of range", action);
            return false;
        }

        // robot_type "0"(error) / "3"(shipping) 禁止所有 action
        if (std::strcmp(rt, "0") == 0 || std::strcmp(rt, "3") == 0) {
            return false;
        }

        return true;
    }

    void HandlerState(const drdds::msg::MotionState* data) {
        uint32_t state = NormalizeMotionStateCommand(data->data().state());
        std::cout << "[DDS] /MOTION_STATE received: 0x" << std::hex << state << std::dec << std::endl;
        if (!IsStateAllowedByCert(state)) {
            logger_DDS_cmd.warn("State 0x%x rejected by cert", state);
            std::cout << "[DDS] motion state rejected by cert: 0x" << std::hex << state << std::dec << std::endl;
            return;
        }

        if (state == RobotMotionState::SquatPreparation || state == RobotMotionState::MimicControl_ChairUp_Mode) {
            const uint32_t status = msfb_->GetHandsetStatus();
            if (state == RobotMotionState::SquatPreparation &&
                ((status & ~MotionStateFeedback::kHandsetReadyFlag) != RobotMotionState::WaitingForStand ||
                 usr_cmd_->safe_control_mode != 0)) {
                logger_DDS_cmd.warn("SquatPreparation rejected: status=0x%x safe=%u",
                                    status, usr_cmd_->safe_control_mode);
                return;
            }
            // Discard a handset confirmation received before the measured squat is ready.
            if (state == RobotMotionState::MimicControl_ChairUp_Mode &&
                status == RobotMotionState::SquatPreparation) {
                logger_DDS_cmd.warn("ChairUp rejected: squat preparation is not ready");
                return;
            }
        }
        if (IsStandElbowCommand(state) &&
            !CanAcceptStandElbowCommand(msfb_->GetCurrentState(), usr_cmd_->target_mode,
                                       usr_cmd_->safe_control_mode)) return;
        const auto current = msfb_->GetCurrentState();
        if (state == RobotMotionState::MjlabControl_Walk2Run_Mode && !CanRequestMjlabControlWalk2Run(current, *usr_cmd_)) {
            logger_DDS_cmd.warn("MjlabControl_Walk2Run_Mode rejected: current=0x%x target=0x%x safe=%u",
                                current, usr_cmd_->target_mode, usr_cmd_->safe_control_mode);
            std::cout << "[DDS] mjrun rejected: current=0x" << std::hex << current
                      << " target=0x" << usr_cmd_->target_mode << std::dec
                      << " safe=" << usr_cmd_->safe_control_mode << std::endl;
            return;
        }
        if (current == RobotMotionState::MjlabControl_Walk2Run_Mode && state != RobotMotionState::JointDamping &&
            (!IsMjlabControlWalk2RunExitMotionState(state) || usr_cmd_->safe_control_mode != 0 ||
             usr_cmd_->target_mode == RobotMotionState::JointDamping)) return;
        usr_cmd_->target_mode = state;
        if (state == RobotMotionState::MjlabControl_Walk2Run_Mode) {
            logger_DDS_cmd.info("MjlabControl_Walk2Run_Mode request accepted from 0x%x", current);
        }
        std::cout << "target_mode: " << int(usr_cmd_->target_mode) << std::endl;
        logger_DDS_cmd.info("target_mode: %u", usr_cmd_->target_mode);
    }

    void HandlerGait(const drdds::msg::Gait* data) {
        uint32_t target_gait = data->data().gait();

        if (!IsGaitAllowedByCert(target_gait)) {
            logger_DDS_cmd.warn("Gait 0x%x rejected by cert", target_gait);
            return;
        }

        SetTargetGait(target_gait);
        std::cout << "target_gait: " << int(usr_cmd_->target_gait) << std::endl;
        logger_DDS_cmd.info("target_gait: %u", usr_cmd_->target_gait);
        //TODO步态特殊处理
    }
    void HandlerAction(const drdds::msg::StdMsgInt32* data) {
        int32_t target_action = data->value();

        if (!IsActionAllowedByCert(target_action)) {
            logger_DDS_cmd.warn("Action %d rejected by cert", target_action);
            return;
        }

        SetTargetAction(target_action);
        std::cout << "target_action: " << (usr_cmd_->target_action) << std::endl;
        logger_DDS_cmd.info("target_action: %u", usr_cmd_->target_action);
    }
    
    void HandlerSteer(const drdds::msg::Steer* data){
        if(data->data().x() != 0 || data->data().y() != 0 || data->data().z() != 0 || data->data().roll() != 0 || data->data().pitch() != 0 || data->data().yaw() != 0){
            std::cout << "Steer Value: "
                  << data->data().x() << " " << data->data().y() << " " << data->data().z() << " " 
                  << data->data().roll() << " " << data->data().pitch() << " " << data->data().yaw() << std::endl;
        }
        
        if (!IsVelocityControlActive()) {
            usr_cmd_->forward_vel_scale = 0;
            usr_cmd_->side_vel_scale = 0;
            usr_cmd_->turnning_vel_scale = 0;
            usr_cmd_->targetPitch = 0;
            usr_cmd_->targetRoll = 0;
            usr_cmd_->targetHeight = 0;
            return;
        }

        usr_cmd_->forward_vel_scale = data->data().x();
        usr_cmd_->side_vel_scale = data->data().y();
        usr_cmd_->turnning_vel_scale = data->data().yaw();
    }

    // 脖子接管指令源：独立 /NECK_STEER 话题，避免被状态/步态分支清零。
    void HandlerNeckSteer(const drdds::msg::Steer* data){
        usr_cmd_->neck_yaw_cmd   = data->data().yaw();
        usr_cmd_->neck_pitch_cmd = data->data().pitch();
        logger_DDS_cmd.info("neck_steer: yaw=%.3f pitch=%.3f",
                            usr_cmd_->neck_yaw_cmd, usr_cmd_->neck_pitch_cmd);
    }
};

#endif
