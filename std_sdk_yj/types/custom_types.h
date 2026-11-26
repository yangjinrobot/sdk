#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"

namespace types{
    enum RobotName{
        CR1STANDARD = 6,    // cr1 标准版（21DOF）
        CR1PRO = 7,         // cr1 pro版（31DOF）
    };

    enum RobotType{
        HumanoidLeg,
        HumanoidStandard,
        HumanoidPro,
    };

    // 基础控制、动作跟踪和安全状态。
    enum RobotMotionState{
        WaitingForStand = 0x0,  //0力矩，开机默认
        JointDamping    = 0x2,  //软急停，阻尼状态

        AmpControl_NiRenWalk_Mode       = 0x11,     // 开始运动/RL，新的拟人行走
        AmpControl_GunShootWalk_Mode    = 0x1100B,  // motion 1：持枪行走
        RLControl_Stand_Mode            = 0x1100C,  // motion 2：RL 站立
        AmpControl_Walk2Run_Mode        = 0x1100D,  // motion 3：walk2run

        MimicControl_ChairDown_Mode     = 0x1100E,  // motion 4：椅子坐姿
        MimicControl_ChairUp_Mode       = 0x1100F,  // motion 5：坐姿起立
        MimicControl_Kneel_Mode         = 0x11010,  // motion 6：跪姿
        MimicControl_Kneel_StandUp_Mode = 0x11011,  // motion 7：跪姿起立

        MjlabControl_Walk2Run_Mode      = 0x11018,  // mjrun

        ZeroPos         = 0x20001,    //回零位
        ZeroCalibration = 0x20004,    //通过打限位校准零位
        SuspendedStand  = 0x20006,    //悬架站立完成，可切入RL模式
        SquatPreparation = 0x20010,   //待机进入 21 轴蹲姿准备

    };

    inline uint32_t NormalizeMotionStateCommand(uint32_t command) {
        switch (command) {
            case 0x2000B: return RobotMotionState::AmpControl_Walk2Run_Mode;
            case 0x2000C: return RobotMotionState::RLControl_Stand_Mode;
            case 0x2000D: case 0x18: return RobotMotionState::MimicControl_ChairDown_Mode;
            case 0x2000E: case 0x19: return RobotMotionState::MimicControl_ChairUp_Mode;
            default: return command;
        }
    }

    // Commands within RL Stand; these are not independent motion states.
    enum RobotMotionCommand {
        StandElbowMinus04 = 0x11012, // motion 8: both elbows -0.4 rad
        StandElbowPlus12  = 0x11013, // motion 9: both elbows +1.2 rad
    };
    inline bool IsStandElbowCommand(uint32_t command) {
        return command == StandElbowMinus04 || command == StandElbowPlus12;
    }
    inline bool RequiresRLControlStandTransition(uint32_t source, uint32_t target) {
        if (source == target) return false;
        if (source == RobotMotionState::AmpControl_NiRenWalk_Mode &&
            (target == RobotMotionState::AmpControl_GunShootWalk_Mode ||
             target == RobotMotionState::MimicControl_ChairDown_Mode))
            return true;
        if (source == RobotMotionState::AmpControl_GunShootWalk_Mode &&
            (target == RobotMotionState::AmpControl_NiRenWalk_Mode ||
             target == RobotMotionState::AmpControl_Walk2Run_Mode))
            return true;
        if (source == RobotMotionState::AmpControl_Walk2Run_Mode &&
            target == RobotMotionState::AmpControl_GunShootWalk_Mode)
            return true;
        if ((source == RobotMotionState::MimicControl_ChairUp_Mode ||
             source == RobotMotionState::MimicControl_Kneel_StandUp_Mode) &&
            (target == RobotMotionState::AmpControl_NiRenWalk_Mode ||
             target == RobotMotionState::AmpControl_GunShootWalk_Mode))
            return true;
        if (source == RobotMotionState::MimicControl_ChairUp_Mode &&
            target == RobotMotionState::AmpControl_Walk2Run_Mode)
            return true;
        return false;
    }
    inline bool CanAcceptStandElbowCommand(uint32_t state, uint32_t pending,
                                          uint32_t safe_control_mode) {
        return state == RobotMotionState::RLControl_Stand_Mode && safe_control_mode == 0 &&
               pending != RobotMotionState::JointDamping && pending != RobotMotionState::ZeroPos;
    }

    inline bool CanRequestMjlabControlWalk2Run(uint32_t state, const UserCommand& command) {
        const bool source_allows_mjrun =
            state == RobotMotionState::AmpControl_NiRenWalk_Mode ||
            state == RobotMotionState::AmpControl_GunShootWalk_Mode ||
            state == RobotMotionState::RLControl_Stand_Mode ||
            state == RobotMotionState::AmpControl_Walk2Run_Mode;
        return source_allows_mjrun && command.safe_control_mode == 0 &&
               command.target_mode != RobotMotionState::JointDamping &&
               command.target_mode != RobotMotionState::ZeroPos;
    }

    inline bool IsMjlabControlWalk2RunExitMotionState(uint32_t state) {
        switch (state) {
            case RobotMotionState::JointDamping:
            case RobotMotionState::AmpControl_NiRenWalk_Mode:
            case RobotMotionState::AmpControl_GunShootWalk_Mode:
            case RobotMotionState::RLControl_Stand_Mode:
            case RobotMotionState::AmpControl_Walk2Run_Mode:
            case RobotMotionState::MimicControl_Kneel_Mode:
            case RobotMotionState::MimicControl_ChairDown_Mode:
            case RobotMotionState::ZeroPos:
                return true;
            default:
                return false;
        }
    }

    enum RobotGaitType{
        //基础模式
        HumanWALK  = 0x21000,   //走路
    };
    
    enum StateName{
        kInvalid      = -1,
        kIdle         = RobotMotionState::WaitingForStand,  //
        kJointDamping = RobotMotionState::JointDamping,
        kSuspendedStand = RobotMotionState::SuspendedStand,
        kSquatPreparation = RobotMotionState::SquatPreparation,
        kAmpControlNiRenWalkMode          = RobotMotionState::AmpControl_NiRenWalk_Mode,
        kAmpControlGunShootWalkMode            = RobotMotionState::AmpControl_GunShootWalk_Mode,
        kRLControlStandMode            = RobotMotionState::RLControl_Stand_Mode,
        kAmpControlWalk2RunMode        = RobotMotionState::AmpControl_Walk2Run_Mode,
        kMjlabControlWalk2RunMode      = RobotMotionState::MjlabControl_Walk2Run_Mode,
        kMimicControlChairDownMode     = RobotMotionState::MimicControl_ChairDown_Mode,
        kMimicControlChairUpMode       = RobotMotionState::MimicControl_ChairUp_Mode,
        kMimicControlKneelMode         = RobotMotionState::MimicControl_Kneel_Mode,
        kMimicControlKneelStandUpMode  = RobotMotionState::MimicControl_Kneel_StandUp_Mode,
        kZeroPos      = RobotMotionState::ZeroPos,
        kZeroCalibration = RobotMotionState::ZeroCalibration,

    };

enum RemoteCommandType{
        kRetroid,
        kSkydroid,
        kDDS,
        kKeyboard,
    };
    

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};

#endif
