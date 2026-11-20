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

        RLControlMode   = 0x11,     // RL模式，amp/1118/nirenwalk.onnx
        Restand         = 0x1100B,  // motion 1：STD RL 站立
        Run             = 0x1100D,  // motion 3：原 NirenWalk

        Sit2Stand       = 0x1100F,  // motion 5：坐到站
        Stand2Sit       = 0x1100E,  // motion 4：站到坐

        Stand2SitMimic = Stand2Sit, // CR1STANDARD 站立到坐下
        Sit2StandMimic = Sit2Stand, // CR1STANDARD 坐下到站立


        // 原 NirenWalk 的兼容名称现在等同 active Run。
        NirenWalk       = Run,

        // 后续旧 Run 控制器保留协议码但全部停用。
        Run1            = 0x11014, // motion 9：旧 1103_2 run
        Run2            = 0x11015, // motion 10：旧 1104 run_2
        Run3            = 0x11016, // motion 11：旧 1106 run_3
        Run4            = 0x11017, // motion 12：旧 1112 run_4

        ZeroPos         = 0x20001,    //回零位
        ZeroCalibration = 0x20004,    //通过打限位校准零位
        SuspendedStand  = 0x20006,    //悬架站立完成，可切入RL模式
        SquatPreparation = 0x20010,   //待机进入 21 轴蹲姿准备
    };

    inline uint32_t NormalizeMotionStateCommand(uint32_t command) {
        switch (command) {
            case 0x2000B: return RobotMotionState::Run;
            case 0x2000C: return RobotMotionState::Restand;
            case 0x2000D: case 0x18: return RobotMotionState::Stand2SitMimic;
            case 0x2000E: case 0x19: return RobotMotionState::Sit2StandMimic;
            default: return command;
        }
    }

    // Legacy Run1-Run4 are intentionally disabled. The new active Run is the
    // renamed former NirenWalk state at 0x1100D.
    inline bool IsDisabledRunMotionState(uint32_t motion_state) {
        return motion_state == RobotMotionState::Run1 ||
               motion_state == RobotMotionState::Run2 ||
               motion_state == RobotMotionState::Run3 ||
               motion_state == RobotMotionState::Run4;
    }

    // Commands within RL Stand; these are not independent motion states.
    enum RobotMotionCommand {
        StandElbowMinus04 = 0x11012, // motion 8: both elbows -0.4 rad
        StandElbowPlus12  = 0x11013, // motion 9: both elbows +1.2 rad
    };
    inline bool IsStandElbowCommand(uint32_t command) {
        return command == StandElbowMinus04 || command == StandElbowPlus12;
    }
    inline bool CanAcceptStandElbowCommand(uint32_t state, uint32_t pending,
                                          uint32_t safe_control_mode) {
        return state == RobotMotionState::Restand && safe_control_mode == 0 &&
               pending != RobotMotionState::JointDamping && pending != RobotMotionState::ZeroPos;
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
        kRLControl    = RobotMotionState::RLControlMode,
        kRun          = RobotMotionState::Run,
        kNirenWalk    = kRun, // 兼容旧代码，反馈码仍为 0x1100D
        kRun1         = RobotMotionState::Run1,
        kRun2         = RobotMotionState::Run2,
        kRun3         = RobotMotionState::Run3,
        kRun4         = RobotMotionState::Run4,
        kRestand      = RobotMotionState::Restand,
        kStand2SitMimic = RobotMotionState::Stand2SitMimic,
        kSit2StandMimic = RobotMotionState::Sit2StandMimic,
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
