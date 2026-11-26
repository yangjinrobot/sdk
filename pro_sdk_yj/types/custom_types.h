#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"
#include <array>

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

enum ControlMode {
    kUpperBody = 1,
    kWholeBody = 2,
};

enum RobotMotionState : uint32_t {
    WaitingForStand = 0x0,  // Zero torque at startup.
    Idle = WaitingForStand,
    JointDamping = 0x2,
    ZeroPos = 0x20002,  // Separate calibration state; 0x2 is damping.
    UpperBodyControlMode = 4,
    
    AmpZeroPos = 0x20006,
    SquatPreparation = 0x20010,
    AmpWholeBodyControlMode = 6,

    AmpWalkingZeroPos = 7,
    AmpControl_Walk_Mode = 0x11,

    // Protocol IDs 0x9..0xc and 0x11017..0x1101b are retired.
    RLControl_Stand_Mode = 0x1100b,
    AmpControl_Walk2Run_Mode = 0x1100c,
    RLControl_StandTalking_Mode = 0x1100d,
    RLControl_StandPlease2_Mode = 0x1100e,
    RLControl_StandSalute_Mode = 0x1100f,
    RLControl_StandSaluteLeftArm_Mode = 0x11010,
    RLControl_StandPlease_Mode = 0x11011,
    MimicControl_ChairDown_Mode = 0x11012,
    MimicControl_ChairUp_Mode = 0x11013,
    MimicControl_BoxFight_Mode = 0x11014,
    MimicControl_LieDown_Mode = 0x11015,
    MimicControl_GetUp_Mode = 0x11016,
};

// Explicit membership: protocol IDs are sparse and must not use range checks.
inline constexpr std::array<RobotMotionState, 21> kRobotMotionStates{
    Idle,                              // 空闲，等待站立
    JointDamping,                      // 关节阻尼
    ZeroPos,                           // 回到预设零位
    UpperBodyControlMode,              // 上半身轨迹控制
    AmpZeroPos,                        // AMP 准备姿态
    SquatPreparation,                  // 下蹲至准备姿态
    AmpWholeBodyControlMode,           // AMP 全身控制
    AmpWalkingZeroPos,                 // AMP 行走准备姿态
    AmpControl_Walk_Mode,              // 拟人行走（手柄「开始运动/RL」）
    RLControl_Stand_Mode,              // 15 自由度 RL 站立
    AmpControl_Walk2Run_Mode,          // 跑步（walk2run）
    RLControl_StandTalking_Mode,       // 配合讲解的手势
    RLControl_StandPlease2_Mode,       // 欢迎手势
    RLControl_StandSalute_Mode,        // 站姿敬礼
    RLControl_StandSaluteLeftArm_Mode, // 左手向前做欢迎手势（欢迎 1）
    RLControl_StandPlease_Mode,        // 双臂向左做邀请手势（欢迎 2）
    MimicControl_ChairDown_Mode,       // 坐到椅子上
    MimicControl_ChairUp_Mode,         // 从坐姿起立
    MimicControl_BoxFight_Mode,        // 低 KP 拳击
    MimicControl_LieDown_Mode,         // 躺下（SDK 保留，当前版本禁用）
    MimicControl_GetUp_Mode,           // 从躺姿起立（SDK 保留，当前版本禁用）
};

inline bool IsKnownRobotMotionState(uint32_t state) {
    return std::find(kRobotMotionStates.begin(), kRobotMotionStates.end(), state) != kRobotMotionStates.end();
}

inline bool IsEnabledRobotMotionState(uint32_t state) {
    return IsKnownRobotMotionState(state) && state != MimicControl_LieDown_Mode && state != MimicControl_GetUp_Mode;
}

enum RobotGaitType {
    HumanWALK = 0x21000,
};

enum StateName {
    kInvalid = -1,
    kIdle = RobotMotionState::Idle,
    kJointDamping = RobotMotionState::JointDamping,
    kZeroPos = RobotMotionState::ZeroPos,
    kRLStandControl = RobotMotionState::RLControl_Stand_Mode,
    kUpperBodyControl = RobotMotionState::UpperBodyControlMode,

    kAmpZeroPos = RobotMotionState::AmpZeroPos,
    kSquatPreparation = RobotMotionState::SquatPreparation,
    kAmpWholeBodyControl = RobotMotionState::AmpWholeBodyControlMode,

    kAmpWalkingZeroPos = RobotMotionState::AmpWalkingZeroPos,
    kAmpWalkingControl = RobotMotionState::AmpControl_Walk_Mode,

    kRLControl = RobotMotionState::AmpControl_Walk2Run_Mode,
    kRLMimic12Control = RobotMotionState::MimicControl_ChairDown_Mode,
    kRLMimic13Control = RobotMotionState::MimicControl_ChairUp_Mode,
    kRLStandSalute = RobotMotionState::RLControl_StandSalute_Mode,
    kRLStandSaluteLeftArm = RobotMotionState::RLControl_StandSaluteLeftArm_Mode,
    kRLStandPlease = RobotMotionState::RLControl_StandPlease_Mode,
    kRLStandTalking = RobotMotionState::RLControl_StandTalking_Mode,
    kRLStandPlease2 = RobotMotionState::RLControl_StandPlease2_Mode,
    kRLMimic2Control = RobotMotionState::MimicControl_BoxFight_Mode,
    kTangxia = RobotMotionState::MimicControl_LieDown_Mode,
    kZhanli = RobotMotionState::MimicControl_GetUp_Mode,
};

enum KeyCode {
    L1,
    L2,
    R1,
    R2,
    UNKNOWN
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
