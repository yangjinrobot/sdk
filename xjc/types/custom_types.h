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
    RLStandControlMode = 0x1100b, //对应手柄motion_1 强化学习站立状态（上肢可以遥操作）
    UpperBodyControlMode = 4,
    
    AmpZeroPos = 0x20006,
    AmpWholeBodyControlMode = 6,

    AmpWalkingZeroPos = 7,
    AmpWalkingControl = 0x11,

    // Protocol IDs 9..12 are retired; never reuse them for another state.
    AmpMoe15Dof = 0x1100c,
    AmpWalkingWAQ = 0x1100d,
    RLControlMode = 0x1100e,
    RLControlMimic12Mode = 0x1100f, //对应手柄motion_5  从站立坐到椅子上；
    RLControlMimic13Mode = 0x11010, //对应手柄motion_6  从椅子上站立起来；
    Stand2KneeMimicMode = 0x11011,
    Knee2StandMimicMode = 0x11012,
    RLStandSaluteMode = 0x11013, //对应手柄motion_9  手臂规划敬礼状态；
    RLStandSaluteLeftArmMode = 0x11014, //对应手柄motion_10  伸手安放炸药状态；

};

// 启用列表：在这里注释状态，指令入口和控制器注册同步停用。
// 使用 C++17 数组长度推导，增删条目时无需手动修改状态数量。
inline constexpr auto kRobotMotionStates = std::array{
    Idle, JointDamping, ZeroPos, RLStandControlMode, UpperBodyControlMode,
    AmpZeroPos, AmpWholeBodyControlMode, AmpWalkingZeroPos, AmpWalkingControl,
    AmpMoe15Dof, AmpWalkingWAQ, RLControlMode, RLControlMimic12Mode, RLControlMimic13Mode,
    Stand2KneeMimicMode, Knee2StandMimicMode,
    RLStandSaluteMode, RLStandSaluteLeftArmMode};

inline bool IsKnownRobotMotionState(uint32_t state) {
    return std::find(kRobotMotionStates.begin(), kRobotMotionStates.end(), state) != kRobotMotionStates.end();
}

enum RobotGaitType {
    HumanWALK = 0x21000,
};

enum StateName {
    kInvalid = -1,
    kIdle = RobotMotionState::Idle,
    kJointDamping = RobotMotionState::JointDamping,
    kZeroPos = RobotMotionState::ZeroPos,
    kRLStandControl = RobotMotionState::RLStandControlMode,
    kUpperBodyControl = RobotMotionState::UpperBodyControlMode,

    kAmpZeroPos = RobotMotionState::AmpZeroPos,
    kAmpWholeBodyControl = RobotMotionState::AmpWholeBodyControlMode,

    kAmpWalkingZeroPos = RobotMotionState::AmpWalkingZeroPos,
    kAmpWalkingControl = RobotMotionState::AmpWalkingControl,

    kAmpMoe15Dof = RobotMotionState::AmpMoe15Dof,
    kAmpWalkingWAQ = RobotMotionState::AmpWalkingWAQ,
    kRLControl = RobotMotionState::RLControlMode,
    kRLMimic12Control = RobotMotionState::RLControlMimic12Mode,
    kRLMimic13Control = RobotMotionState::RLControlMimic13Mode,
    kStand2KneeMimic = RobotMotionState::Stand2KneeMimicMode,
    kKnee2StandMimic = RobotMotionState::Knee2StandMimicMode,
    kRLStandSalute = RobotMotionState::RLStandSaluteMode,
    kRLStandSaluteLeftArm = RobotMotionState::RLStandSaluteLeftArmMode,
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
