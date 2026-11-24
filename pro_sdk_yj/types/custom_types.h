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
    RLStandControlMode = 0x1100b,
    UpperBodyControlMode = 4,
    
    AmpZeroPos = 0x20006,
    SquatPreparation = 0x20010,
    AmpWholeBodyControlMode = 6,

    AmpWalkingZeroPos = 7,
    AmpWalkingControl = 0x11,

    // Protocol IDs 9..12 are retired; never reuse them for another state.
    AmpMoe15Dof = 0x1100c,
    AmpWalkingWAQ = 0x1100d,
    RLControlMode = 0x1100e,//4
    RLStandTalkingMode = 0x1100f,//5
    RLStandPlease2Mode = 0x11010,//6
    Stand2KneeMimicMode = 0x11011,
    Knee2StandMimicMode = 0x11012,
    RLStandSaluteMode = 0x11013,//9
    RLStandSaluteLeftArmMode = 0x11014,
    RLStandPleaseMode = 0x11015,
    // RLStandTalkingMode = 0x11016,
    // RLStandPlease2Mode = 0x11017,
    RLControlMimic12Mode = 0x11016,//12
    RLControlMimic13Mode = 0x11017,//13
    RLControlMimic2Mode = 0x11018,  // 14 box_fight(low kp): only reachable from AmpWalkingControl.
    RLControlMimic1Mode = 0x11019,  // 15 box fight(high kp): only reachable from AmpWalkingControl.
    TangxiaMode = 0x1101a,          // 16 Lie down: only reachable from RLControlMode.
    ZhanliMode = 0x1101b,           // 17 Stand up: only reachable from TangxiaMode.

};

// Explicit membership: protocol IDs are sparse and must not use range checks.
inline constexpr std::array<RobotMotionState, 26> kRobotMotionStates{
    Idle, JointDamping, ZeroPos, RLStandControlMode, UpperBodyControlMode,
    AmpZeroPos, SquatPreparation, AmpWholeBodyControlMode, AmpWalkingZeroPos, AmpWalkingControl,
    AmpMoe15Dof, AmpWalkingWAQ, RLControlMode, RLControlMimic12Mode, RLControlMimic13Mode,
    Stand2KneeMimicMode, Knee2StandMimicMode, RLStandSaluteMode, RLStandSaluteLeftArmMode,
    RLStandPleaseMode, RLStandTalkingMode, RLStandPlease2Mode,
    RLControlMimic1Mode, RLControlMimic2Mode, TangxiaMode, ZhanliMode};

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
    kSquatPreparation = RobotMotionState::SquatPreparation,
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
    kRLStandPlease = RobotMotionState::RLStandPleaseMode,
    kRLStandTalking = RobotMotionState::RLStandTalkingMode,
    kRLStandPlease2 = RobotMotionState::RLStandPlease2Mode,
    kRLMimic1Control = RobotMotionState::RLControlMimic1Mode,
    kRLMimic2Control = RobotMotionState::RLControlMimic2Mode,
    kTangxia = RobotMotionState::TangxiaMode,
    kZhanli = RobotMotionState::ZhanliMode,
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
