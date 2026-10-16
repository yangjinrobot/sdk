#pragma once

#include "common_types.h"

namespace types {
enum RobotName {
    DR2PRO = 0,
};

enum ControlMode {
    kUpperBody = 1,
    kWholeBody = 2,
};

enum RobotMotionState {
    Idle = 0,
    JointDamping = 1,
    ZeroPos = 2,
    WholeBodyControlMode = 3,
    UpperBodyControlMode = 4,
    RunControlMode = 5,   
    ArmControlMode = 6,   
    ArmLoadZeroPos = 7,
    RLStandControlMode = 8,
    AmpWalkingControlMode = 9,
    DepthWalkControlMode = 10,
    GroundStandControlMode = 11,
};

enum RobotGaitType {
    HumanWALK = 0x21000,
};

enum StateName {
    kInvalid = -1,
    kIdle = RobotMotionState::Idle,
    kJointDamping = RobotMotionState::JointDamping,
    kZeroPos = RobotMotionState::ZeroPos,
    kArmLoadZeroPos = RobotMotionState::ArmLoadZeroPos,

    kWholeBodyControl = RobotMotionState::WholeBodyControlMode,
    kUpperBodyControl = RobotMotionState::UpperBodyControlMode,
    kRunControl = RobotMotionState::RunControlMode,   // 新增
    kArmControl = RobotMotionState::ArmControlMode,   // 新增
    kRLStandControl = RobotMotionState::RLStandControlMode,
    kAmpWalkingControl = RobotMotionState::AmpWalkingControlMode,
    kDepthWalkControl = RobotMotionState::DepthWalkControlMode,
    kGroundStandControl = RobotMotionState::GroundStandControlMode,
};

enum KeyCode {
    L1,
    L2,
    R1,
    R2,
    UNKNOWN
};

enum CommandIndex {
  kIndexDisable = 1,
  kIndexEnable = 2,
  kIndexMotorControl = 4,
  kIndexErrorReset = 17
};

};  // namespace types
