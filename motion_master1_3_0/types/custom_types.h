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

    // 手柄接收状态集合
    enum RobotMotionState{
        WaitingForStand = 0x0,  //0力矩，开机默认
        JointDamping    = 0x2,  //软急停，阻尼状态

        RLControlMode   = 0x11, //RL模式

        ZeroPos         = 0x20001,    //回零位
        ZeroCalibration = 0x20004,    //通过打限位校准零位
        SuspendedStand  = 0x20006,    //悬架站立完成，可切入RL模式
        ReStand         = 0x2000B,    //独立恢复站立/步态 RL 策略（restand.onnx）
        ZhanliShooting  = 0x2000C,    //站立射击 RL 策略（policy_shoot_walk.onnx）
        ShootDown       = 0x2000D,    //默认站姿 -> 射击下蹲（model_33000.onnx）
        ShootUp         = 0x2000E,    //射击下蹲 -> 默认站姿（model_28000.onnx）
        AmpGunWalking   = 0x2000F,    //AMP 持枪行走（policy_0911_zjie.onnx，21DOF）
    };

    enum RobotGaitType{
        //基础模式
        HumanWALK  = 0x21000,   //走路
    };
    
    enum StateName{
        kInvalid      = -1,
        kIdle         = RobotMotionState::WaitingForStand,  //
        kJointDamping = RobotMotionState::JointDamping,
        kSuspendedStand = RobotMotionState::SuspendedStand,
        kReStand        = RobotMotionState::ReStand,
        kZhanliShooting = RobotMotionState::ZhanliShooting,
        kShootDown      = RobotMotionState::ShootDown,
        kShootUp        = RobotMotionState::ShootUp,
        kAmpGunWalking  = RobotMotionState::AmpGunWalking,
        kRLControl    = RobotMotionState::RLControlMode,
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
