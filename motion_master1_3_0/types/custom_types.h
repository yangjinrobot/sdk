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
        StandPolicy     = 0x20007,    //地面起立 RL 策略，可切入阻尼或步态 RL
        StandShooting   = 0x20008,    //站立射击 RL 策略
        Ysc             = 0x20009,    //YSC RL 策略（policy_CR1_8.onnx）
        Mofang          = 0x2000A,    //21DOF 动作模仿策略
        ReStand         = 0x2000B,    //独立恢复站立/步态 RL 策略（restand.onnx）
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
        kStandPolicy    = RobotMotionState::StandPolicy,
        kStandShooting  = RobotMotionState::StandShooting,
        kYsc            = RobotMotionState::Ysc,
        kMofang         = RobotMotionState::Mofang,
        kReStand        = RobotMotionState::ReStand,
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
