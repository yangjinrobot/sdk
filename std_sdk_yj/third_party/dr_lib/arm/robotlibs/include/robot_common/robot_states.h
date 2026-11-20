#ifndef COMMON_ROBOT_STATES_H
#define COMMON_ROBOT_STATES_H

#include <string>

namespace robot_common {

/**
 * @brief 机器人运动状态枚举
 */
enum class RobotMotionState {
  kIdle = 0,      // 空闲状态
  kStandby,       // 待机状态
  kInitializing,  // 初始化中
  kReady,         // 就绪状态
  kMoving,        // 运动中
  kPaused,        // 暂停状态
  kEmergencyStop, // 急停状态
  kError,         // 错误状态
  kRecovering,    // 恢复中
  kShuttingDown   // 关机中
};

/**
 * @brief 机器人运动事件枚举
 */
enum class RobotMotionEvent {
  kPowerOn = 0,      // 上电
  kPowerOff,         // 断电
  kInitialize,       // 初始化
  kInitDone,         // 初始化完成
  kStart,            // 开始运动
  kStop,             // 停止运动
  kPause,            // 暂停
  kResume,           // 恢复
  kEmergency,        // 急停触发
  kEmergencyRelease, // 急停释放
  kErrorOccurred,    // 发生错误
  kErrorCleared,     // 错误清除
  kRecoveryDone,     // 恢复完成
  kTimeout,          // 超时
  kHeartbeat         // 心跳
};

/**
 * @brief 任务执行状态枚举
 */
enum class TaskState {
  kPending = 0, // 待执行
  kQueued,      // 已入队
  kRunning,     // 执行中
  kCompleted,   // 已完成
  kFailed,      // 失败
  kCancelled,   // 已取消
  kTimeout      // 超时
};

/**
 * @brief 任务事件枚举
 */
enum class TaskEvent {
  kEnqueue = 0, // 入队
  kExecute,     // 开始执行
  kComplete,    // 完成
  kFail,        // 失败
  kCancel,      // 取消
  kRetry,       // 重试
  kTimeout      // 超时
};

/**
 * @brief 状态转字符串工具
 */
inline std::string RobotMotionStateToString(RobotMotionState state) {
  switch (state) {
  case RobotMotionState::kIdle:
    return "Idle";
  case RobotMotionState::kStandby:
    return "Standby";
  case RobotMotionState::kInitializing:
    return "Initializing";
  case RobotMotionState::kReady:
    return "Ready";
  case RobotMotionState::kMoving:
    return "Moving";
  case RobotMotionState::kPaused:
    return "Paused";
  case RobotMotionState::kEmergencyStop:
    return "EmergencyStop";
  case RobotMotionState::kError:
    return "Error";
  case RobotMotionState::kRecovering:
    return "Recovering";
  case RobotMotionState::kShuttingDown:
    return "ShuttingDown";
  default:
    return "Unknown";
  }
}

inline std::string RobotMotionEventToString(RobotMotionEvent event) {
  switch (event) {
  case RobotMotionEvent::kPowerOn:
    return "PowerOn";
  case RobotMotionEvent::kPowerOff:
    return "PowerOff";
  case RobotMotionEvent::kInitialize:
    return "Initialize";
  case RobotMotionEvent::kInitDone:
    return "InitDone";
  case RobotMotionEvent::kStart:
    return "Start";
  case RobotMotionEvent::kStop:
    return "Stop";
  case RobotMotionEvent::kPause:
    return "Pause";
  case RobotMotionEvent::kResume:
    return "Resume";
  case RobotMotionEvent::kEmergency:
    return "Emergency";
  case RobotMotionEvent::kEmergencyRelease:
    return "EmergencyRelease";
  case RobotMotionEvent::kErrorOccurred:
    return "ErrorOccurred";
  case RobotMotionEvent::kErrorCleared:
    return "ErrorCleared";
  case RobotMotionEvent::kRecoveryDone:
    return "RecoveryDone";
  case RobotMotionEvent::kTimeout:
    return "Timeout";
  case RobotMotionEvent::kHeartbeat:
    return "Heartbeat";
  default:
    return "Unknown";
  }
}

} // namespace robot_common

#endif // COMMON_ROBOT_STATES_H
