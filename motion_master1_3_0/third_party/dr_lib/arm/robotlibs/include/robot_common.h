#ifndef ROBOT_COMMON_H
#define ROBOT_COMMON_H

/**
 * @file robot_common.h
 * @brief 机器人公共库统一头文件
 *
 * 包含所有公共库组件的头文件：
 * - 线程管理 (thread_utils.h)
 * - 状态机管理 (state_machine.h)
 * - 时间工具 (time_utils.h)
 * - 机器人状态定义 (robot_states.h)
 */

#include "robot_common/robot_states.h"
#include "robot_common/state_machine.h"
#include "robot_common/thread_utils.h"
#include "robot_common/time_utils.h"

#endif // ROBOT_COMMON_H
