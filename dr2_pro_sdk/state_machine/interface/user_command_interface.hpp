/**
 * @file user_command_interface.hpp
 * @brief this file is used for robot's user command input
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-12-12
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once

#include <termios.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <unordered_map>

#include "common_types.h"
#include "custom_types.h"
#include "drdds/msg/steer.hpp"
#include "motion_state_store.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iomanip>  // 需要包含头文件
#define AXIS_STEP 0.70f
#define KEYBOARD_REFRESH_MS 20
#define KEY_HOLD_TIMEOUT_MS 320.0

using namespace types;

namespace interface {

class UserCommandInterface {
 public:
  explicit UserCommandInterface(ControlMode control_mode, rclcpp::Node::SharedPtr node)
      : control_mode_(control_mode), node_(node) {
    if (!node_) {
      std::cerr << "UserCommandInterface node is null" << std::endl;
    }
    usr_cmd_ = new UserCommand();
    std::memset(usr_cmd_, 0, sizeof(UserCommand));

    gamepad_key_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/GAMEPAD_KEY", 1, std::bind(&UserCommandInterface::HandleKey, this, std::placeholders::_1));
    steer_sub_ = node_->create_subscription<drdds::msg::Steer>(
        "/STEER", 1, std::bind(&UserCommandInterface::HandleSteer, this, std::placeholders::_1));
#ifdef USE_SIMULATION
    sim_steer_sub_ = node_->create_subscription<drdds::msg::Steer>(
        "/SIM_STEER", 1,
        std::bind(&UserCommandInterface::HandleSimulationSteer, this, std::placeholders::_1));
#endif
  }

  virtual ~UserCommandInterface() { delete usr_cmd_; }

  virtual void Start() {
    std::memset(usr_cmd_, 0, sizeof(UserCommand));
    std::cout << "\n╔════════════════════════════════════════════════╗\n"
              << "║                GAMEPAD TELEOP                  ║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  Movement:  Left joystick\n"
              << "  Rotation:  Right joystick\n"
              << "  Mode:      L1 (zeropos)  L2 (walk_control) R1 (rl_stand) R2 (depth_walk) \n"
              << "╔════════════════════════════════════════════════╗\n"
              << "║                KEYBOARD TELEOP                 ║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  Movement:  W/S (forward/back)  A/D (left/right)\n"
              << "  Rotation:  Q (CCW)  E (CW)\n"
              << "  Mode:      Z (zeropos) C (walk_control) V (run_control) B (arm_control)\n"
              << "             U (rl_stand) M (amp_walking) N (depth_walk) P (ground_stand_0907) R (damping)\n"
              << "\n";

    start_thread_flag_ = true;
    kb_thread_ = std::thread(std::bind(&UserCommandInterface::KeyboardLoop, this));
  }

  virtual void Stop() {
    start_thread_flag_ = false;
    if (kb_thread_.joinable()) {
      kb_thread_.join();
    }
  }

  virtual UserCommand* GetUserCommand() { return usr_cmd_; }

  virtual void SetMotionStateStore(const std::shared_ptr<MotionStateStore>& motion_state_store) {
    motion_state_store_ = motion_state_store;
  }

  ControlMode control_mode_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MotionStateStore> motion_state_store_;
  UserCommand* usr_cmd_;

 private:
  double GetCurrentTimeStamp() {
    static timespec startup_timestamp;
    timespec now_timestamp;
    if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
      clock_gettime(CLOCK_MONOTONIC, &startup_timestamp);
    }
    clock_gettime(CLOCK_MONOTONIC, &now_timestamp);
    return (now_timestamp.tv_sec - startup_timestamp.tv_sec) * 1e3 +
           (now_timestamp.tv_nsec - startup_timestamp.tv_nsec) / 1e6;
  }

  RobotMotionState GetCurrentState() const {
    return motion_state_store_ ? motion_state_store_->GetState() : RobotMotionState::Idle;
  }

  void HandleKey(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << "[RAW KEY] " << msg->data << std::endl;   // 添加这一行
    static const std::unordered_map<std::string, KeyCode> key_map = {{"G20_KEY_L1", KeyCode::L1},
                                                                     {"G20_KEY_L2", KeyCode::L2},
                                                                     {"G20_KEY_R1", KeyCode::R1},
                                                                     {"G20_KEY_R2", KeyCode::R2}};

    KeyCode code = KeyCode::UNKNOWN;
    const auto it = key_map.find(msg->data);
    if (it != key_map.end()) {
      code = it->second;
    }

    const RobotMotionState current_state = GetCurrentState();
    switch (code) {
      case KeyCode::L1:
        if (current_state == RobotMotionState::Idle || current_state == RobotMotionState::UpperBodyControlMode ||
            current_state == RobotMotionState::JointDamping) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
          std::cout << "[MODE] Zero Pos\n";
        }
        break;
      case KeyCode::L2:
        if (current_state == RobotMotionState::ZeroPos) {
            usr_cmd_->target_mode = (control_mode_ == ControlMode::kUpperBody)
                                        ? uint8_t(RobotMotionState::UpperBodyControlMode)
                                        : uint8_t(RobotMotionState::WholeBodyControlMode);
            std::cout << "[MODE] WholeBody Control\n";
        } else if (current_state == RobotMotionState::DepthWalkControlMode) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::WholeBodyControlMode);
            std::cout << "[MODE] WholeBody Control\n";
        }
        break;
      //   case KeyCode::R1:   // 切换到 RunControl
      //   if (current_state == RobotMotionState::ZeroPos ||
      //       // current_state == RobotMotionState::WholeBodyControlMode ||
      //       current_state == RobotMotionState::RLStandControlMode) {   // 仅新增这一项
      //       usr_cmd_->target_mode = uint8_t(RobotMotionState::RunControlMode);
      //       std::cout << "[MODE] Run Control\n";
      //   }
      //   break;
      // break;
      // // case KeyCode::R2:
      // //   if (control_mode_ == ControlMode::kWholeBody)
      // //   {
      // //     usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
      // //     std::cout << "[MODE] Joint Damping\n";
      // //   }
      // //   break;
      // case KeyCode::R2:
      // if (control_mode_ == ControlMode::kWholeBody)
      // {
      //     usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
      //     std::cout << "[MODE] RL Stand Control\n";
      // }
      // break;
      case KeyCode::R1:
      // 切换到 RLStandControlMode
      // 可根据需要放宽或保留条件，这里建议允许从 ZeroPos 或任何运动状态切换
      if (current_state == RobotMotionState::ZeroPos ||
          current_state == RobotMotionState::WholeBodyControlMode ||
          current_state == RobotMotionState::RunControlMode ||
          current_state == RobotMotionState::ArmControlMode ||
          current_state == RobotMotionState::AmpWalkingControlMode) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
          std::cout << "[MODE] RL Stand Control\n";
      }
      break;
  
      case KeyCode::R2:
      // 切换到 DepthWalkControlMode（深度行走）
      // 与键盘 N 键保持一致，允许从 ZeroPos / RLStand / AmpWalking 进入
      if (control_mode_ == ControlMode::kWholeBody &&
          (current_state == RobotMotionState::ZeroPos ||
           current_state == RobotMotionState::RLStandControlMode ||
           current_state == RobotMotionState::AmpWalkingControlMode)) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::DepthWalkControlMode);
          std::cout << "[MODE] Depth Walk Control\n";
      }
      break;
      
      default:
        break;
    }
  }

  // void HandleSteer(const drdds::msg::Steer::SharedPtr msg) {
  //   std::cout << "[Steer] x=" << msg->data.x << " y=" << msg->data.y << " yaw=" << msg->data.yaw << std::endl;
  //   usr_cmd_->forward_vel_scale = msg->data.x;
  //   usr_cmd_->side_vel_scale = msg->data.y;
  //   usr_cmd_->turnning_vel_scale = msg->data.yaw;
  // }
//   void HandleSteer(const drdds::msg::Steer::SharedPtr msg) {
//     rclcpp::Time recv_time = node_->now();
//     rclcpp::Time send_time(msg->header.stamp);
//     double delay_ms = (recv_time - send_time).seconds() * 1000.0;

//     usr_cmd_->forward_vel_scale = msg->data.x;
//     usr_cmd_->side_vel_scale = msg->data.y;
//     usr_cmd_->turnning_vel_scale = msg->data.yaw;

//     // 使用固定小数点格式输出，保留6位小数
//     std::cout << std::fixed << std::setprecision(6)
//               << "[Steer] recv_time=" << recv_time.seconds()
//               << " send_time=" << send_time.seconds()
//               << " delay=" << delay_ms << " ms"
//               << " f=" << usr_cmd_->forward_vel_scale
//               << " s=" << usr_cmd_->side_vel_scale
//               << " t=" << usr_cmd_->turnning_vel_scale
//               << std::defaultfloat << std::endl;  // 恢复默认格式（可选）
// }

  void HandleSteer(const drdds::msg::Steer::SharedPtr msg) {

    auto now=std::chrono::system_clock::now();
    auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    auto timestamp_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    long nsec = (timestamp_us - timestamp_s * 1000000) * 1000;

    std::cout << "[Steer] timestamp sec=" << timestamp_s 
              << ", nsec=" << nsec << std::endl;

    rclcpp::Time recv_time = node_->now();
    rclcpp::Time send_time(msg->header.stamp);
    double delay_ms = (recv_time - send_time).seconds() * 1000.0;

    // Store the current message before logging it. Previously the log printed
    // the preceding message, which made joystick changes appear ineffective.
    usr_cmd_->forward_vel_scale = std::clamp(msg->data.x, -1.0f, 1.0f);
    usr_cmd_->side_vel_scale = std::clamp(msg->data.y, -1.0f, 1.0f);
    usr_cmd_->turnning_vel_scale = std::clamp(msg->data.yaw, -1.0f, 1.0f);

    std::cout << std::fixed << std::setprecision(6)
              << "[Steer] recv_time=" << recv_time.seconds()
              << " send_time=" << send_time.seconds()
              << " delay=" << delay_ms << " ms"
              << " raw=" << msg->data.x << "," << msg->data.y << "," << msg->data.yaw
              << " f=" << usr_cmd_->forward_vel_scale
              << " s=" << usr_cmd_->side_vel_scale
              << " t=" << usr_cmd_->turnning_vel_scale
              << std::defaultfloat << std::endl;  // 恢复默认格式（可选）
  }

#ifdef USE_SIMULATION
  void HandleSimulationSteer(const drdds::msg::Steer::SharedPtr msg) {
    // Simulation-only normalized velocity input. No button or mode-state handling.
    usr_cmd_->forward_vel_scale = std::clamp(msg->data.x, -1.0f, 1.0f);
    usr_cmd_->side_vel_scale = std::clamp(msg->data.y, -1.0f, 1.0f);
    usr_cmd_->turnning_vel_scale = std::clamp(msg->data.yaw, -1.0f, 1.0f);
  }
#endif

  // void KeyboardLoop() {
  //   struct termios oldt, newt;
  //   tcgetattr(STDIN_FILENO, &oldt);
  //   newt = oldt;
  //   newt.c_lflag &= ~(ICANON | ECHO);
  //   tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  //   int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  //   fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

  //   char input;
  //   double forward_last_input_ms = -1e9;
  //   double side_last_input_ms = -1e9;
  //   double turnning_last_input_ms = -1e9;
  //   bool keyboard_forward_active = false;
  //   bool keyboard_side_active = false;
  //   bool keyboard_turnning_frunctive = false;

  //   while (start_thread_flag_) {
  //     usr_cmd_->time_stamp = GetCurrentTimeStamp();
  //     const double current_time_ms = usr_cmd_->time_stamp;
  //     const RobotMotionState current_state = GetCurrentState();

  //     if (read(STDIN_FILENO, &input, 1) != -1) {
  //       const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
  //       std::cout << "input: " << input << std::endl;
  //       if (key == 'r' && control_mode_ == ControlMode::kWholeBody) {
  //         usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
  //       }
  //       switch (current_state) {
  //         case RobotMotionState::Idle:
  //         case RobotMotionState::UpperBodyControlMode:
  //           if (key == 'z') {
  //             usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
  //           }
  //           break;
  //         case RobotMotionState::ZeroPos:
  //           if (key == 'c') {
  //             usr_cmd_->target_mode = (control_mode_ == ControlMode::kUpperBody)
  //                                         ? uint8_t(RobotMotionState::UpperBodyControlMode)
  //                                         : uint8_t(RobotMotionState::WholeBodyControlMode);
  //           }
  //           break;
  //         case RobotMotionState::WholeBodyControlMode:
  //           if (key == 'w') {
  //             usr_cmd_->forward_vel_scale = AXIS_STEP;
  //             forward_last_input_ms = current_time_ms;
  //             keyboard_forward_active = true;
  //           } else if (key == 's') {
  //             usr_cmd_->forward_vel_scale = -AXIS_STEP;
  //             forward_last_input_ms = current_time_ms;
  //             keyboard_forward_active = true;
  //           }

  //           if (key == 'a') {
  //             usr_cmd_->side_vel_scale = AXIS_STEP;
  //             side_last_input_ms = current_time_ms;
  //             keyboard_side_active = true;
  //           } else if (key == 'd') {
  //             usr_cmd_->side_vel_scale = -AXIS_STEP;
  //             side_last_input_ms = current_time_ms;
  //             keyboard_side_active = true;
  //           }

  //           if (key == 'q') {
  //             usr_cmd_->turnning_vel_scale = AXIS_STEP;
  //             turnning_last_input_ms = current_time_ms;
  //             keyboard_turnning_active = true;
  //           } else if (key == 'e') {
  //             usr_cmd_->turnning_vel_scale = -AXIS_STEP;
  //             turnning_last_input_ms = current_time_ms;
  //             keyboard_turnning_active = true;
  //           }
  //           break;
  //         default:
  //           break;
  //       }
  //     }

  //     if (current_state == RobotMotionState::WholeBodyControlMode) {
  //       if (keyboard_forward_active && current_time_ms - forward_last_input_ms > KEY_HOLD_TIMEOUT_MS) {
  //         usr_cmd_->forward_vel_scale = 0.0f;
  //         keyboard_forward_active = false;
  //       }
  //       if (keyboard_side_active && current_time_ms - side_last_input_ms > KEY_HOLD_TIMEOUT_MS) {
  //         usr_cmd_->side_vel_scale = 0.0f;
  //         keyboard_side_active = false;
  //       }
  //       if (keyboard_turnning_active && current_time_ms - turnning_last_input_ms > KEY_HOLD_TIMEOUT_MS) {
  //         usr_cmd_->turnning_vel_scale = 0.0f;
  //         keyboard_turnning_active = false;
  //       }

  //       ClipNumber(usr_cmd_->forward_vel_scale, -1., 1.);
  //       ClipNumber(usr_cmd_->side_vel_scale, -1., 1.);
  //       ClipNumber(usr_cmd_->turnning_vel_scale, -1., 1.);
  //     } else {
  //       usr_cmd_->forward_vel_scale = 0.0f;
  //       usr_cmd_->side_vel_scale = 0.0f;
  //       usr_cmd_->turnning_vel_scale = 0.0f;
  //       keyboard_forward_active = false;
  //       keyboard_side_active = false;
  //       keyboard_turnning_active = false;
  //     }
  //     std::this_thread::sleep_for(std::chrono::milliseconds(KEYBOARD_REFRESH_MS));
  //   }
  //   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  // }
  void KeyboardLoop() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  
    char input;
  
    while (start_thread_flag_) {
      // ===== 新增：每帧重置所有手臂预设标志，保证上升沿检测 =====
      usr_cmd_->arm_preset_1 = false;
      usr_cmd_->arm_preset_2 = false;
      usr_cmd_->arm_preset_3 = false;
      usr_cmd_->arm_preset_4 = false;
      usr_cmd_->arm_preset_5 = false;
      usr_cmd_->arm_preset_6 = false;
      // 若 arm_preset_seq 用于其他目的，此处可不重置，由外部赋值
  
      usr_cmd_->time_stamp = GetCurrentTimeStamp();
      const RobotMotionState current_state = GetCurrentState();
  
      if (read(STDIN_FILENO, &input, 1) != -1) {
        const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
  
        // === 全局紧急阻尼 ===
        if (key == 'r' && control_mode_ == ControlMode::kWholeBody) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
          std::cout << "[KB] Joint Damping\n";
          continue;
        }
        if (key == 'p' && control_mode_ == ControlMode::kWholeBody) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::GroundStandControlMode);
          std::cout << "[KB] Ground Stand 0907 Control\n";
          continue;
        }
  
        // ===== 新增：数字键 1 触发 arm_preset_1（挥手） =====
        // 在 KeyboardLoop 的 read 处理中
        if (key == '1') {
          usr_cmd_->arm_preset_1 = true;
          std::cout << "[KB] arm_preset_1 (prep wave)\n";
        }
        else if (key == '2') {
          usr_cmd_->arm_preset_2 = true;
          std::cout << "[KB] arm_preset_2 (start wave)\n";
        }
        if (key == '3') {
          usr_cmd_->arm_preset_3 = true;
          std::cout << "[KB] arm_preset_3 (salute)\n";
        }
        else if (key == '4') {
            usr_cmd_->arm_preset_4 = true;
            std::cout << "[KB] arm_preset_4 (home)\n";
        }
  
        // === 速度控制（仅运动模式有效） ===
        bool is_motion = (current_state == RobotMotionState::WholeBodyControlMode ||
                          current_state == RobotMotionState::RunControlMode ||
                          current_state == RobotMotionState::ArmControlMode ||
                          current_state == RobotMotionState::RLStandControlMode ||
                          current_state == RobotMotionState::AmpWalkingControlMode ||
                          current_state == RobotMotionState::DepthWalkControlMode);
        if (is_motion) {
          if (key == 'w') {
            usr_cmd_->forward_vel_scale += 0.1f;
            if (usr_cmd_->forward_vel_scale > 1.0f) usr_cmd_->forward_vel_scale = 1.0f;
            std::cout << "[KB] forward=" << usr_cmd_->forward_vel_scale << std::endl;
          } else if (key == 's') {
            usr_cmd_->forward_vel_scale -= 0.1f;
            if (usr_cmd_->forward_vel_scale < -1.0f) usr_cmd_->forward_vel_scale = -1.0f;
            std::cout << "[KB] forward=" << usr_cmd_->forward_vel_scale << std::endl;
          } else if (key == 'a') {
            usr_cmd_->side_vel_scale += 0.1f;
            if (usr_cmd_->side_vel_scale > 1.0f) usr_cmd_->side_vel_scale = 1.0f;
            std::cout << "[KB] side=" << usr_cmd_->side_vel_scale << std::endl;
          } else if (key == 'd') {
            usr_cmd_->side_vel_scale -= 0.1f;
            if (usr_cmd_->side_vel_scale < -1.0f) usr_cmd_->side_vel_scale = -1.0f;
            std::cout << "[KB] side=" << usr_cmd_->side_vel_scale << std::endl;
          } else if (key == 'q') {
            usr_cmd_->turnning_vel_scale += 0.1f;
            if (usr_cmd_->turnning_vel_scale > 1.0f) usr_cmd_->turnning_vel_scale = 1.0f;
            std::cout << "[KB] turn=" << usr_cmd_->turnning_vel_scale << std::endl;
          } else if (key == 'e') {
            usr_cmd_->turnning_vel_scale -= 0.1f;
            if (usr_cmd_->turnning_vel_scale < -1.0f) usr_cmd_->turnning_vel_scale = -1.0f;
            std::cout << "[KB] turn=" << usr_cmd_->turnning_vel_scale << std::endl;
          }
        }
  
        // === 模式切换（原有逻辑保持不变） ===
        switch (current_state) {
          case RobotMotionState::Idle:
          case RobotMotionState::UpperBodyControlMode:
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            }
            break;
          case RobotMotionState::JointDamping:
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            }
            break;
          case RobotMotionState::ZeroPos:
            if (key == 'c') {
              usr_cmd_->target_mode = (control_mode_ == ControlMode::kUpperBody)
                                          ? uint8_t(RobotMotionState::UpperBodyControlMode)
                                          : uint8_t(RobotMotionState::WholeBodyControlMode);
              std::cout << "[KB] WholeBody Control\n";
            } else if (key == 'v') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RunControlMode);
              std::cout << "[KB] Run Control\n";
            } else if (key == 'b') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ArmControlMode);
              std::cout << "[KB] Arm Control\n";
            } else if (key == 'u' && control_mode_ == ControlMode::kWholeBody) {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
              std::cout << "[KB] RL Stand Control\n";
            } else if (key == 'n' && control_mode_ == ControlMode::kWholeBody) {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::DepthWalkControlMode);
              std::cout << "[KB] Depth Walk Control\n";
            }
            break;
          case RobotMotionState::WholeBodyControlMode:
            // if (key == 'z') {
            //   usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
            //   std::cout << "[KB] Zero Pos\n";
            // } else if (key == 'v') {
            //   usr_cmd_->target_mode = uint8_t(RobotMotionState::RunControlMode);
            //   std::cout << "[KB] Run Control\n";
            // } else if (key == 'b') {
            //   usr_cmd_->target_mode = uint8_t(RobotMotionState::ArmControlMode);
            //   std::cout << "[KB] Arm Control\n";
            // } else if (key == 'u') {
            //   usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
            //   std::cout << "[KB] RL Stand Control\n";
            // }
            // break;

            // 移除 v 和 b 的 else if
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            } else if (key == 'u') {   // 只保留 u，删除 v 和 b
                usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
                std::cout << "[KB] RL Stand Control\n";
            }
            break;
          case RobotMotionState::RunControlMode:
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            } else if (key == 'c') {
              usr_cmd_->target_mode = (control_mode_ == ControlMode::kUpperBody)
                                          ? uint8_t(RobotMotionState::UpperBodyControlMode)
                                          : uint8_t(RobotMotionState::WholeBodyControlMode);
              std::cout << "[KB] WholeBody Control\n";
            } else if (key == 'b') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ArmControlMode);
              std::cout << "[KB] Arm Control\n";
            }else if (key == 'u') {   // 新增：从 RunControl 切换到 RLStand
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
              std::cout << "[KB] RL Stand Control\n";
            }else if (key == 'm') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::AmpWalkingControlMode);
              std::cout << "[KB] Amp Walking Control\n";
            }
            break;
          case RobotMotionState::ArmControlMode:
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            } else if (key == 'c') {
              usr_cmd_->target_mode = (control_mode_ == ControlMode::kUpperBody)
                                          ? uint8_t(RobotMotionState::UpperBodyControlMode)
                                          : uint8_t(RobotMotionState::WholeBodyControlMode);
              std::cout << "[KB] WholeBody Control\n";
            } else if (key == 'v') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RunControlMode);
              std::cout << "[KB] Run Control\n";
            }
            break;
          case RobotMotionState::RLStandControlMode:
            if (key == 'm') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::AmpWalkingControlMode);
              std::cout << "[KB] AMP Walking Control\n";
            } else if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            }else if (key == 'v') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RunControlMode);
              std::cout << "[KB] Run Control\n";
            }else if (key == 'n') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::DepthWalkControlMode);
              std::cout << "[KB] Depth Walk Control\n";
            }
            break;
          case RobotMotionState::AmpWalkingControlMode:
            if (key == 'u') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
              std::cout << "[KB] RL Stand Control\n";
            } else if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            }else if (key == 'v') {   // 新增：允许从 RLStand 切换到 RunControl
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RunControlMode);
              std::cout << "[KB] Run Control\n";
            }else if (key == 'n') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::DepthWalkControlMode);
              std::cout << "[KB] Depth Walk Control\n";
            }
            break;
          case RobotMotionState::DepthWalkControlMode:
            if (key == 'u') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::RLStandControlMode);
              std::cout << "[KB] RL Stand Control\n";
            } else if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
              std::cout << "[KB] Zero Pos\n";
            } else if (key == 'c') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::WholeBodyControlMode);
              std::cout << "[KB] WholeBody Control\n";
            }
            break;
          default:
            break;
        }
      }
  
      // 非运动状态速度清零
      if (current_state != RobotMotionState::WholeBodyControlMode &&
          current_state != RobotMotionState::RunControlMode &&
          current_state != RobotMotionState::ArmControlMode &&
          current_state != RobotMotionState::RLStandControlMode &&
          current_state != RobotMotionState::AmpWalkingControlMode &&
          current_state != RobotMotionState::DepthWalkControlMode) {
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
      }
  
      // 裁剪
      usr_cmd_->forward_vel_scale = std::clamp(usr_cmd_->forward_vel_scale, -1.0f, 1.0f);
      usr_cmd_->side_vel_scale   = std::clamp(usr_cmd_->side_vel_scale,   -1.0f, 1.0f);
      usr_cmd_->turnning_vel_scale = std::clamp(usr_cmd_->turnning_vel_scale, -1.0f, 1.0f);
  
      std::this_thread::sleep_for(std::chrono::milliseconds(KEYBOARD_REFRESH_MS));
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
  bool start_thread_flag_ = false;
  std::thread kb_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamepad_key_sub_;
  rclcpp::Subscription<drdds::msg::Steer>::SharedPtr steer_sub_;
#ifdef USE_SIMULATION
  rclcpp::Subscription<drdds::msg::Steer>::SharedPtr sim_steer_sub_;
#endif
};
};  // namespace interface
