#ifndef KEYBOARD_INTERFACE_HPP_
#define KEYBOARD_INTERFACE_HPP_

#include <termios.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <functional>
#include <map>
#include <thread>
#include <unistd.h>

#include "custom_types.h"
#include "user_command_interface.h"

#define KEY_STEP 0.10f
#define KEYBOARD_REFRESH_MS 20
#define KEY_DEBOUNCE_MS 300.0

using namespace interface;
using namespace types;

namespace {
static void ClipNumber(float &val, float lo, float hi) {
  if (val < lo)
    val = lo;
  else if (val > hi)
    val = hi;
}
}  // namespace

class KeyboardInterface : public UserCommandInterface {
 public:
  explicit KeyboardInterface(RobotName robot_name = RobotName::CR1STANDARD)
      : UserCommandInterface(robot_name) {
    std::cout << "Using Keyboard Command Interface" << std::endl;
  }

  virtual ~KeyboardInterface() { Stop(); }

  virtual void Start() {
    std::memset(usr_cmd_, 0, sizeof(UserCommand));

    std::cout << "\n╔════════════════════════════════════════════════╗\n"
              << "║                KEYBOARD TELEOP                 ║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  Movement:  W/S (forward/back)  A/D (left/right), 0.1 per press\n"
              << "             Q (CCW)  E (CW), speed is retained until changed\n"
              << "Mode:      Z (zeropos)  X (suspended stand)  M (ground stand)  P (stand shooting)  I (ysc)\n"
              << "           O (mofang, from zeropos/RL)\n"
              << "           C (RL control)  V (restand RL)  R (joint damping)\n"
              << "Flow:      Z/X -> C (RL) or V (restand)  |  V from RL/mofang switches to restand\n"
              << "           M/P/I -> C (policy -> RL); P: 1 (raise hand), 2 (lower hand)\n"
              << "           Z from RL returns to zeropos\n"
              << "\n";

    start_thread_flag_ = true;
    kb_thread_ = std::thread(std::bind(&KeyboardInterface::KeyboardLoop, this));
  }

  virtual void Stop() {
    start_thread_flag_ = false;
    if (kb_thread_.joinable()) {
      kb_thread_.join();
    }
  }

  virtual UserCommand* GetUserCommand() { return usr_cmd_; }

 private:
  RobotMotionState GetCurrentState() const {
    return msfb_ ? static_cast<RobotMotionState>(msfb_->GetCurrentState()) : RobotMotionState::WaitingForStand;
  }

  static double GetCurrentTimeMs(const std::chrono::steady_clock::time_point &start) {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
               std::chrono::steady_clock::now() - start)
        .count();
  }

  void KeyboardLoop() {
    struct termios oldt, newt;
    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
      std::cerr << "KeyboardInterface: failed to get terminal attr" << std::endl;
      return;
    }

    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    char input = '\0';
    const auto start_time = std::chrono::steady_clock::now();
    std::map<char, double> key_last_trigger_ms;

    while (start_thread_flag_) {
      const double current_time_ms = GetCurrentTimeMs(start_time);
      const RobotMotionState current_state = GetCurrentState();
      bool has_input = false;
      char key = '\0';

      if (read(STDIN_FILENO, &input, 1) != -1) {
        has_input = true;
        key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
      }

      if (has_input) {
        usr_cmd_->time_stamp = current_time_ms;

        if (key == 'r') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::JointDamping);
        }

        // Ground stand-up policy; controller mappings remain unchanged.
        if (key == 'm') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::StandPolicy);
        }

        // YSC policy (policy_CR1_8.onnx).  This is intentionally global like
        // M so it can be selected directly from a keyboard-controlled session.
        if (key == 'i') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::Ysc);
        }

        if (current_state == RobotMotionState::RLControlMode && key == 'z') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
        }

        if ((current_state == RobotMotionState::ZeroPos ||
             current_state == RobotMotionState::RLControlMode) && key == 'o') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::Mofang);
        }

        if ((current_state == RobotMotionState::ZeroPos ||
             current_state == RobotMotionState::SuspendedStand ||
             current_state == RobotMotionState::RLControlMode ||
             current_state == RobotMotionState::Mofang) && key == 'v') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ReStand);
        }

        switch (current_state) {
          case RobotMotionState::WaitingForStand:
          case RobotMotionState::ZeroCalibration:
            if (key == 'z') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
            }
            if (key == 'x') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::SuspendedStand);
            }
            break;
          case RobotMotionState::SuspendedStand:
            if (key == 'z') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
            }
            if (key == 'x') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::SuspendedStand);
            }
            if (key == 'c') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
            }
            break;
          case RobotMotionState::ZeroPos:
            if (key == 'c') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
            }
            if (key == 'z') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
            }
            if (key == 'p') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::StandShooting);
              usr_cmd_->target_action = 2;
            }
            break;
          case RobotMotionState::StandPolicy:
            if (key == 'c') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
            }
            break;
          case RobotMotionState::StandShooting:
            if (key == 'c') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
            } else if (key == '1') {
              usr_cmd_->target_action = 1;
            } else if (key == '2') {
              usr_cmd_->target_action = 2;
            }
            break;
          case RobotMotionState::Mofang:
            if (key == 'c') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
            }
            break;
          case RobotMotionState::ReStand:
            if (key == 'c') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
            } else if (key == 'z') {
              usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
            }
            break;
          default:
            break;
        }

        const bool velocity_control_state =
            current_state == RobotMotionState::RLControlMode ||
            current_state == RobotMotionState::ReStand || current_state == RobotMotionState::Ysc;
        if (velocity_control_state &&
            (key == 'w' || key == 's' || key == 'a' || key == 'd' || key == 'q' || key == 'e')) {
          const double last_ms = key_last_trigger_ms.count(key) ? key_last_trigger_ms[key] : -1e9;
          if (current_time_ms - last_ms > KEY_DEBOUNCE_MS) {
            key_last_trigger_ms[key] = current_time_ms;
            if (key == 'w') usr_cmd_->forward_vel_scale += KEY_STEP;
            else if (key == 's') usr_cmd_->forward_vel_scale -= KEY_STEP;
            else if (key == 'a') usr_cmd_->side_vel_scale += KEY_STEP;
            else if (key == 'd') usr_cmd_->side_vel_scale -= KEY_STEP;
            else if (key == 'q') usr_cmd_->turnning_vel_scale += KEY_STEP;
            else if (key == 'e') usr_cmd_->turnning_vel_scale -= KEY_STEP;
          }
        }
      }

      if (current_state == RobotMotionState::RLControlMode ||
          current_state == RobotMotionState::ReStand || current_state == RobotMotionState::Ysc) {
        ClipNumber(usr_cmd_->forward_vel_scale, -1.0f, 1.0f);
        ClipNumber(usr_cmd_->side_vel_scale, -1.0f, 1.0f);
        ClipNumber(usr_cmd_->turnning_vel_scale, -1.0f, 1.0f);
      } else {
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
        key_last_trigger_ms.clear();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(KEYBOARD_REFRESH_MS));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  std::atomic<bool> start_thread_flag_{false};
  std::thread kb_thread_;
};

#endif
