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
              << "  Modes:     Z (zeropos)  X (suspended stand)\n"
              << "             C (RL control)  V (run)  R (joint damping)\n"
              << "  Entry:     idle/calibration -> Z (zeropos) or X (suspended stand)\n"
              << "  Transitions: zeropos/suspended stand -> C; RL -> V; run -> C\n"
              << "  Squat: G (DunXia) from RL/run/Stand; H (ZhanLi) only after squat\n"
              << "  Sit/stand:  RL/run -> T (sit); seated -> Y (stand)\n"
              << "             after standing: C (RL) / V (run); Z or R exits\n"
              << "  Velocity:  W/S/A/D/Q/E work in RL and run\n"
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

        if (robot_name_ == RobotName::CR1STANDARD) {
          if ((current_state == RobotMotionState::RLControlMode ||
               current_state == RobotMotionState::Run ||
               current_state == RobotMotionState::Sit2StandMimic) && key == 't')
            usr_cmd_->target_mode = RobotMotionState::Stand2SitMimic;
          if (current_state == RobotMotionState::Stand2SitMimic && key == 'y')
            usr_cmd_->target_mode = RobotMotionState::Sit2StandMimic;
          if (current_state == RobotMotionState::Sit2StandMimic && key == 'c')
            usr_cmd_->target_mode = RobotMotionState::RLControlMode;
          if (current_state == RobotMotionState::Sit2StandMimic && key == 'v')
            usr_cmd_->target_mode = RobotMotionState::Run;
          if ((current_state == RobotMotionState::Stand2SitMimic ||
               current_state == RobotMotionState::Sit2StandMimic) && key == 'z')
            usr_cmd_->target_mode = RobotMotionState::ZeroPos;
        }

        if (robot_name_ == RobotName::CR1STANDARD) {
          if (key == 'g' && (current_state == RobotMotionState::RLControlMode ||
                            current_state == RobotMotionState::Run ||
                            current_state == RobotMotionState::Restand ||
                            current_state == RobotMotionState::ZhanLi))
            usr_cmd_->target_mode = RobotMotionState::DunXia;
          if (key == 'h' && current_state == RobotMotionState::DunXia)
            usr_cmd_->target_mode = RobotMotionState::ZhanLi;
          if (current_state == RobotMotionState::ZhanLi) {
            if (key == 'c') usr_cmd_->target_mode = RobotMotionState::RLControlMode;
            if (key == 'v') usr_cmd_->target_mode = RobotMotionState::Run;
            if (key == 't') usr_cmd_->target_mode = RobotMotionState::Stand2SitMimic;
          }
          if (key == 'z' && (current_state == RobotMotionState::DunXia ||
                            current_state == RobotMotionState::ZhanLi))
            usr_cmd_->target_mode = RobotMotionState::ZeroPos;
        }

        if (key == 'r') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::JointDamping);
        }

        if (current_state == RobotMotionState::RLControlMode && key == 'z') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
        }
        if (current_state == RobotMotionState::RLControlMode && key == 'v') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::Run);
        }
        if (current_state == RobotMotionState::Run && key == 'c') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::RLControlMode);
        }
        if (current_state == RobotMotionState::Run && key == 'z') {
          usr_cmd_->target_mode = static_cast<uint32_t>(RobotMotionState::ZeroPos);
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
            break;
          default:
            break;
        }

        const bool velocity_control_state = current_state == RobotMotionState::RLControlMode ||
                                            current_state == RobotMotionState::Run;
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
          current_state == RobotMotionState::Run) {
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
