#ifndef KEYBOARD_INTERFACE_HPP_
#define KEYBOARD_INTERFACE_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include "user_command_interface.h"
#include "udp_node.hpp"
#ifndef MOTION_MASTER_COMMANDS_OFFLINE
#include "drdds/core/drdds_core.h"
#endif

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface {
public:
    explicit KeyboardInterface(RobotName robot = RobotName::CR1PRO,
                               ControlMode mode = ControlMode::kWholeBody)
        : UserCommandInterface(robot, mode) {}
    ~KeyboardInterface() override {
        Stop();
        EnableDdsInput(false);
    }
    void Start() override {
        std::cout << "CR1PRO DDS protocol: idle=0x0, damping=0x2, AMP zero=0x20006, walking=0x11, stand=0x1100b, MoE=0x1100c, WAQ=0x1100d, AMP=0x1100e, sit=0x11016, rise=0x11017, kneel=0x11011, kneel rise=0x11012, salute=0x11013, salute left arm=0x11014, lie down=0x1101a, stand up=0x1101b\n"
                  << "  U: walking/RL stand -> salute; M/C/L2: salute -> AMP walking with smooth arm release\n"
                  << "  O: AMP preparation/RL stand/AMP walking/MoE -> WAQ; V/M: WAQ -> stand/AMP walking\n"
                  << "  Gamepad (whole-body): L1 AMP zero pos; L2 AMP walking; R1 RL stand; R2 RL stand -> AMP MoE 15 DOF\n"
                  << "  B: stand/walking -> AMP; N: AMP -> sit; M: sit -> rise; B: rise -> AMP\n"
                  << "  P: RL stand -> AMP MoE; V: MoE -> RL stand; M: MoE -> AMP walking\n"
                  << "  Gamepad (upper-body): L1 zero pos; L2 upper-body control\n"
                  << "  Z: idle/damping/zero pos -> AMP zero pos; C: AMP zero pos -> AMP walking\n"
                  << "  V: AMP walking -> RL stand; M: RL stand -> AMP walking\n"
                  << "  F: idle/damping/AMP whole body -> AMP zero pos; H: AMP zero pos -> AMP whole body\n"
                  << "  J: idle/damping/zero pos/AMP zero pos/walking -> walking zero pos\n"
                  << "  L: walking zero pos -> AMP walking; R: emergency damping\n"
                  << "  W/S: forward step 0.05 (holds); A/D: side; Q/E: turn (release timeout 1s)\n"
                  << "  X: clear velocity; 1..6: arm preset\n";
        StartEventLoop(true);
    }
    void Stop() override {
        running_.store(false, std::memory_order_release);
        if (event_thread_.joinable()) event_thread_.join();
    }

    // Device channels are opt-in; constructing this class is safe for offline tests.
    void EnableDdsInput(bool enabled = true) override {
#ifndef MOTION_MASTER_COMMANDS_OFFLINE
        if (enabled) {
            if (gamepad_key_sub_) return;
            gamepad_key_sub_ = std::make_unique<ChannelString>(
                [this](const std_msgs::msg::String* message) { ProcessGamepadKey(message->data()); },
                "/GAMEPAD_KEY", 0);
            steer_sub_ = std::make_unique<ChannelSteer>(
                [this](const drdds::msg::Steer* message) {
                    ProcessSteer(message->data().x(), message->data().y(), message->data().yaw());
                }, "/STEER", 0);
            real_steer_sub_ = std::make_unique<ChannelSteer>(
                [this](const drdds::msg::Steer* message) {
                    ProcessRealSteer(message->data().x(), message->data().y(), message->data().yaw());
                }, "/REAL_STEER", 0);
            neck_steer_sub_ = std::make_unique<ChannelSteer>(
                [this](const drdds::msg::Steer* message) {
                    ProcessNeckSteer(message->data().yaw(), message->data().pitch());
                }, "/NECK_STEER", 0);
        } else {
            gamepad_key_sub_.reset();
            steer_sub_.reset();
            real_steer_sub_.reset();
            neck_steer_sub_.reset();
        }
#else
        (void)enabled;
#endif
    }

    virtual void ProcessKey(char input, double current_time_ms) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        usr_cmd_->time_stamp = current_time_ms;
        const RobotMotionState state = GetCurrentState();
        if (PollRemoteCommands(current_time_ms) || EmergencyDampingPending()) {
            ClearVelocity();
            return;
        }
        if (input != '\0') {
            const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
            HandleKeyboardModeKey(key, state);
            if (key >= '1' && key <= '6') SetArmPreset(key - '0');
            if (IsKeyboardVelocityState(state)) {
                if (key == 'w' || key == 's') {
                    if (current_time_ms - forward_step_last_input_ms_ > 120.0) {
                        usr_cmd_->forward_vel_scale += key == 'w' ? 0.05f : -0.05f;
                        forward_step_last_input_ms_ = current_time_ms;
                    }
                } else if (key == 'x') {
                    ClearVelocity();
                    side_active_ = turn_active_ = false;
                }
                const bool rl = state == RobotMotionState::RLStandControlMode ||
                                state == RobotMotionState::AmpWholeBodyControlMode;
                if (key == 'a' || key == 'd') {
                    usr_cmd_->side_vel_scale = (key == 'a' ? 1.0f : -1.0f) * (rl ? 0.70f : 0.5f);
                    side_last_input_ms_ = current_time_ms;
                    side_active_ = true;
                }
                if (key == 'q' || key == 'e') {
                    usr_cmd_->turnning_vel_scale = (key == 'q' ? 1.0f : -1.0f) * (rl ? 0.70f : 1.57f);
                    turn_last_input_ms_ = current_time_ms;
                    turn_active_ = true;
                }
            }
        }
        if (EmergencyDampingPending()) {
            ClearVelocity();
            side_active_ = turn_active_ = false;
            return;
        }
        if (IsKeyboardVelocityState(state)) {
            if (side_active_ && current_time_ms - side_last_input_ms_ > 1000.0) {
                usr_cmd_->side_vel_scale = 0.0f;
                side_active_ = false;
            }
            if (turn_active_ && current_time_ms - turn_last_input_ms_ > 1000.0) {
                usr_cmd_->turnning_vel_scale = 0.0f;
                turn_active_ = false;
            }
            ClipKeyboardVelocityCommand(state);
        } else {
            ClearVelocity();
            side_active_ = turn_active_ = false;
        }
    }

    void ProcessGamepadKey(const std::string& key) {
        KeyCode code = KeyCode::UNKNOWN;
        if (key == "G20_KEY_L1") code = KeyCode::L1;
        else if (key == "G20_KEY_L2") code = KeyCode::L2;
        else if (key == "G20_KEY_R1") code = KeyCode::R1;
        else if (key == "G20_KEY_R2") code = KeyCode::R2;
        ProcessGamepadKey(code);
    }
    void ProcessGamepadKey(KeyCode key) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        usr_cmd_->disable_arm_motion_planning = true;
        const RobotMotionState state = GetCurrentState();
        switch (key) {
        case KeyCode::L1:
            if (control_mode_ == ControlMode::kWholeBody) {
                HandleKeyboardModeKey('z', state);
            } else if (state == RobotMotionState::Idle || state == RobotMotionState::UpperBodyControlMode) {
                SetTargetMode(RobotMotionState::ZeroPos, "Zero Pos");
            }
            break;
        case KeyCode::L2:
            if (control_mode_ == ControlMode::kWholeBody) {
                if (state == RobotMotionState::AmpWalkingZeroPos)
                    HandleKeyboardModeKey('l', state);
                else if (state == RobotMotionState::RLControlMode ||
                         state == RobotMotionState::RLStandControlMode || state == RobotMotionState::AmpMoe15Dof ||
                         state == RobotMotionState::AmpWalkingWAQ || state == RobotMotionState::RLStandSaluteMode ||
                         state == RobotMotionState::RLStandSaluteLeftArmMode)
                    HandleKeyboardModeKey('m', state);
                else if (state == RobotMotionState::AmpZeroPos)
                    HandleKeyboardModeKey('c', state);
            } else if (state == RobotMotionState::ZeroPos) {
                SetTargetMode(RobotMotionState::UpperBodyControlMode, "Upper Body Control");
            }
            break;
        case KeyCode::R1:
            if (control_mode_ == ControlMode::kWholeBody)
                HandleKeyboardModeKey('v', state);
            break;
        case KeyCode::R2:
            if (control_mode_ == ControlMode::kWholeBody)
                HandleKeyboardModeKey('p', state);
            break;
        default:
            break;
        }
    }
    virtual void ProcessSteer(float forward, float side, float turn) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (EmergencyDampingPending()) { ClearVelocity(); return; }
        usr_cmd_->forward_vel_scale = forward;
        usr_cmd_->side_vel_scale = side;
        usr_cmd_->turnning_vel_scale = turn;
    }
    virtual void ProcessRealSteer(float forward, float side, float turn) {
        ProcessSteer(std::clamp(forward, -0.4f, 0.8f), std::clamp(side / 0.75f, -0.5f, 0.5f),
                     std::clamp(turn / 2.25f, -1.0f, 1.0f));
    }
    void ProcessNeckSteer(float yaw, float pitch) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        usr_cmd_->neck_yaw_cmd = yaw;
        usr_cmd_->neck_pitch_cmd = pitch;
    }
    bool PollRemoteCommands(double current_time_ms) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (!udp_remote_) return false;
        usr_cmd_->time_stamp = current_time_ms;
        const RobotMotionState state = GetCurrentState();
        if (udp_remote_->hasPendingEstop()) {
            udp_remote_->consumePendingEstop();
            SetTargetMode(RobotMotionState::JointDamping, "[GUI] Emergency Damping");
            ClearVelocity();
            if (udp_remote_->hasPendingKey()) udp_remote_->consumePendingKey();
            if (udp_remote_->hasPendingVelocity()) {
                float forward, side, turn;
                udp_remote_->consumePendingVelocity(forward, side, turn);
            }
            return true;
        }
        if (udp_remote_->hasPendingKey())
            HandleKeyboardModeKey(udp_remote_->consumePendingKey(), state);
        if (udp_remote_->hasPendingVelocity()) {
            float forward, side, turn;
            udp_remote_->consumePendingVelocity(forward, side, turn);
            ProcessSteer(forward, side, turn);
        }
        return false;
    }

protected:
    void StartEventLoop(bool terminal_input) {
        Stop();
        {
            std::lock_guard<std::recursive_mutex> lock(command_mutex_);
            ResetUserCommand();
            forward_step_last_input_ms_ = -1e9;
            side_active_ = turn_active_ = false;
        }
        running_.store(true, std::memory_order_release);
        event_thread_ = std::thread([this, terminal_input] { EventLoop(terminal_input); });
    }
    RobotMotionState GetCurrentState() const {
        if (motion_state_store_) return motion_state_store_->GetState();
        return msfb_ ? static_cast<RobotMotionState>(msfb_->GetCurrentState()) : RobotMotionState::Idle;
    }

private:
    static void ClipNumber(float& value, float lower, float upper) {
        value = std::clamp(value, lower, upper);
    }
    void ClearVelocity() {
        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }
    void SetTargetMode(RobotMotionState state, const char* name) {
        if (!SetTargetModeRequest(static_cast<uint32_t>(state))) return;
        if (name) std::cout << "[MODE] " << name << std::endl;
    }
  bool IsKeyboardVelocityState(RobotMotionState state) const {
    return state == RobotMotionState::RLControlMode ||
           state == RobotMotionState::RLStandControlMode ||
           state == RobotMotionState::AmpMoe15Dof ||
           state == RobotMotionState::AmpWalkingWAQ ||
           state == RobotMotionState::AmpWholeBodyControlMode ||
           state == RobotMotionState::AmpWalkingControl;
  }

  void ClipKeyboardVelocityCommand(RobotMotionState state) {
    if (state == RobotMotionState::AmpWalkingControl) {
      ClipNumber(usr_cmd_->forward_vel_scale, -0.6f, 1.5f);
      ClipNumber(usr_cmd_->side_vel_scale, -0.5f, 0.5f);
      ClipNumber(usr_cmd_->turnning_vel_scale, -1.57f, 1.57f);
      return;
    }

    ClipNumber(usr_cmd_->forward_vel_scale, -0.8f, 0.8f);
    ClipNumber(usr_cmd_->side_vel_scale, -0.8f, 0.8f);
    ClipNumber(usr_cmd_->turnning_vel_scale, -0.8f, 0.8f);
  }

  void HandleKeyboardModeKey(char key, RobotMotionState current_state) {
    if (control_mode_ == ControlMode::kWholeBody) {
      if (key == 'u' && (current_state == RobotMotionState::RLStandControlMode ||
                         current_state == RobotMotionState::AmpWalkingControl)) {
        ClearVelocity();
        SetTargetMode(RobotMotionState::RLStandSaluteMode, "RL Stand Salute");
        return;
      }
      if (key == 't' && current_state == RobotMotionState::AmpWalkingControl) {
        ClearVelocity();
        SetTargetMode(RobotMotionState::Stand2KneeMimicMode, "Stand to Knee Mimic");
        return;
      }
      if (key == 'y' && current_state == RobotMotionState::Stand2KneeMimicMode) {
        ClearVelocity();
        SetTargetMode(RobotMotionState::Knee2StandMimicMode, "Knee to Stand Mimic");
        return;
      }
      if (key == 'b' && (current_state == RobotMotionState::RLStandControlMode ||
                         current_state == RobotMotionState::AmpWalkingControl)) {
        ClearVelocity();
        SetTargetMode(RobotMotionState::RLControlMode, "AMP Control");
        return;
      }
      if (key == 'n' && current_state == RobotMotionState::RLControlMode) {
        ClearVelocity();
        SetTargetMode(RobotMotionState::RLControlMimic12Mode, "Mimic12 Stand to Sit");
        return;
      }
      if (key == 'm' && current_state == RobotMotionState::RLControlMimic12Mode) {
        SetTargetMode(RobotMotionState::RLControlMimic13Mode, "Mimic13 Sit to Stand");
        return;
      }
      if (key == 'm' && current_state == RobotMotionState::RLControlMimic13Mode) {
        SetTargetMode(RobotMotionState::AmpWalkingControl, "AMP Walking Control");
        return;
      }
      // Dance policies toggle with AMP walking only.
      if (key == 'g' && (current_state == RobotMotionState::AmpWalkingControl ||
                         current_state == RobotMotionState::RLControlMimic1Mode)) {
        ClearVelocity();
        const bool to_walking = current_state == RobotMotionState::RLControlMimic1Mode;
        SetTargetMode(to_walking ? RobotMotionState::AmpWalkingControl
                                 : RobotMotionState::RLControlMimic1Mode,
                      to_walking ? "AMP Walking Control" : "Mimic1 Dance");
        return;
      }
      if (key == 'k' && (current_state == RobotMotionState::AmpWalkingControl ||
                         current_state == RobotMotionState::RLControlMimic2Mode)) {
        ClearVelocity();
        const bool to_walking = current_state == RobotMotionState::RLControlMimic2Mode;
        SetTargetMode(to_walking ? RobotMotionState::AmpWalkingControl
                                 : RobotMotionState::RLControlMimic2Mode,
                      to_walking ? "AMP Walking Control" : "Mimic2 Dance");
        return;
      }
    }
    if (key == 'o') {
      if (control_mode_ == ControlMode::kWholeBody &&
          (current_state == RobotMotionState::AmpZeroPos ||
           current_state == RobotMotionState::AmpWalkingZeroPos ||
           current_state == RobotMotionState::RLStandControlMode ||
           current_state == RobotMotionState::AmpWalkingControl ||
           current_state == RobotMotionState::AmpMoe15Dof))
        SetTargetMode(RobotMotionState::AmpWalkingWAQ, "AMP Walking WAQ");
      return;
    }
    if (key == 'p') {
      if (control_mode_ == ControlMode::kWholeBody && current_state == RobotMotionState::RLStandControlMode)
        SetTargetMode(RobotMotionState::AmpMoe15Dof, "AMP MoE 15 DOF");
      return;
    }
    if (key == 'r') {
      SetTargetMode(RobotMotionState::JointDamping, "Joint Damping");
      return;
    }

    if (key == 'z') {
      if (current_state == RobotMotionState::Idle ||
          current_state == RobotMotionState::JointDamping ||
          current_state == RobotMotionState::ZeroPos) {
        SetTargetMode(RobotMotionState::AmpZeroPos, "AMP Zero Pos");
      }
      return;
    }

    if (key == 'c') {
      if (current_state == RobotMotionState::AmpZeroPos ||
          (control_mode_ == ControlMode::kWholeBody &&
           (current_state == RobotMotionState::Knee2StandMimicMode ||
            current_state == RobotMotionState::RLStandSaluteMode ||
            current_state == RobotMotionState::RLStandSaluteLeftArmMode))) {
        ClearVelocity();
        SetTargetMode(RobotMotionState::AmpWalkingControl, "AMP Walking Control");
      }
      return;
    }

    if (key == 'v') {
      if (current_state == RobotMotionState::RLControlMode || current_state == RobotMotionState::AmpWalkingControl || current_state == RobotMotionState::AmpMoe15Dof ||
          current_state == RobotMotionState::AmpWalkingWAQ) {
        SetTargetMode(RobotMotionState::RLStandControlMode, "RL Stand Control");
      }
      return;
    }

    if (key == 'm') {
      if (current_state == RobotMotionState::RLControlMode || current_state == RobotMotionState::RLStandControlMode || current_state == RobotMotionState::AmpMoe15Dof ||
          current_state == RobotMotionState::AmpWalkingWAQ ||
          (control_mode_ == ControlMode::kWholeBody &&
           (current_state == RobotMotionState::RLStandSaluteMode || current_state == RobotMotionState::RLStandSaluteLeftArmMode))) {
        if (current_state == RobotMotionState::RLStandSaluteMode || current_state == RobotMotionState::RLStandSaluteLeftArmMode) ClearVelocity();
        SetTargetMode(RobotMotionState::AmpWalkingControl, "AMP Walking Control");
      }
      return;
    }

    if (key == 'f') {
      if (current_state == RobotMotionState::Idle ||
          current_state == RobotMotionState::JointDamping ||
          current_state == RobotMotionState::AmpWholeBodyControlMode) {
        SetTargetMode(RobotMotionState::AmpZeroPos, "AMP Zero Pos");
      }
      return;
    }

    if (key == 'h') {
      if (current_state == RobotMotionState::AmpZeroPos) {
        SetTargetMode(RobotMotionState::AmpWholeBodyControlMode, "AMP Whole Body Control");
      }
      return;
    }

    if (key == 'j') {
      if (current_state == RobotMotionState::Idle ||
          current_state == RobotMotionState::JointDamping ||
          current_state == RobotMotionState::ZeroPos ||
          current_state == RobotMotionState::AmpZeroPos ||
          current_state == RobotMotionState::AmpWalkingControl) {
        SetTargetMode(RobotMotionState::AmpWalkingZeroPos, "AMP Walking Zero Pos");
      }
      return;
    }

    if (key == 'l') {
      if (current_state == RobotMotionState::AmpWalkingZeroPos) {
        SetTargetMode(RobotMotionState::AmpWalkingControl, "AMP Walking Control");
      }
      return;
    }

  }

  void SetArmPreset(int preset) {
    usr_cmd_->disable_arm_motion_planning = false;
    usr_cmd_->arm_preset = preset;
    ++usr_cmd_->arm_preset_seq;
    usr_cmd_->arm_preset_1 = preset == 1;
    usr_cmd_->arm_preset_2 = preset == 2;
    usr_cmd_->arm_preset_3 = preset == 3;
    usr_cmd_->arm_preset_4 = preset == 4;
    usr_cmd_->arm_preset_5 = preset == 5;
    usr_cmd_->arm_preset_6 = preset == 6;
  }

    void EventLoop(bool terminal_input) {
        termios original{}, raw{};
        const int original_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        const bool terminal_ready = terminal_input && tcgetattr(STDIN_FILENO, &original) == 0;
        if (terminal_ready) {
            raw = original;
            raw.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
        }
        const auto start = std::chrono::steady_clock::now();
        while (running_.load(std::memory_order_acquire)) {
            char input = '\0';
            if (terminal_ready && read(STDIN_FILENO, &input, 1) != 1) input = '\0';
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            ProcessKey(input, elapsed_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (terminal_ready) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original);
            if (original_flags >= 0) fcntl(STDIN_FILENO, F_SETFL, original_flags);
        }
    }
    std::atomic<bool> running_{false};
    std::thread event_thread_;
    double forward_step_last_input_ms_ = -1e9;
    double side_last_input_ms_ = -1e9;
    double turn_last_input_ms_ = -1e9;
    bool side_active_ = false;
    bool turn_active_ = false;
#ifndef MOTION_MASTER_COMMANDS_OFFLINE
    std::unique_ptr<ChannelString> gamepad_key_sub_;
    std::unique_ptr<ChannelSteer> steer_sub_, real_steer_sub_, neck_steer_sub_;
#endif
};
#endif
