#ifndef SKYDROID_GAMEPAD_INTERFACE_HPP_
#define SKYDROID_GAMEPAD_INTERFACE_HPP_
#include "skydroid_gamepad.h"
#include "keyboard_interface.hpp"

class SkydroidGamepadInterface : public KeyboardInterface {
public:
    explicit SkydroidGamepadInterface(int port, RobotName robot = RobotName::CR1PRO,
                                     ControlMode mode = ControlMode::kWholeBody)
        : KeyboardInterface(robot, mode), gamepad_ptr_(std::make_shared<SkydroidGamepad>(port)) {}
    ~SkydroidGamepadInterface() override { Stop(); }
    void Start() override {
        Stop();
        ResetUserCommand();
        first_frame_ = true;
        transform_running_.store(true);
        gamepad_ptr_->StartDataThread();
        transform_thread_ = std::thread(&SkydroidGamepadInterface::TransformSkydroidToUserCommand, this);
    }
    void Stop() override {
        if (transform_running_.exchange(false)) gamepad_ptr_->StopDataThread();
        if (transform_thread_.joinable()) transform_thread_.join();
        KeyboardInterface::Stop();
    }
    void TransformSkydroidToUserCommand() {
        while (transform_running_.load()) {
            const SkydroidKeys keys = gamepad_ptr_->GetKeys();
            if (first_frame_) {
                previous_ = keys;
                first_frame_ = false;
            } else {
                ProcessSteer(keys.left_axis_y, -keys.left_axis_x, -keys.right_axis_x);
                // A/B map to L1/L2; retain the dedicated damping switch chord.
                if (keys.A && !previous_.A) ProcessGamepadKey(KeyCode::L1);
                if (keys.B && !previous_.B) ProcessGamepadKey(KeyCode::L2);
                const bool damping_chord = keys.sw1 != 0 && keys.sw4 != 0;
                const bool previous_chord = previous_.sw1 != 0 && previous_.sw4 != 0;
                if (damping_chord && !previous_chord) ProcessKey('r', keys.time_stamp);
                ProcessKey('\0', keys.time_stamp);
                previous_ = keys;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
private:
    std::shared_ptr<SkydroidGamepad> gamepad_ptr_;
    SkydroidKeys previous_{};
    bool first_frame_ = true;
    std::atomic<bool> transform_running_{false};
    std::thread transform_thread_;
};
#endif
