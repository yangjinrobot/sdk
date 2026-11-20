#ifndef RETROID_GAMEPAD_INTERFACE_HPP_
#define RETROID_GAMEPAD_INTERFACE_HPP_
#include "retroid_gamepad.h"
#include "keyboard_interface.hpp"

class RetroidGamepadInterface : public KeyboardInterface {
public:
    explicit RetroidGamepadInterface(int port, RobotName robot = RobotName::CR1PRO,
                                    ControlMode mode = ControlMode::kWholeBody)
        : KeyboardInterface(robot, mode), gamepad_ptr_(std::make_shared<RetroidGamepad>(port)) {}
    ~RetroidGamepadInterface() override { Stop(); }
    void Start() override {
        Stop();
        ResetUserCommand();
        first_frame_ = true;
        transform_running_.store(true);
        gamepad_ptr_->StartDataThread();
        transform_thread_ = std::thread(&RetroidGamepadInterface::TransformRetroidToUserCommand, this);
    }
    void Stop() override {
        if (transform_running_.exchange(false)) gamepad_ptr_->StopDataThread();
        if (transform_thread_.joinable()) transform_thread_.join();
        KeyboardInterface::Stop();
    }
    void TransformRetroidToUserCommand() {
        while (transform_running_.load()) {
            const RetroidKeys keys = gamepad_ptr_->GetKeys();
            if (first_frame_) {
                previous_ = keys;
                first_frame_ = false;
            } else {
                ProcessSteer(keys.left_axis_y, -keys.left_axis_x, -keys.right_axis_x);
                if (keys.L1 && !previous_.L1) ProcessGamepadKey(KeyCode::L1);
                if (keys.L2 && !previous_.L2) ProcessGamepadKey(KeyCode::L2);
                if (keys.R1 && !previous_.R1) ProcessGamepadKey(KeyCode::R1);
                if (keys.R2 && !previous_.R2) ProcessGamepadKey(KeyCode::R2);
                ProcessKey('\0', keys.time_stamp);
                previous_ = keys;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
private:
    std::shared_ptr<RetroidGamepad> gamepad_ptr_;
    RetroidKeys previous_{};
    bool first_frame_ = true;
    std::atomic<bool> transform_running_{false};
    std::thread transform_thread_;
};
#endif
