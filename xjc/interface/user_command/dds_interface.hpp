#ifndef DDS_INTERFACE_HPP_
#define DDS_INTERFACE_HPP_
#include "keyboard_interface.hpp"

class DdsInterface : public KeyboardInterface {
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_steer_{};
    bool have_steer_ = false;

    bool CanSteer() const {
        const auto state = GetCurrentState();
        return IsKnownRobotMotionState(state) && (state == RobotMotionState::RLControlMode ||
               state == RobotMotionState::AmpWalkingControl ||
               state == RobotMotionState::AmpWalkingWAQ ||
               state == RobotMotionState::RLStandControlMode ||
               state == RobotMotionState::AmpMoe15Dof ||
               state == RobotMotionState::AmpWholeBodyControlMode);
    }
public:
    explicit DdsInterface(RobotName robot = RobotName::CR1PRO,
                          ControlMode mode = ControlMode::kWholeBody)
        : KeyboardInterface(robot, mode) {}
    ~DdsInterface() override {
        Stop();
        EnableDdsInput(false);
    }
    // The controller sends sparse /MOTION_STATE codes, and /STEER or
    // /REAL_STEER velocity packets. Keep the existing gamepad-key/GUI path too.
    void Start() override {
        {
            std::lock_guard<std::recursive_mutex> lock(command_mutex_);
            have_steer_ = false;
        }
        StartEventLoop(false);
        ProcessGait(RobotGaitType::HumanWALK);
    }
    void EnableDdsInput(bool enabled = true) override {
        KeyboardInterface::EnableDdsInput(enabled);
#ifndef MOTION_MASTER_COMMANDS_OFFLINE
        if (enabled && !gait_sub_) {
            gait_sub_ = std::make_unique<ChannelGait>(
                [this](const drdds::msg::Gait* message) { ProcessGait(message->data().gait()); },
                "/GAIT", 0);
        } else if (!enabled) {
            gait_sub_.reset();
        }
#endif
    }
    static bool IsStateAllowedByCert(uint32_t state) { return IsKnownRobotMotionState(state); }
    static bool IsGaitAllowedByCert(uint32_t gait) { return gait == RobotGaitType::HumanWALK; }
    static bool IsActionAllowedByCert(int32_t) { return false; }
    bool ProcessGait(uint32_t gait) {
        if (!IsGaitAllowedByCert(gait)) return false;
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        SetTargetGait(gait);
        if (msfb_) msfb_->UpdateCurrentGait(gait);
        return true;
    }
    void ProcessSteer(float forward, float side, float turn) override {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (!CanSteer() || !std::isfinite(forward) || !std::isfinite(side) || !std::isfinite(turn))
            forward = side = turn = 0.f;
        KeyboardInterface::ProcessSteer(forward, side, turn);
        last_steer_ = Clock::now();
        have_steer_ = true;
    }
    void ProcessRealSteer(float forward, float side, float turn) override {
        ProcessSteer(std::clamp(forward, -1.f, 1.f), std::clamp(side / .75f, -1.f, 1.f),
                     std::clamp(turn / 2.25f, -1.f, 1.f));
    }
    void ExpireSteerCommand(Clock::time_point now = Clock::now()) {
        std::lock_guard<std::recursive_mutex> lock(command_mutex_);
        if (!CanSteer() || (have_steer_ && now - last_steer_ > std::chrono::seconds(2))) {
            KeyboardInterface::ProcessSteer(0.f, 0.f, 0.f);
            have_steer_ = false;
        }
    }
    void ProcessKey(char input, double current_time_ms) override {
        KeyboardInterface::ProcessKey(input, current_time_ms);
        ExpireSteerCommand();
    }
#ifndef MOTION_MASTER_COMMANDS_OFFLINE
    void HandlerState(const drdds::msg::MotionState* data) { ProcessMotionState(data->data().state()); }
    void HandlerGait(const drdds::msg::Gait* data) { ProcessGait(data->data().gait()); }
    void HandlerSteer(const drdds::msg::Steer* data) {
        ProcessSteer(data->data().x(), data->data().y(), data->data().yaw());
    }
    void HandlerVel(const drdds::msg::Steer* data) {
        ProcessRealSteer(data->data().x(), data->data().y(), data->data().yaw());
    }
    void HandlerNeckSteer(const drdds::msg::Steer* data) {
        ProcessNeckSteer(data->data().yaw(), data->data().pitch());
    }
private:
    std::unique_ptr<ChannelGait> gait_sub_;
#endif
};
#endif
