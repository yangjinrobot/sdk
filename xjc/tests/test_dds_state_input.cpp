#include "dds_interface.hpp"
#include <stdexcept>

static void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
static int SendState(uint32_t state) {
    return ChannelMotionState::Publish(drdds::msg::MotionState{{state}}, "/MOTION_STATE");
}
template <typename Input>
static void CheckSubscriptions(ControlMode mode) {
    Input input(RobotName::CR1PRO, mode);
    auto store = std::make_shared<MotionStateStore>();
    input.SetMotionStateStore(store);
    for (int entry = 0; entry < 2; ++entry) {
        input.EnableDdsInput();
        input.EnableDdsInput(); // Repeated setup must not create duplicate callbacks.
        input.Start();
        store->SetState(RobotMotionState::AmpWalkingControl);
        Require(SendState(0x1100b) == 1, "motion_1 has no /MOTION_STATE subscriber in this command mode");
        Require(input.GetUserCommandSnapshot().target_mode == RobotMotionState::RLStandControlMode,
                "motion_1 did not request RL Stand from AMP Walking");
        // A subsequent joystick packet/event-loop tick must retain the request.
        ChannelSteer::Publish(drdds::msg::Steer{{.2f, .1f, 0.f, 0.f}}, "/STEER");
        input.ProcessKey('\0', 50.0);
        Require(input.GetUserCommandSnapshot().target_mode == RobotMotionState::RLStandControlMode,
                "steering or the input loop overwrote motion_1");
        store->SetState(RobotMotionState::RLStandControlMode);
        SendState(0x11);
        Require(input.GetUserCommandSnapshot().target_mode == RobotMotionState::AmpWalkingControl,
                "DDS return to AMP Walking failed");
        for (uint32_t state : {0x1100cu, 0x1100du, 0x1100eu, 0x1100fu, 0x11010u, 0x11011u, 0x11012u}) {
            Require(SendState(state) == 1 && input.GetUserCommandSnapshot().target_mode == state,
                    "the shared DDS channel did not accept an enabled motion state");
        }
        SendState(0x9);
        Require(input.GetUserCommandSnapshot().target_mode == RobotMotionState::Knee2StandMimicMode,
                "a retired protocol ID replaced the last accepted request");
        Require(ChannelMotionState::Publish(drdds::msg::MotionState{{0x1100b}}, "/MOTION_STATE", 1) == 0,
                "motion-state subscriber accepted the wrong domain");
        SendState(0x2);
        SendState(0x1100b);
        Require(input.GetUserCommandSnapshot().target_mode == RobotMotionState::JointDamping,
                "motion_1 bypassed an unacknowledged emergency damping request");
        store->SetState(RobotMotionState::JointDamping);
        SendState(0x20006);
        Require(input.GetUserCommandSnapshot().target_mode == RobotMotionState::AmpZeroPos,
                "acknowledged damping did not release later requests");
        input.Stop();
        input.EnableDdsInput(false);
        Require(SendState(0x1100b) == 0, "disabled DDS input retained a motion-state subscription");
        Require(ChannelString::Publish(std_msgs::msg::String{"G20_KEY_R1"}, "/GAMEPAD_KEY") == 0,
                "disabled DDS input retained a gamepad subscription");
    }
}
int main() {
    try {
        for (const auto mode : {ControlMode::kWholeBody, ControlMode::kUpperBody}) {
            CheckSubscriptions<KeyboardInterface>(mode);
            CheckSubscriptions<DdsInterface>(mode);
        }
        std::cout << "Shared DDS subscriptions, enabled motion states and emergency priority passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
