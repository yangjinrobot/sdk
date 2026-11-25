#include "keyboard_interface.hpp"
#include "dds_interface.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

using namespace types;
namespace {
constexpr uint32_t kUnchanged = 99;
void Near(float actual, float expected) { assert(std::fabs(actual - expected) < 1e-5f); }
struct Transition { char key; RobotMotionState from; RobotMotionState to; };
constexpr Transition transitions[] = {
    {'u', AmpWalkingControl, RLStandSaluteMode}, {'u', RLStandControlMode, RLStandSaluteMode},
    {'m', RLStandSaluteMode, AmpWalkingControl}, {'c', RLStandSaluteMode, AmpWalkingControl},
    {'m', RLStandSaluteLeftArmMode, AmpWalkingControl}, {'c', RLStandSaluteLeftArmMode, AmpWalkingControl},
    {'t', AmpWalkingControl, Stand2KneeMimicMode},
    {'y', Stand2KneeMimicMode, Knee2StandMimicMode},
    {'c', Knee2StandMimicMode, AmpWalkingControl},
    {'o', AmpZeroPos, AmpWalkingWAQ}, {'o', AmpWalkingZeroPos, AmpWalkingWAQ},
    {'o', RLStandControlMode, AmpWalkingWAQ}, {'o', AmpWalkingControl, AmpWalkingWAQ},
    {'o', AmpMoe15Dof, AmpWalkingWAQ}, {'v', AmpWalkingWAQ, RLStandControlMode},
    {'m', AmpWalkingWAQ, AmpWalkingControl},
    {'b', RLStandControlMode, RLControlMode}, {'b', AmpWalkingControl, RLControlMode},
    {'b', RLControlMimic13Mode, RLControlMode}, {'n', RLControlMode, RLControlMimic12Mode},
    {'m', RLControlMimic12Mode, RLControlMimic13Mode},
    {'v', RLControlMode, RLStandControlMode}, {'m', RLControlMode, AmpWalkingControl},
    {'z', Idle, AmpZeroPos}, {'z', JointDamping, AmpZeroPos}, {'z', ZeroPos, AmpZeroPos},
    {'p', RLStandControlMode, AmpMoe15Dof}, {'v', AmpMoe15Dof, RLStandControlMode},
    {'m', AmpMoe15Dof, AmpWalkingControl},
    {'c', AmpZeroPos, AmpWalkingControl}, {'v', AmpWalkingControl, RLStandControlMode},
    {'c', RLStandControlMode, AmpWalkingControl}, {'m', RLStandControlMode, AmpWalkingControl},
    {'f', Idle, AmpZeroPos}, {'f', JointDamping, AmpZeroPos}, {'f', AmpWholeBodyControlMode, AmpZeroPos},
    {'h', AmpZeroPos, AmpWholeBodyControlMode},
    {'j', Idle, AmpWalkingZeroPos}, {'j', JointDamping, AmpWalkingZeroPos},
    {'j', ZeroPos, AmpWalkingZeroPos}, {'j', AmpZeroPos, AmpWalkingZeroPos},
    {'j', AmpWalkingControl, AmpWalkingZeroPos}, {'l', AmpWalkingZeroPos, AmpWalkingControl}
};
void CheckKeyboardAndGamepad() {
    for (ControlMode mode : {kWholeBody, kUpperBody}) {
        KeyboardInterface input(CR1PRO, mode);
        auto store = std::make_shared<MotionStateStore>();
        input.SetMotionStateStore(store);
        for (const auto state : kRobotMotionStates) {
            store->SetState(state);
            for (char key : std::string("rzcvbmngufhtyjlpo")) {
                input.ResetUserCommand();
                input.UpdateCommand([](UserCommand& command) { command.target_mode = kUnchanged; });
                uint32_t expected = kUnchanged;
                if (key == 'r') expected = JointDamping;
                for (const auto& transition : transitions)
                    if (transition.key == key && transition.from == state) expected = transition.to;
                if (mode == kUpperBody && (key == 'p' || key == 'o' || key == 'b' || key == 'n' || key == 't' || key == 'y' || key == 'u' ||
                    ((key == 'm' || key == 'c') && (state == RLStandSaluteMode || state == RLStandSaluteLeftArmMode)) ||
                    (key == 'c' && state == Knee2StandMimicMode) ||
                    (key == 'm' && state == RLControlMimic12Mode))) expected = kUnchanged;
                input.ProcessKey(key, 100.0);
                assert(input.GetUserCommandSnapshot().target_mode == expected);
            }
            for (const std::string key : {"G20_KEY_L1", "G20_KEY_L2", "G20_KEY_R1", "G20_KEY_R2", "UNKNOWN"}) {
                input.UpdateCommand([](UserCommand& command) {
                    command.target_mode = kUnchanged;
                    command.disable_arm_motion_planning = false;
                });
                uint32_t expected = kUnchanged;
                if (mode == kWholeBody) {
                    if (key == "G20_KEY_L1" && (state == Idle || state == JointDamping || state == ZeroPos))
                        expected = AmpZeroPos;
                    if (key == "G20_KEY_L2" &&
                        (state == AmpZeroPos || state == AmpWalkingZeroPos || state == RLStandControlMode || state == AmpMoe15Dof || state == AmpWalkingWAQ || state == RLControlMode || state == RLStandSaluteMode || state == RLStandSaluteLeftArmMode))
                        expected = AmpWalkingControl;
                    if (key == "G20_KEY_R1" && (state == AmpWalkingControl || state == AmpMoe15Dof || state == AmpWalkingWAQ || state == RLControlMode)) expected = RLStandControlMode;
                } else {
                    if (key == "G20_KEY_L1" && (state == Idle || state == UpperBodyControlMode)) expected = ZeroPos;
                    if (key == "G20_KEY_L2" && state == ZeroPos) expected = UpperBodyControlMode;
                }
                if (key == "G20_KEY_R2" && mode == kWholeBody && state == RLStandControlMode) expected = AmpMoe15Dof;
                input.ProcessGamepadKey(key);
                const auto command = input.GetUserCommandSnapshot();
                assert(command.target_mode == expected);
                assert(command.disable_arm_motion_planning);
                // Retroid enum events and DDS string events share the same mapping.
                input.UpdateCommand([](UserCommand& command) { command.target_mode = kUnchanged; });
                const KeyCode code = key == "G20_KEY_L1" ? KeyCode::L1 :
                                     key == "G20_KEY_L2" ? KeyCode::L2 :
                                     key == "G20_KEY_R1" ? KeyCode::R1 :
                                     key == "G20_KEY_R2" ? KeyCode::R2 : KeyCode::UNKNOWN;
                input.ProcessGamepadKey(code);
                assert(input.GetUserCommandSnapshot().target_mode == expected);
            }
        }
        for (int preset = 1; preset <= 6; ++preset) {
            input.ProcessKey(static_cast<char>('0' + preset), preset * 200.0);
            const auto command = input.GetUserCommandSnapshot();
            assert(command.arm_preset == preset && command.arm_preset_seq == static_cast<uint32_t>(preset));
            assert(!command.disable_arm_motion_planning);
            const bool enabled[] = {command.arm_preset_1, command.arm_preset_2, command.arm_preset_3,
                                    command.arm_preset_4, command.arm_preset_5, command.arm_preset_6};
            for (int i = 0; i < 6; ++i) assert(enabled[i] == (i + 1 == preset));
        }
    }
}
void CheckEmergencyPriority() {
    DdsInterface input;
    auto store = std::make_shared<MotionStateStore>();
    input.SetMotionStateStore(store);
    store->SetState(RLStandControlMode);
    UdpNode remote("127.0.0.1", 8888, "127.0.0.1", 9999);  // No sockets started.
    input.SetUdpRemote(&remote);
    input.ProcessSteer(.5f, .3f, .2f);
    remote.queueKey('p');
    remote.queueVelocity(.8f, .5f, .4f);
    remote.queueEstop();
    input.ProcessKey('m', 100);
    auto command = input.GetUserCommandSnapshot();
    assert(command.target_mode == JointDamping);
    Near(command.forward_vel_scale, 0); Near(command.side_vel_scale, 0); Near(command.turnning_vel_scale, 0);
    assert(!remote.hasPendingKey() && !remote.hasPendingVelocity() && !remote.hasPendingEstop());
    input.ProcessGamepadKey(KeyCode::R2);
    assert(!input.ProcessMotionState(AmpMoe15Dof));
    input.ProcessSteer(.5f, .5f, .5f);
    input.ProcessKey('p', 110);
    assert(input.GetUserCommandSnapshot().target_mode == JointDamping);
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0);
    // Release only after the FSM reports that it applied the damping request.
    store->SetState(JointDamping);
    input.ProcessKey('z', 120);
    assert(input.GetUserCommandSnapshot().target_mode == AmpZeroPos);
    store->SetState(AmpWalkingControl);
    input.ProcessKey('r', 130);
    input.ProcessKey('v', 140);
    assert(input.GetUserCommandSnapshot().target_mode == JointDamping);
    for (uint32_t retired : {9u, 10u, 11u, 12u}) assert(!input.ProcessMotionState(retired));
}
void CheckVelocity() {
    KeyboardInterface input;
    auto store = std::make_shared<MotionStateStore>();
    input.SetMotionStateStore(store);
    store->SetState(RLStandControlMode);
    input.ProcessKey('W', 200.0);
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0.05f);
    input.ProcessKey('w', 300.0);  // Debounce is 120 ms.
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0.05f);
    input.ProcessKey('w', 321.0);
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0.10f);
    input.ProcessKey('a', 400.0);
    input.ProcessKey('q', 500.0);
    input.ProcessKey('\0', 1400.0);
    Near(input.GetUserCommandSnapshot().side_vel_scale, 0.70f);
    input.ProcessKey('\0', 1401.0);
    Near(input.GetUserCommandSnapshot().side_vel_scale, 0.0f);
    Near(input.GetUserCommandSnapshot().turnning_vel_scale, 0.70f);
    input.ProcessKey('\0', 1501.0);
    Near(input.GetUserCommandSnapshot().turnning_vel_scale, 0.0f);
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0.10f);
    input.ProcessSteer(3.0f, -3.0f, 3.0f);
    input.ProcessKey('\0', 1600.0);
    auto command = input.GetUserCommandSnapshot();
    Near(command.forward_vel_scale, 0.8f); Near(command.side_vel_scale, -0.8f); Near(command.turnning_vel_scale, 0.8f);

    for (RobotMotionState state : {AmpWalkingControl}) {
        store->SetState(state);
        input.ProcessSteer(3.0f, -3.0f, 3.0f);
        input.ProcessKey('\0', 1800.0);
        command = input.GetUserCommandSnapshot();
        Near(command.forward_vel_scale, 1.5f); Near(command.side_vel_scale, -0.5f); Near(command.turnning_vel_scale, 1.57f);
        input.ProcessSteer(-3.0f, 0.0f, 0.0f);
        input.ProcessKey('d', 2000.0);
        input.ProcessKey('e', 2000.0);
        command = input.GetUserCommandSnapshot();
        Near(command.forward_vel_scale, -0.6f); Near(command.side_vel_scale, -0.5f); Near(command.turnning_vel_scale, -1.57f);
        input.ProcessKey('x', 2100.0);
        command = input.GetUserCommandSnapshot();
        Near(command.forward_vel_scale, 0); Near(command.side_vel_scale, 0); Near(command.turnning_vel_scale, 0);
    }
    input.ProcessRealSteer(4.0f, -4.0f, 9.0f);
    command = input.GetUserCommandSnapshot();
    Near(command.forward_vel_scale, 0.8f); Near(command.side_vel_scale, -0.5f); Near(command.turnning_vel_scale, 1.0f);
    input.ProcessNeckSteer(0.4f, -0.3f);
    store->SetState(JointDamping);
    input.ProcessKey('\0', 2300.0);
    command = input.GetUserCommandSnapshot();
    Near(command.forward_vel_scale, 0); Near(command.side_vel_scale, 0); Near(command.turnning_vel_scale, 0);
    Near(command.neck_yaw_cmd, 0.4f); Near(command.neck_pitch_cmd, -0.3f);
}
void CheckProtocolAndSnapshots() {
    static_assert(WaitingForStand == 0x0 && Idle == WaitingForStand && JointDamping == 0x2);
    static_assert(AmpZeroPos == 0x20006 && AmpWalkingControl == 0x11);
    static_assert(RLStandControlMode == 0x1100b && AmpMoe15Dof == 0x1100c);
    static_assert(AmpWalkingWAQ == 0x1100d && kAmpWalkingWAQ == 0x1100d);
    static_assert(RLControlMode == 0x1100e && RLControlMimic12Mode == 0x1100f && RLControlMimic13Mode == 0x11010);
    static_assert(Stand2KneeMimicMode == 0x11011 && Knee2StandMimicMode == 0x11012);
    static_assert(RLStandSaluteMode == 0x11013 && kRLStandSalute == 0x11013);
    static_assert(RLStandSaluteLeftArmMode == 0x11014 && kRLStandSaluteLeftArm == 0x11014);
    static_assert(ZeroPos != JointDamping);
    static_assert(alignof(RobotBasicState) >= alignof(Vec4f));
    DdsInterface input;
    for (const auto state : kRobotMotionStates) {
        assert(input.ProcessMotionState(state));
        assert(input.GetUserCommandSnapshot().target_mode == state);
    }
    for (uint32_t state : {1u, 3u, 5u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 0x11015u, std::numeric_limits<uint32_t>::max()}) {
        assert(!input.ProcessMotionState(state));
        assert(input.GetUserCommandSnapshot().target_mode == RLStandSaluteLeftArmMode);
    }
    input.SetSafeControlMode(1u << kSafeImuData);
    assert(input.GetUserCommandSnapshot().safe_control_mode == (1u << kSafeImuData));
    auto store = std::make_shared<MotionStateStore>();
    store->SetState(AmpWalkingControl);
    input.SetMotionStateStore(store);
    std::atomic<bool> complete{false};
    std::thread writer([&] {
        for (int i = 0; i < 20000; ++i) input.ProcessSteer(static_cast<float>(i), static_cast<float>(i), static_cast<float>(i));
        complete.store(true);
    });
    do {
        const auto command = input.GetUserCommandSnapshot();
        assert(command.forward_vel_scale == command.side_vel_scale && command.side_vel_scale == command.turnning_vel_scale);
    } while (!complete.load());
    writer.join();
}
void CheckDdsControllerInput() {
    DdsInterface input;
    auto store = std::make_shared<MotionStateStore>();
    MotionStateFeedback feedback;
    input.SetMotionStateStore(store);
    input.SetMotionStateFeedback(&feedback);
    assert(input.ProcessGait(0x21000));
    assert(feedback.GetCurrentGait() == HumanWALK);
    assert(!input.ProcessGait(0x21001));
    assert(!input.IsActionAllowedByCert(0x3000));
    // The lambdas subscribed by KeyboardInterface must dispatch to the DDS
    // overrides, including their two-second stale-packet timeout.
    KeyboardInterface& callback = input;
    for (const auto state : {RLControlMode, AmpWalkingControl, RLStandControlMode, AmpMoe15Dof, AmpWalkingWAQ}) {
        store->SetState(state);
        const auto received = std::chrono::steady_clock::now();
        callback.ProcessSteer(.4f, .2f, -.3f);
        input.ExpireSteerCommand(received + std::chrono::milliseconds(1900));
        auto command = input.GetUserCommandSnapshot();
        Near(command.forward_vel_scale, .4f); Near(command.side_vel_scale, .2f);
        Near(command.turnning_vel_scale, -.3f);
        input.ExpireSteerCommand(received + std::chrono::milliseconds(2100));
        command = input.GetUserCommandSnapshot();
        Near(command.forward_vel_scale, 0); Near(command.side_vel_scale, 0);
        Near(command.turnning_vel_scale, 0);
        callback.ProcessRealSteer(.6f, .375f, 1.125f);
        command = input.GetUserCommandSnapshot();
        Near(command.forward_vel_scale, .6f); Near(command.side_vel_scale, .5f);
        Near(command.turnning_vel_scale, .5f);
    }
    store->SetState(Idle);
    callback.ProcessSteer(.5f, .5f, .5f);
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0);
    store->SetState(AmpWalkingControl);
    callback.ProcessSteer(std::numeric_limits<float>::quiet_NaN(), 1.f, 1.f);
    Near(input.GetUserCommandSnapshot().side_vel_scale, 0);
    feedback.UpdateCurrentState(AmpWalkingControl);
    feedback.UpdateCurrentState(AmpMoe15Dof);
    assert(feedback.GetCurrentState() == 0x1100c && feedback.GetLastState() == 0x11);
}
}
int main() {
    CheckKeyboardAndGamepad();
    CheckEmergencyPriority();
    CheckVelocity();
    CheckProtocolAndSnapshots();
    CheckDdsControllerInput();
    std::cout << "CR1PRO command mapping, protocol, velocity timeouts and snapshot checks passed.\n";
}
