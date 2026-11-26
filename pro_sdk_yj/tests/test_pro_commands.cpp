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
    {'u', AmpControl_Walk_Mode, RLControl_StandSalute_Mode}, {'u', RLControl_Stand_Mode, RLControl_StandSalute_Mode},
    {'m', RLControl_StandSalute_Mode, AmpControl_Walk_Mode}, {'c', RLControl_StandSalute_Mode, AmpControl_Walk_Mode},
    {'m', RLControl_StandSaluteLeftArm_Mode, AmpControl_Walk_Mode}, {'c', RLControl_StandSaluteLeftArm_Mode, AmpControl_Walk_Mode},
    {'b', RLControl_Stand_Mode, AmpControl_Walk2Run_Mode}, {'b', AmpControl_Walk_Mode, AmpControl_Walk2Run_Mode},
    {'m', MimicControl_ChairUp_Mode, AmpControl_Walk_Mode}, {'n', AmpControl_Walk2Run_Mode, MimicControl_ChairDown_Mode},
    {'m', MimicControl_ChairDown_Mode, MimicControl_ChairUp_Mode},
    {'v', AmpControl_Walk2Run_Mode, RLControl_Stand_Mode}, {'m', AmpControl_Walk2Run_Mode, AmpControl_Walk_Mode},
    {'z', Idle, AmpZeroPos}, {'z', JointDamping, AmpZeroPos}, {'z', ZeroPos, AmpZeroPos},
    {'c', AmpZeroPos, AmpControl_Walk_Mode}, {'v', AmpControl_Walk_Mode, RLControl_Stand_Mode},
    {'m', RLControl_Stand_Mode, AmpControl_Walk_Mode},
    {'f', Idle, AmpZeroPos}, {'f', JointDamping, AmpZeroPos}, {'f', AmpWholeBodyControlMode, AmpZeroPos},
    {'h', AmpZeroPos, AmpWholeBodyControlMode},
    {'j', Idle, AmpWalkingZeroPos}, {'j', JointDamping, AmpWalkingZeroPos},
    {'j', ZeroPos, AmpWalkingZeroPos}, {'j', AmpZeroPos, AmpWalkingZeroPos},
    {'j', AmpControl_Walk_Mode, AmpWalkingZeroPos}, {'l', AmpWalkingZeroPos, AmpControl_Walk_Mode},
    {'k', AmpControl_Walk_Mode, MimicControl_BoxFight_Mode},
    {'k', MimicControl_BoxFight_Mode, AmpControl_Walk_Mode}
};
void CheckKeyboardAndGamepad() {
    for (ControlMode mode : {kWholeBody, kUpperBody}) {
        KeyboardInterface input(CR1PRO, mode);
        auto store = std::make_shared<MotionStateStore>();
        input.SetMotionStateStore(store);
        for (const auto state : kRobotMotionStates) {
            store->SetState(state);
            for (char key : std::string("rzcvbmngufhtyjlpogk")) {
                input.ResetUserCommand();
                input.UpdateCommand([](UserCommand& command) { command.target_mode = kUnchanged; });
                uint32_t expected = kUnchanged;
                if (key == 'r') expected = JointDamping;
                for (const auto& transition : transitions)
                    if (transition.key == key && transition.from == state) expected = transition.to;
                if (mode == kUpperBody && (key == 'b' || key == 'n' || key == 'u' ||
                    key == 'k' ||
                    ((key == 'm' || key == 'c') && (state == RLControl_StandSalute_Mode || state == RLControl_StandSaluteLeftArm_Mode)) ||
                    ((key == 'm') && (state == MimicControl_ChairDown_Mode ||
                                      state == MimicControl_ChairUp_Mode)))) expected = kUnchanged;
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
                        (state == AmpZeroPos || state == AmpWalkingZeroPos || state == RLControl_Stand_Mode || state == AmpControl_Walk2Run_Mode || state == RLControl_StandSalute_Mode || state == RLControl_StandSaluteLeftArm_Mode))
                        expected = AmpControl_Walk_Mode;
                    if (key == "G20_KEY_R1" && (state == AmpControl_Walk_Mode || state == AmpControl_Walk2Run_Mode)) expected = RLControl_Stand_Mode;
                } else {
                    if (key == "G20_KEY_L1" && (state == Idle || state == UpperBodyControlMode)) expected = ZeroPos;
                    if (key == "G20_KEY_L2" && state == ZeroPos) expected = UpperBodyControlMode;
                }
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
    store->SetState(RLControl_Stand_Mode);
    UdpNode remote("127.0.0.1", 8888, "127.0.0.1", 9999);  // No sockets started.
    input.SetUdpRemote(&remote);
    input.ProcessSteer(.5f, .3f, .2f);
    remote.queueKey('b');
    remote.queueVelocity(.8f, .5f, .4f);
    remote.queueEstop();
    input.ProcessKey('m', 100);
    auto command = input.GetUserCommandSnapshot();
    assert(command.target_mode == JointDamping);
    Near(command.forward_vel_scale, 0); Near(command.side_vel_scale, 0); Near(command.turnning_vel_scale, 0);
    assert(!remote.hasPendingKey() && !remote.hasPendingVelocity() && !remote.hasPendingEstop());
    input.ProcessGamepadKey(KeyCode::L2);
    assert(!input.ProcessMotionState(AmpControl_Walk2Run_Mode));
    input.ProcessSteer(.5f, .5f, .5f);
    input.ProcessKey('b', 110);
    assert(input.GetUserCommandSnapshot().target_mode == JointDamping);
    Near(input.GetUserCommandSnapshot().forward_vel_scale, 0);
    // Release only after the FSM reports that it applied the damping request.
    store->SetState(JointDamping);
    input.ProcessKey('z', 120);
    assert(input.GetUserCommandSnapshot().target_mode == AmpZeroPos);
    store->SetState(AmpControl_Walk_Mode);
    input.ProcessKey('r', 130);
    input.ProcessKey('v', 140);
    assert(input.GetUserCommandSnapshot().target_mode == JointDamping);
    for (uint32_t retired : {9u, 10u, 11u, 12u}) assert(!input.ProcessMotionState(retired));
}
void CheckVelocity() {
    KeyboardInterface input;
    auto store = std::make_shared<MotionStateStore>();
    input.SetMotionStateStore(store);
    store->SetState(RLControl_Stand_Mode);
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

    for (RobotMotionState state : {AmpControl_Walk_Mode}) {
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
    static_assert(AmpZeroPos == 0x20006 && AmpControl_Walk_Mode == 0x11);
    static_assert(RLControl_Stand_Mode == 0x1100b);
    static_assert(AmpControl_Walk2Run_Mode == 0x1100c && kRLControl == 0x1100c);
    static_assert(RLControl_StandTalking_Mode == 0x1100d && kRLStandTalking == 0x1100d);
    static_assert(RLControl_StandPlease2_Mode == 0x1100e && kRLStandPlease2 == 0x1100e);
    static_assert(RLControl_StandSalute_Mode == 0x1100f && kRLStandSalute == 0x1100f);
    static_assert(RLControl_StandSaluteLeftArm_Mode == 0x11010 && kRLStandSaluteLeftArm == 0x11010);
    static_assert(RLControl_StandPlease_Mode == 0x11011 && kRLStandPlease == 0x11011);
    static_assert(MimicControl_ChairDown_Mode == 0x11012 && kRLMimic12Control == 0x11012);
    static_assert(MimicControl_ChairUp_Mode == 0x11013 && kRLMimic13Control == 0x11013);
    static_assert(MimicControl_BoxFight_Mode == 0x11014 && kRLMimic2Control == 0x11014);
    static_assert(MimicControl_LieDown_Mode == 0x11015 && kTangxia == 0x11015);
    static_assert(MimicControl_GetUp_Mode == 0x11016 && kZhanli == 0x11016);
    static_assert(ZeroPos != JointDamping);
    static_assert(alignof(RobotBasicState) >= alignof(Vec4f));
    DdsInterface input;
    uint32_t lastAccepted = Idle;
    for (const auto state : kRobotMotionStates) {
        if (state == MimicControl_LieDown_Mode || state == MimicControl_GetUp_Mode) {
            assert(IsKnownRobotMotionState(state));
            assert(!input.ProcessMotionState(state));
            assert(!input.SetTargetModeRequest(state));
            assert(input.GetUserCommandSnapshot().target_mode == lastAccepted);
            continue;
        }
        assert(input.ProcessMotionState(state));
        lastAccepted = state;
        assert(input.GetUserCommandSnapshot().target_mode == state);
    }
    for (uint32_t state : {1u, 3u, 5u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 0x11017u, 0x11018u,
                           0x11019u, 0x1101au, 0x1101bu, 0x1101cu,
                           std::numeric_limits<uint32_t>::max()}) {
        assert(!input.ProcessMotionState(state));
        assert(input.GetUserCommandSnapshot().target_mode == lastAccepted);
    }
    input.SetSafeControlMode(1u << kSafeImuData);
    assert(input.GetUserCommandSnapshot().safe_control_mode == (1u << kSafeImuData));
    auto store = std::make_shared<MotionStateStore>();
    store->SetState(AmpControl_Walk_Mode);
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
    for (const auto state : {AmpControl_Walk2Run_Mode, AmpControl_Walk_Mode, RLControl_Stand_Mode}) {
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
    store->SetState(AmpControl_Walk_Mode);
    callback.ProcessSteer(std::numeric_limits<float>::quiet_NaN(), 1.f, 1.f);
    Near(input.GetUserCommandSnapshot().side_vel_scale, 0);
    feedback.UpdateCurrentState(AmpControl_Walk_Mode);
    feedback.UpdateCurrentState(AmpControl_Walk2Run_Mode);
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
