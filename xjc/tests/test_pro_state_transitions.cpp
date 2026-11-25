// Production state implementations with isolated robot, user and inference services.
#include "drdds/core/drdds_core.h"
#include "humanoid_pro/amp_moe_15dof_state.hpp"
#include "humanoid_pro/amp_walking_waq_state.hpp"
#include "humanoid/idle_state.hpp"
#include "humanoid/joint_damping_state.hpp"
#include "humanoid_pro/knee2stand_mimic_state.hpp"
#include "humanoid_pro/rl_control_state.hpp"
#include "humanoid_pro/rl_mimic12_control_state.hpp"
#include "humanoid_pro/rl_mimic13_control_state.hpp"
#include "humanoid_pro/rl_stand_salute_left_arm_state.hpp"
#include "humanoid_pro/rl_stand_salute_state.hpp"
#include "humanoid_pro/stand2knee_mimic_state.hpp"
#include "humanoid/zeropos_state.hpp"
#include "humanoid_pro/amp_zeropos_state.hpp"
#include "humanoid_pro/amp_walking_zeropos_state.hpp"
#include "humanoid_pro/upper_body_control_state.hpp"
#include "humanoid_pro/rl_stand_control_state.hpp"
#include "humanoid_pro/amp_whole_body_control_state.hpp"
#include "humanoid_pro/amp_walking_state.hpp"
#include "humanoid_pro/pro_controllers.hpp"
#include "policy_runner_base.hpp"
#include <array>
#include <functional>
#include <stdexcept>
#include <vector>

using namespace humanoid;
using namespace std::chrono;

static void Require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
template <typename Predicate>
static void WaitFor(Predicate predicate, const std::string& message) {
    const auto deadline = steady_clock::now() + milliseconds(500);
    while (!predicate() && steady_clock::now() < deadline)
        std::this_thread::sleep_for(milliseconds(1));
    Require(predicate(), message);
}
template <typename T>
static std::unique_ptr<StateBase> Make(std::shared_ptr<ControllerData> data) {
    return std::make_unique<T>(RobotName::CR1PRO, "test", data);
}
struct Contract {
    StateName state;
    std::function<std::unique_ptr<StateBase>(std::shared_ptr<ControllerData>)> make;
    std::vector<StateName> exits;
    bool policy = false;
};
static const std::vector<Contract> contracts = {
    {kRLControl, Make<RLControlState>, {kRLMimic12Control, kRLStandControl, kAmpWalkingControl, kRLStandSalute, kRLStandSaluteLeftArm}, true},
    {kRLMimic12Control, Make<RLMimic12ControlState>, {kRLMimic13Control}, true},
    {kRLMimic13Control, Make<RLMimic13ControlState>, {kRLControl}, true},
    {kStand2KneeMimic, Make<Stand2KneeMimicState>, {kKnee2StandMimic}, true},
    {kKnee2StandMimic, Make<Knee2StandMimicState>, {kAmpWalkingControl}, true},
    {kIdle, Make<IdleState>, {kZeroPos, kAmpZeroPos, kAmpWalkingZeroPos}},
    {kJointDamping, Make<JointDampingState>, {}},
    {kZeroPos, Make<ZeroPosState>, {kRLStandControl, kUpperBodyControl, kAmpWalkingZeroPos, kAmpZeroPos, kRLStandSalute, kRLStandSaluteLeftArm}},
    {kRLStandControl, Make<RLStandControlState>, {kAmpWalkingControl, kAmpMoe15Dof, kAmpWalkingWAQ, kRLControl, kRLStandSalute, kRLStandSaluteLeftArm}, true},
    {kRLStandSalute, Make<RLStandSaluteState>, {kAmpWalkingControl, kAmpMoe15Dof, kAmpWalkingWAQ, kRLControl, kRLStandSaluteLeftArm}, true},
    {kRLStandSaluteLeftArm, Make<RLStandSaluteLeftArmState>, {kAmpWalkingControl, kAmpMoe15Dof, kAmpWalkingWAQ, kRLControl, kRLStandSalute}, true},
    {kUpperBodyControl, Make<UpperBodyControlState>, {kZeroPos}},
    {kAmpZeroPos, Make<AmpZeroPosState>, {kAmpWholeBodyControl, kAmpWalkingZeroPos, kAmpWalkingControl, kAmpWalkingWAQ}},
    {kAmpWholeBodyControl, Make<AmpWholeBodyControlState>, {kAmpWalkingControl, kAmpZeroPos}, true},
    {kAmpWalkingZeroPos, Make<AmpWalkingZeroPosState>, {kAmpWalkingControl, kAmpWalkingWAQ}},
    {kAmpWalkingControl, Make<AmpWalkingState>, {kRLStandControl, kAmpWalkingWAQ, kAmpWalkingZeroPos, kRLControl, kStand2KneeMimic, kRLStandSalute, kRLStandSaluteLeftArm}, true},
    {kAmpMoe15Dof, Make<AmpMoe15DofState>, {kRLStandControl, kAmpWalkingControl, kAmpWalkingWAQ, kRLStandSalute, kRLStandSaluteLeftArm}, true},
    {kAmpWalkingWAQ, Make<AmpWalkingWAQState>, {kRLStandControl, kAmpWalkingControl, kRLStandSalute, kRLStandSaluteLeftArm}, true},
};

static StateName Next(StateBase& state) {
    return state.LoseControlJudge() ? kJointDamping : state.GetNextStateName();
}
static void CheckProductionRegistry() {
    ProPolicyFaults::Reset();
    auto data = std::make_shared<ControllerData>();
    data->motion_state_store->SetState(RobotMotionState::AmpWalkingControl);
    const auto registry = MakeProControllers(RobotName::CR1PRO, data);
    Require(registry.size() == kRobotMotionStates.size(), "production registry must match the enabled states");
    Require(registry.size() == 18, "production registry must contain exactly 18 states");
    Require(contracts.size() == registry.size(), "transition contracts must cover every registered state");
    for (const auto state : kRobotMotionStates) {
        Require(std::count_if(contracts.begin(), contracts.end(), [state](const Contract& contract) {
            return static_cast<uint32_t>(contract.state) == state;
        }) == 1, "a supported state does not have exactly one transition contract");
    }
    for (const auto& contract : contracts) {
        const auto id = contract.state;
        const auto found = registry.find(id);
        Require(found != registry.end() && found->second, "production registry is missing state " + std::to_string(id));
        const auto expected = contract.make(data);
        Require(typeid(*found->second) == typeid(*expected), "production registry registered the wrong controller class");
    }
    Require(registry.find(static_cast<StateName>(14)) == registry.end(), "unknown state was registered");
    Require(data->motion_state_store->GetState() == RobotMotionState::AmpWalkingControl,
            "controller construction changed the previous-state store");
    bool rejected = false;
    try { MakeProControllers(RobotName::CR1STANDARD, data); }
    catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "production registry accepted STANDARD");
    rejected = false;
    try { MakeProControllers(RobotName::CR1PRO, nullptr); }
    catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "production registry accepted a null context");
    data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
    rejected = false;
    try { MakeProControllers(RobotName::CR1PRO, data); }
    catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "production registry accepted 21 DOF parameters");
}
static void CheckAllTransitions() {
    for (const auto& contract : contracts) {
        ProPolicyFaults::Reset();
        auto data = std::make_shared<ControllerData>();
        auto state = contract.make(data);
        state->OnEnter();
        Require(data->motion_state_store->GetState() == static_cast<RobotMotionState>(contract.state), "wrong state feedback");
        // Exhaustively check every supported target after preparation completes.
        data->ri_ptr->time = 3.0;
        std::vector<uint32_t> targets(kRobotMotionStates.begin(), kRobotMotionStates.end());
        for (uint32_t retired : {9u, 10u, 11u, 12u}) targets.push_back(retired);
        targets.push_back(14); // Unknown protocol code must not add an edge.
        for (const uint32_t target : targets) {
            data->uc_ptr->Set(target);
            state->Run();
            if ((contract.state == kRLStandSalute || contract.state == kRLStandSaluteLeftArm) && target == AmpWalkingControl) {
                Require(Next(*state) == contract.state,"Salute exited before lowering completed");
                Require(ProPolicyFaults::salute_lowering_requested,"Salute did not request arm lowering");
                ProPolicyFaults::salute_lowering_complete = true;
                Require(Next(*state) == kAmpWalkingControl,"Lowered salute did not enter walking");
                state->OnExit();
                ProPolicyFaults::salute_lowering_complete = false;
                state->OnEnter();
                continue;
            }
            StateName expected = contract.state;
            if (std::find(contract.exits.begin(), contract.exits.end(), static_cast<StateName>(target)) != contract.exits.end())
                expected = static_cast<StateName>(target);
            if (target == RobotMotionState::JointDamping && contract.state != kIdle && contract.state != kJointDamping)
                expected = kJointDamping;
            Require(Next(*state) == expected, "transition mismatch from " + std::to_string(contract.state) + " to " + std::to_string(target));
        }
        data->uc_ptr->Set(255);
        state->Run();
        Require(Next(*state) == contract.state, "unknown target changed state");
        data->uc_ptr->Set(0, 1);
        state->Run();
        const auto safe_expected = contract.state == kIdle || contract.state == kJointDamping
            ? contract.state : kJointDamping;
        Require(Next(*state) == safe_expected, "source safety contract changed");
        data->uc_ptr->Set(0);
        for (int i = 0; i < 25; ++i) state->Run();
        WaitFor([&] { return data->ri_ptr->packets.load() > 0; }, "state never produced a packet");
        const auto command = data->ri_ptr->GetJointCommand();
        Require(command.rows() == 31 && command.cols() == 5 && command.allFinite(), "state packet is not finite 31x5");
        if (contract.policy) Require(command.bottomRows(2).isZero(), "policy powered neck joints");
        state->OnExit();
        Require(!ProPolicyFaults::unsafe_exit, "policy OnExit raced an active inference call");
    }
}
static void CheckSaluteDeferredWalking() {
    ProPolicyFaults::Reset();
    auto data = std::make_shared<ControllerData>();
    RLStandSaluteState state(RobotName::CR1PRO,"salute-lowering",data);
    state.OnEnter();
    data->uc_ptr->Set(AmpWalkingControl); state.Run();
    Require(Next(state) == kRLStandSalute,"Walking request bypassed salute lowering");
    Require(data->motion_state_store->GetState() == RLStandSaluteMode,"Feedback changed before lowering");
    // Once accepted, a single walking request remains pending even if a normal
    // mode packet follows. RL Control takeover and safety take precedence.
    data->uc_ptr->Set(AmpMoe15Dof); state.Run();
    Require(Next(state) == kRLStandSalute,"A later normal command bypassed the pending lowering");
    data->uc_ptr->Set(RLControlMode); state.Run();
    Require(Next(state) == kRLControl,"RL Control takeover waited for arm lowering");
    data->uc_ptr->Set(RLControlMode,1); state.Run();
    Require(Next(state) == kJointDamping,"Lowering delayed a safety fault");
    data->uc_ptr->Set(JointDamping); state.Run();
    Require(Next(state) == kJointDamping,"Lowering delayed explicit damping");
    state.OnExit();
    ProPolicyFaults::Reset();
    data->uc_ptr->Set(RLStandSaluteMode); state.OnEnter(); state.Run();
    Require(Next(state) == kRLStandSalute,"Reentry retained a pending walking request");
    data->uc_ptr->Set(AmpWalkingControl); state.Run();
    Require(Next(state) == kRLStandSalute,"Reentry skipped lowering");
    data->uc_ptr->Set(AmpMoe15Dof); state.Run();
    ProPolicyFaults::salute_lowering_complete = true;
    Require(Next(state) == kAmpWalkingControl,"A latched walking request was lost");
    data->uc_ptr->Set(RLControlMode); state.Run();
    Require(Next(state) == kRLControl,"Completed lowering overrode RL Control takeover");
    state.OnExit();
    ProPolicyFaults::Reset();
}

static void CheckPreparationGate() {
    struct Entry {
        std::function<std::unique_ptr<StateBase>(std::shared_ptr<ControllerData>)> make;
        StateName from;
        RobotMotionState to;
    };
    for (const auto& entry : std::vector<Entry>{
        {Make<ZeroPosState>, kZeroPos, RLStandControlMode},
        {Make<ZeroPosState>, kZeroPos, RLStandSaluteMode},
        {Make<ZeroPosState>, kZeroPos, RLStandSaluteLeftArmMode},
        {Make<AmpZeroPosState>, kAmpZeroPos, AmpWholeBodyControlMode},
        {Make<AmpWalkingZeroPosState>, kAmpWalkingZeroPos, AmpWalkingControl}}) {
        auto data = std::make_shared<ControllerData>();
        auto state = entry.make(data);
        state->OnEnter();
        data->uc_ptr->Set(entry.to);
        data->ri_ptr->time = 2.999;
        state->Run();
        Require(Next(*state) == entry.from, "preparation was interrupted before the spline completed");
        data->uc_ptr->Set(entry.to, 1);
        state->Run();
        Require(Next(*state) == kJointDamping, "preparation timer delayed a safety stop");
        data->uc_ptr->Set(JointDamping);
        state->Run();
        Require(Next(*state) == kJointDamping, "preparation timer delayed explicit damping");
        data->uc_ptr->Set(entry.to);
        data->ri_ptr->time = 3.0;
        state->Run();
        Require(Next(*state) == static_cast<StateName>(entry.to), "completed preparation blocked its target");
        state->OnExit();
    }
}

static void CheckPreparationAndIdle() {
    for (auto make : {Make<ZeroPosState>, Make<AmpZeroPosState>, Make<AmpWalkingZeroPosState>}) {
        auto data = std::make_shared<ControllerData>();
        auto state = make(data);
        state->OnEnter();
        data->ri_ptr->time = 3;
        state->Run();
        const auto command = data->ri_ptr->GetJointCommand();
        Require(command.rows() == 31 && command.allFinite(), "preparation failed 31 DOF initialization");
        Require(std::abs(command(6, 1) - 1.4f) < 1e-5f && std::abs(command(13, 1) - 1.4f) < 1e-5f,
                "source preparation elbow pose changed");
        Require(command.bottomRows(2).col(1).isZero(), "preparation neck positions not initialized");
        state->OnExit();
    }
    auto data = std::make_shared<ControllerData>();
    data->ri_ptr->command.setConstant(9);
    IdleState idle(RobotName::CR1PRO, "idle", data);
    idle.OnEnter();
    idle.Run();
    Require(data->ri_ptr->GetJointCommand().isZero(), "whole-body idle retained previous powered packet");
    idle.OnExit();
    data->control_mode = ControlMode::kUpperBody;
    data->ri_ptr->q.setConstant(.2f);
    IdleState upper_idle(RobotName::CR1PRO, "upper-idle", data);
    upper_idle.OnEnter();
    upper_idle.Run();
    Require(data->ri_ptr->GetJointCommand().col(1).isApprox(data->ri_ptr->q), "upper-body idle failed to retain initial position");
    upper_idle.OnExit();
}
static void CheckIdleMarkZero() {
    for (ControlMode mode : {ControlMode::kWholeBody, ControlMode::kUpperBody}) {
        auto data = std::make_shared<ControllerData>();
        data->control_mode = mode;
        auto& robot = *data->ri_ptr;
        IdleState idle(RobotName::CR1PRO, "idle-mark-zero", data);
        Require(ChannelUInt64::Publish(1) == 1, "Idle is missing /JOINTS_MARK_ZERO on domain 0");
        idle.OnEnter();
        idle.Run();
        Require(robot.zero_masks.empty(), "Idle accepted a mark-zero request before entry");

        // Every individual bit, including the highest PRO joint, stays intact.
        for (int joint = 0; joint < 31; ++joint) {
            const uint64_t mask = uint64_t{1} << joint;
            const auto count = robot.zero_masks.size();
            robot.zero_events.clear();
            robot.command.setConstant(9.f);
            std::thread receiver([mask] { ChannelUInt64::Publish(mask); });
            receiver.join();
            idle.Run();
            Require(robot.zero_masks.size() == count + 1 && robot.zero_masks.back() == mask,
                    "single-joint mark-zero selected the wrong joint");
            Require(robot.offset_masks.back() == mask, "offset reset used a different joint mask");
            Require(robot.zero_events == std::vector<std::string>{"zero", "offset", "wrap"},
                    "mark-zero did not reset and wrap offsets in order");
            Require(robot.offset_time - robot.zero_time >= milliseconds(5),
                    "offset reset did not wait for the driver after SetHome");
            Require(robot.command_at_zero.isZero(), "mark-zero retained a powered command");
            idle.Run();
            Require(robot.zero_masks.size() == count + 1, "mark-zero repeated without another request");
        }

        const auto count = robot.zero_masks.size();
        ChannelUInt64::Publish(0);
        ChannelUInt64::Publish(uint64_t{1} << 31);
        ChannelUInt64::Publish(uint64_t{1} << 63);
        Require(ChannelUInt64::Publish(1, "/WRONG_TOPIC") == 0, "mark-zero subscribed to the wrong topic");
        Require(ChannelUInt64::Publish(1, "/JOINTS_MARK_ZERO", 1) == 0, "mark-zero subscribed to the wrong domain");
        idle.Run();
        Require(robot.zero_masks.size() == count, "invalid mask issued a hardware zero command");
        ChannelUInt64::Publish((uint64_t{1} << 63) | 4);
        idle.Run();
        Require(robot.zero_masks.back() == 4, "out-of-range bits reached the hardware");

        ChannelUInt64::Publish(1);
        ChannelUInt64::Publish(uint64_t{1} << 30);
        idle.Run();
        Require(robot.zero_masks.back() == (1 | (uint64_t{1} << 30)),
                "a second request overwrote a pending joint selection");

        // A DDS callback during SetHome must survive for the next control tick.
        robot.on_set_zero = [] { ChannelUInt64::Publish(8); };
        ChannelUInt64::Publish(2);
        idle.Run();
        robot.on_set_zero = {};
        Require(robot.zero_masks.back() == 2, "in-flight request changed the active mask");
        idle.Run();
        Require(robot.zero_masks.back() == 8, "request received during mark-zero was lost");

        ChannelUInt64::Publish(16);
        idle.OnExit();
        ChannelUInt64::Publish(32);
        idle.OnEnter();
        const auto before_reentry = robot.zero_masks.size();
        idle.Run();
        Require(robot.zero_masks.size() == before_reentry, "Idle re-entry executed a stale mark-zero request");

        // Upper-body Idle must hold the new coordinates after calibration.
        robot.on_wrap_offset = [&robot] { robot.q.setConstant(.7f); };
        ChannelUInt64::Publish(64);
        idle.Run();
        if (mode == ControlMode::kUpperBody)
            Require(robot.GetJointCommand().col(1).isApprox(robot.q),
                    "upper-body Idle held pre-calibration joint coordinates");
        else
            Require(robot.GetJointCommand().isZero(), "whole-body mark-zero powered the joints");
        idle.OnExit();
    }
    Require(ChannelUInt64::Publish(1) == 0, "Idle destruction leaked its mark-zero subscription");
}
static void CheckDampingTimer() {
    auto data = std::make_shared<ControllerData>();
    JointDampingState state(RobotName::CR1PRO, "damping", data);
    state.OnEnter();
    data->uc_ptr->Set(RobotMotionState::AmpZeroPos);
    data->ri_ptr->time = 9.999;
    state.Run();
    Require(Next(state) == kJointDamping, "damping exited before 10 seconds");
    data->ri_ptr->time = 10;
    state.Run();
    Require(Next(state) == kIdle, "damping did not return to idle after 10 seconds");
    Require(data->uc_ptr->GetUserCommandSnapshot().target_mode == RobotMotionState::AmpZeroPos,
            "damping cleared queued source target");
    const auto command = data->ri_ptr->GetJointCommand();
    for (int col : {0, 1, 3, 4}) Require(command.col(col).isZero(), "damping generated active command fields");
    state.OnExit();
}
static void CheckIdleDriverRecovery() {
    const uint32_t driver_fault = 1u << kSafeDriverStatus;
    const uint32_t imu_fault = 1u << kSafeImuData;
    for (ControlMode mode : {ControlMode::kWholeBody, ControlMode::kUpperBody}) {
        auto data = std::make_shared<ControllerData>();
        data->control_mode = mode;
        data->ri_ptr->time = 100.0;
        IdleState idle(RobotName::CR1PRO, "idle-recovery", data);
        idle.OnEnter();
        const auto step = [&](double time, uint32_t safe) {
            data->ri_ptr->time = time;
            data->uc_ptr->Set(RobotMotionState::AmpZeroPos, safe);
            idle.Run();
            return Next(idle);
        };

        Require(step(102.999, driver_fault) == kIdle, "driver fault left Idle before recovery");
        Require(data->ri_ptr->driver_resets == 0, "driver reset ran before 3 seconds in Idle");
        Require(step(103.0, driver_fault | imu_fault) == kIdle, "recovery bypassed active safety faults");
        Require(data->ri_ptr->driver_resets == 1, "driver fault did not invoke reset/re-enable at 3 seconds");
        Require(data->uc_ptr->GetUserCommandSnapshot().target_mode == RobotMotionState::AmpZeroPos,
                "driver recovery discarded the pending target");
        step(103.0, driver_fault);
        step(105.999, driver_fault);
        Require(data->ri_ptr->driver_resets == 1, "driver recovery retried too soon");
        step(106.0, driver_fault);
        Require(data->ri_ptr->driver_resets == 2, "persistent driver fault was not retried");
        Require(step(110.0, imu_fault) == kIdle, "non-driver safety fault did not hold Idle");
        Require(data->ri_ptr->driver_resets == 2, "IMU fault incorrectly re-enabled motors");
        Require(step(111.0, 0) == kAmpZeroPos, "cleared fault did not release the pending AMP target");
        Require(data->ri_ptr->driver_resets == 2, "healthy state incorrectly reset drivers");
        idle.OnExit();

        data->ri_ptr->time = 200.0;
        idle.OnEnter();
        step(200.0, driver_fault);
        step(202.999, driver_fault);
        Require(data->ri_ptr->driver_resets == 2, "Idle re-entry retained an expired recovery timer");
        step(203.0, driver_fault);
        Require(data->ri_ptr->driver_resets == 3, "Idle re-entry did not restart driver recovery");
        idle.OnExit();
    }
}
template <typename State>
static void CheckImmediateEntryAndWorkerFailure() {
    ProPolicyFaults::Reset();
    auto data = std::make_shared<ControllerData>();
    State state(RobotName::CR1PRO, "immediate-entry", data);
    for (int entry = 0; entry < 2; ++entry) {
        const int packets = data->ri_ptr->packets.load();
        state.OnEnter();
        Require(data->ri_ptr->packets == packets + 1, "Policy entry did not send its first action immediately");
        Require(data->ri_ptr->GetJointCommand().bottomRows(2).isZero(), "Policy entry powered virtual neck joints");
        for (int tick = 1; tick < 20; ++tick) state.Run();
        std::this_thread::sleep_for(milliseconds(5));
        Require(data->ri_ptr->packets == packets + 1, "Policy did not hold the entry action for 20 ticks");
        state.Run();
        WaitFor([&] { return data->ri_ptr->packets == packets + 2; }, "Policy did not infer at tick 20");
        // Entry errors and asynchronous inference errors must both reach damping.
        ProPolicyFaults::failure = 1;
        for (int tick = 0; tick < 20; ++tick) state.Run();
        WaitFor([&] { return Next(state) == kJointDamping; }, "Policy worker failure did not route to damping");
        state.OnExit();
        Require(!ProPolicyFaults::unsafe_exit, "Policy exit raced inference");
        ProPolicyFaults::Reset();
    }
}
static void CheckWalkToRunEntryLifecycle() {
    ProPolicyFaults::Reset();
    auto data = std::make_shared<ControllerData>();
    AmpWalkingWAQState state(RobotName::CR1PRO, "walk-to-run-entry", data);
    data->uc_ptr->Set(AmpWalkingWAQ);
    state.OnEnter();
    state.OnEnter(); // Repeated entry must join the previous worker before resetting history.
    Require(data->ri_ptr->packets == 2, "Walk-to-run repeated entry did not reset/send once");
    Require(Next(state) == kAmpWalkingWAQ, "Walk-to-run changed the WAQ state ID");
    state.OnExit();
    Require(state.LoseControlJudge(), "Exited walk-to-run policy remained available");
    for (const auto& request : std::array<std::pair<uint32_t, uint32_t>, 2>{{
            {JointDamping, 0}, {AmpWalkingWAQ, 1}}}) {
        data->uc_ptr->Set(request.first, request.second);
        state.OnEnter();
        Require(data->ri_ptr->packets == 2, "Walk-to-run sent an action despite a damping/safety request");
        Require(Next(state) == kJointDamping, "Blocked walk-to-run entry did not select damping");
        Require(data->motion_state_store->GetState() == JointDamping, "Blocked entry reported WAQ");
    }
    data->uc_ptr->Set(AmpWalkingWAQ);
    state.OnEnter();
    Require(data->ri_ptr->packets == 3 && Next(state) == kAmpWalkingWAQ,
            "Walk-to-run did not recover after a blocked entry");
    state.OnExit();
    Require(!ProPolicyFaults::unsafe_exit, "Walk-to-run re-entry raced inference");
}
static void CheckPolicyFailuresAndDestruction() {
    for (const auto& contract : contracts) {
        if (!contract.policy) continue;
        for (int fault : {0, 1, 2, 3}) {
            ProPolicyFaults::Reset();
            auto data = std::make_shared<ControllerData>();
            auto state = contract.make(data);
            if (fault == 0) ProPolicyFaults::ready = false;
            else ProPolicyFaults::failure = fault;
            state->OnEnter();
            for (int i = 0; i < 25; ++i) state->Run();
            WaitFor([&] { return Next(*state) == kJointDamping; }, "policy failure did not route to damping");
            state->OnExit();
        }
        ProPolicyFaults::Reset();
        {
            auto data = std::make_shared<ControllerData>();
            auto state = contract.make(data);
            state->OnEnter();
            ProPolicyFaults::failure = 2;
            for (int i = 0; i < 40; ++i) state->Run();
            WaitFor([&] { return Next(*state) == kJointDamping; }, "inference fault did not stop the state");
            state->OnExit();
            ProPolicyFaults::failure = 0;
            state->OnEnter();
            data->uc_ptr->Set(static_cast<uint32_t>(contract.state));
            state->Run();
            Require(Next(*state) == contract.state, "transient inference fault prevented re-entry");
            state->OnExit();
        }
        ProPolicyFaults::Reset();
        auto data = std::make_shared<ControllerData>();
        const auto started = steady_clock::now();
        {
            auto state = contract.make(data);
            state->OnEnter();
            for (int i = 0; i < 25; ++i) state->Run();
            WaitFor([&] { return data->ri_ptr->packets.load() > 0; }, "destruction setup produced no packet");
            // Deliberately omit OnExit: active-state destruction must stop/join.
        }
        Require(steady_clock::now() - started < seconds(1), "active controller destruction hung");
    }
}
int main() {
    try {
        CheckProductionRegistry();
        CheckAllTransitions();
        CheckSaluteDeferredWalking();
        CheckPreparationGate();
        CheckPreparationAndIdle();
        CheckIdleMarkZero();
        CheckIdleDriverRecovery();
        CheckDampingTimer();
        CheckImmediateEntryAndWorkerFailure<AmpMoe15DofState>();
        CheckImmediateEntryAndWorkerFailure<AmpWalkingWAQState>();
        CheckWalkToRunEntryLifecycle();
        CheckImmediateEntryAndWorkerFailure<RLControlState>();
        CheckImmediateEntryAndWorkerFailure<RLMimic12ControlState>();
        CheckImmediateEntryAndWorkerFailure<RLMimic13ControlState>();
        CheckImmediateEntryAndWorkerFailure<Stand2KneeMimicState>();
        CheckImmediateEntryAndWorkerFailure<Knee2StandMimicState>();
        CheckPolicyFailuresAndDestruction();
        std::cout << "Enabled PRO registry, state contracts, 31x5 packets, failure routing and thread lifecycle passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
