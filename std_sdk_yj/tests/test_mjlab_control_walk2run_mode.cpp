#include "humanoid_std/mjlab_control_walk2run_mode_state.hpp"

#include <cassert>
#include <limits>

namespace {
constexpr int kJoints = 21;
constexpr int kFrameSize = 72;
constexpr int kInputSize = 360;

VecXf Home() {
    VecXf home(kJoints);
    home << 0.0f, 0.0f, .15f, 0.0f, 1.2f, 0.0f, -.15f, 0.0f, 1.2f,
        -.2f, 0.0f, 0.0f, .4f, -.2f, 0.0f, -.2f, 0.0f, 0.0f, .4f, -.2f, 0.0f;
    return home;
}

void CheckPolicy(const std::string& path) {
    MjlabControlWalk2RunModePolicyRunner runner(RobotName::CR1STANDARD, path);
    assert(runner.decimation_ == 20);
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "mjrun_reference");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Ort::Session reference(env, path.c_str(), options);
    const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const int64_t shape[] = {1, kInputSize};
    const char* inputs[] = {"obs"};
    const char* outputs[] = {"actions"};
    const auto home = Home();
    VecXf kp(kJoints), kd(kJoints);
    kp << 200, 100, 100, 100, 100, 100, 100, 100, 100,
        300, 300, 200, 300, 80, 30, 300, 300, 200, 300, 80, 30;
    kd << 5, 5, 5, 5, 5, 5, 5, 5, 5, 10, 10, 5, 10, 3, 1, 10, 10, 5, 10, 3, 1;

    constexpr int forward_end = 200;
    constexpr int reverse_end = forward_end + 220;
    constexpr int stop_end = reverse_end + 50;
    for (int replay = 0; replay < 2; ++replay) {
        runner.OnEnter();
        double expected_vx = 0.;
        VecXf previous_action = VecXf::Zero(kJoints), history(kInputSize);
        for (int step = 0; step < stop_end; ++step) {
            RobotBasicState state{};
            state.base_rpy = Vec3f(.10f, -.07f, .2f) * std::sin(step * .03f + .3f * replay);
            state.base_rot_mat = RpyToRm(state.base_rpy);
            state.base_omega = Vec3f(.12f, .18f, -.16f) * std::cos(step * .05f);
            state.joint_pos = home + .035f * VecXf::LinSpaced(kJoints, -1.f, 1.f) * std::cos(step * .037f);
            state.joint_vel = VecXf::LinSpaced(kJoints, -.7f, .4f) * std::sin(step * .05f);
            UserCommand command{};
            command.forward_vel_scale = step < forward_end ? 1.2f : (step < reverse_end ? -1.1f : 0.f);
            command.side_vel_scale = (step < reverse_end ? -.45f : 1.1f) * std::cos(step * .02f);
            command.turnning_vel_scale = step < forward_end ? .4f : -1.1f;

            const double target_vx = std::clamp(static_cast<double>(command.forward_vel_scale), -1., 1.)
                * (command.forward_vel_scale < 0.f ? .25 : 3.0);
            expected_vx += std::clamp(target_vx - expected_vx, -.016, .016);
            const Vec3f expected_velocity(
                static_cast<float>(expected_vx),
                .6f * std::clamp(command.side_vel_scale, -1.f, 1.f),
                std::clamp(command.turnning_vel_scale, -1.f, 1.f));
            const float roll = state.base_rpy.x(), pitch = state.base_rpy.y();
            VecXf frame(kFrameSize);
            frame << state.base_omega,
                Vec3f(std::sin(pitch), -std::sin(roll) * std::cos(pitch), -std::cos(roll) * std::cos(pitch)),
                expected_velocity, state.joint_pos - home, .05f * state.joint_vel, previous_action;
            frame = frame.cwiseMin(100.f).cwiseMax(-100.f);
            if (step == 0)
                for (int offset = 0; offset < kInputSize; offset += kFrameSize)
                    history.segment(offset, kFrameSize) = frame;
            else {
                history.head(kInputSize - kFrameSize) = history.tail(kInputSize - kFrameSize).eval();
                history.tail(kFrameSize) = frame;
            }
            auto input = Ort::Value::CreateTensor<float>(memory, history.data(), history.size(), shape, 2);
            const auto result = reference.Run(Ort::RunOptions{nullptr}, inputs, &input, 1, outputs, 1);
            previous_action = Eigen::Map<const VecXf>(result[0].GetTensorData<float>(), kJoints);
            const auto action = runner.GetRobotAction(state, command);
            assert((runner.CommandVelocity() - expected_velocity).cwiseAbs().maxCoeff() < 1e-5f);
            if (step == forward_end - 1) assert(runner.CommandVelocity().x() == 3.f);
            if (step == reverse_end - 1) assert(runner.CommandVelocity().x() == -.25f);
            assert((action.goal_joint_pos - home - .25f * previous_action).cwiseAbs().maxCoeff() < 3e-4f);
            assert(action.kp.isApprox(kp) && action.kd.isApprox(kd));
            assert(action.goal_joint_vel.isZero() && action.tau_ff.isZero());
        }
        runner.OnExit();
    }
}

void CheckState(const std::string& path) {
    static_assert(RobotMotionState::MjlabControl_Walk2Run_Mode == 0x11018);
    assert(NormalizeMotionStateCommand(0x11018) == RobotMotionState::MjlabControl_Walk2Run_Mode);
    auto data = std::make_shared<ControllerData>();
    data->ri_ptr = std::make_shared<TestRobot>();
    data->uc_ptr = std::make_shared<TestUser>();
    auto& robot = *data->ri_ptr;
    auto& user = data->uc_ptr->command;
    user.target_mode = RobotMotionState::AmpControl_GunShootWalk_Mode;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_GunShootWalk_Mode, user));
    user.target_mode = RobotMotionState::AmpControl_NiRenWalk_Mode;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_NiRenWalk_Mode, user));
    user.target_mode = RobotMotionState::RLControl_Stand_Mode;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::RLControl_Stand_Mode, user));
    user.target_mode = RobotMotionCommand::StandElbowPlus12;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::RLControl_Stand_Mode, user));
    user.target_mode = RobotMotionState::AmpControl_Walk2Run_Mode;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_Walk2Run_Mode, user));
    UserCommand mismatched_pending = user;
    mismatched_pending.target_mode = RobotMotionState::RLControl_Stand_Mode;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_Walk2Run_Mode, mismatched_pending));
    assert(!CanRequestMjlabControlWalk2Run(RobotMotionState::WaitingForStand, user));
    assert(!CanRequestMjlabControlWalk2Run(RobotMotionState::MimicControl_Kneel_Mode, user));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::AmpControl_GunShootWalk_Mode));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::AmpControl_NiRenWalk_Mode));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::RLControl_Stand_Mode));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::AmpControl_Walk2Run_Mode));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::MimicControl_Kneel_Mode));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::MimicControl_ChairDown_Mode));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::ZeroPos));
    assert(IsMjlabControlWalk2RunExitMotionState(RobotMotionState::JointDamping));
    user.target_mode = RobotMotionState::AmpControl_NiRenWalk_Mode;
    user.safe_control_mode = 1;
    assert(!CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_GunShootWalk_Mode, user));
    user.safe_control_mode = 0;
    user.forward_vel_scale = .1f;
    assert(CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_GunShootWalk_Mode, user));
    user.target_mode = RobotMotionState::JointDamping;
    assert(!CanRequestMjlabControlWalk2Run(RobotMotionState::AmpControl_GunShootWalk_Mode, user));
    user.target_mode = RobotMotionState::ZeroPos;
    assert(!CanRequestMjlabControlWalk2Run(RobotMotionState::RLControl_Stand_Mode, user));

    robot.q = Home();
    robot.command = MatXf::Zero(kJoints, 5);
    robot.command.col(1) = robot.q;
    robot.command.col(0).setConstant(75.f);
    const MatXf entry = robot.command;
    StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControl_Stand_Mode);
    user.target_mode = RobotMotionState::MjlabControl_Walk2Run_Mode;
    humanoid_std::MjlabControlWalk2RunModeState state(RobotName::CR1STANDARD, "mjlab_control_walk2run_mode", data, path);
    state.OnEnter();
    assert(StateBase::msfb_.GetCurrentState() == RobotMotionState::RLControl_Stand_Mode);
    state.Run();
    assert(StateBase::msfb_.GetCurrentState() == RobotMotionState::MjlabControl_Walk2Run_Mode);
    assert(robot.command.isApprox(entry));
    user.forward_vel_scale = 1.f;
    for (int tick = 1; tick <= 1220; ++tick) {
        robot.time = tick * .001;
        state.Run();
        assert(state.GetNextStateName() == StateName::kMjlabControlWalk2RunMode);
    }
    const MatXf stationary = robot.command;
    user.forward_vel_scale = 0.f;
    for (int tick = 1221; tick <= 1240; ++tick) {
        robot.time = tick * .001;
        state.Run();
    }
    assert(robot.command.isApprox(stationary, 5e-3f));
    user.forward_vel_scale = 1.f;
    for (int tick = 1241; tick <= 1660; ++tick) {
        robot.time = tick * .001;
        state.Run();
    }
    user.target_mode = RobotMotionState::AmpControl_NiRenWalk_Mode;
    int exit_tick = 0;
    for (int tick = 1; tick <= 1000; ++tick) {
        robot.time = (1660 + tick) * .001;
        state.Run();
        if (state.GetNextStateName() == StateName::kAmpControlNiRenWalkMode) {
            exit_tick = tick;
            break;
        }
    }
    assert(exit_tick > 500 && exit_tick < 850);
    state.OnExit();

    user.target_mode = RobotMotionState::MjlabControl_Walk2Run_Mode;
    state.OnEnter();
    user.target_mode = RobotMotionState::JointDamping;
    const int publishes = robot.publishes;
    state.Run();
    assert(robot.publishes == publishes && state.GetNextStateName() == StateName::kJointDamping);
    state.OnExit();
    user.target_mode = RobotMotionState::MjlabControl_Walk2Run_Mode;
    StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControl_Stand_Mode);
    state.OnEnter();
    robot.q(0) = std::numeric_limits<float>::quiet_NaN();
    state.Run();
    assert(robot.publishes == publishes && state.GetNextStateName() == StateName::kJointDamping);
    assert(StateBase::msfb_.GetCurrentState() == RobotMotionState::RLControl_Stand_Mode);
    state.OnExit();

    struct ExitCase {
        uint32_t target;
        StateName expected;
    };
    const ExitCase exit_cases[] = {
        {RobotMotionState::RLControl_Stand_Mode, StateName::kRLControlStandMode},
        {RobotMotionState::AmpControl_Walk2Run_Mode, StateName::kAmpControlWalk2RunMode},
        {RobotMotionState::MimicControl_Kneel_Mode, StateName::kMimicControlKneelMode},
        {RobotMotionState::MimicControl_ChairDown_Mode, StateName::kMimicControlChairDownMode},
        {RobotMotionState::ZeroPos, StateName::kZeroPos},
    };
    robot.q = Home();
    for (const auto& exit_case : exit_cases) {
        user.target_mode = RobotMotionState::MjlabControl_Walk2Run_Mode;
        state.OnEnter();
        user.target_mode = exit_case.target;
        assert(state.GetNextStateName() == exit_case.expected);
        state.OnExit();
    }
}
}

int main(int argc, char** argv) {
    assert(argc == 2);
    CheckPolicy(argv[1]);
    CheckState(argv[1]);
    std::cout << "PASS: mjlab_control_walk2run_mode model contract, history and ramp parity, entry, decelerated exit and damping\n";
}
