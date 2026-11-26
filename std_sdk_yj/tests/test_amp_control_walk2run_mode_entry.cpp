// Build with tests/mimic_stubs before the production state include path.
#include "humanoid_std/amp_control_walk2run_mode_state.hpp"
#include <cassert>
#include <limits>

int main(int argc, char** argv) {
    assert(argc == 2);
    // Deterministically check all five arm columns, the endpoint, and re-entry.
    ArmEntryTransition blend;
    const auto start = ArmEntryTransition::Clock::now();
    const MatXf previous = MatXf::Constant(8, 5, 2.f);
    for (int episode = 0; episode < 2; ++episode) {
        const MatXf entry = previous * float(episode + 1);
        blend.Reset(entry, start);
        for (int ms : {0, 250, 500, 1000, 1200}) {
            MatXf output = MatXf::Constant(21, 5, 10.f);
            blend.Apply(output, 1, start + std::chrono::milliseconds(ms));
            const float s = std::min(ms / 1000.f, 1.f);
            const float w = s * s * (3 - 2 * s);
            const MatXf expected = (1 - w) * entry + w * MatXf::Constant(8, 5, 10.f);
            assert(output.block(1, 0, 8, 5).isApprox(expected));
            assert(output.row(0).isConstant(10.f));
            assert(output.bottomRows(12).isConstant(10.f));
        }
    }
    auto data = std::make_shared<ControllerData>();
    data->ri_ptr = std::make_shared<TestRobot>();
    data->uc_ptr = std::make_shared<TestUser>();
    data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
    auto& robot = *data->ri_ptr;
    auto& user = data->uc_ptr->command;
    humanoid_std::AmpControlWalk2RunModeState state(RobotName::CR1STANDARD, "run", data, argv[1]);
    for (int source : {int(RobotMotionState::AmpControl_NiRenWalk_Mode),
                       int(RobotMotionState::AmpControl_GunShootWalk_Mode),
                       int(RobotMotionState::RLControl_Stand_Mode)}) {
        StateBase::msfb_.UpdateCurrentState(source);
        robot.command = MatXf::Constant(
            21, 5, source == RobotMotionState::RLControl_Stand_Mode ? 3.f : 2.f);
        const MatXf entry = robot.command.block(1, 0, 8, 5);
        user.target_mode = RobotMotionState::AmpControl_Walk2Run_Mode;
        const auto before = ArmEntryTransition::Clock::now();
        state.OnEnter();
        state.Run();
        const double seconds = std::chrono::duration<double>(ArmEntryTransition::Clock::now() - before).count();
        const float s = float(std::clamp(seconds, 0., 1.));
        const float max_weight = s * s * (3 - 2 * s);
        // Policy arms have kp <=100; all other columns are below this bound.
        assert((robot.command.block(1, 0, 8, 5) - entry).cwiseAbs().maxCoeff()
               <= max_weight * 200.f + 1e-4f);
        assert(state.GetNextStateName() == StateName::kAmpControlWalk2RunMode);
        state.OnExit();
    }
    robot.command(0, 0) = std::numeric_limits<float>::quiet_NaN();
    state.OnEnter();
    const int before = robot.publishes;
    state.Run();
    assert(robot.publishes == before);
    assert(state.GetNextStateName() == StateName::kJointDamping);
    std::cout << "PASS: Run entry from RL/RLControl_Stand_Mode, five-column arm blend, re-entry and invalid command\n";
}
