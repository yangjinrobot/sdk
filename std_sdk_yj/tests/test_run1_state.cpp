// Real-model regression for the disabled Run1 controller.
#include "humanoid_std/run1_state.hpp"
#include <cassert>
#include <limits>

int main(int argc, char** argv) {
    assert(argc == 2);
    auto data = std::make_shared<ControllerData>();
    data->ri_ptr = std::make_shared<TestRobot>();
    data->uc_ptr = std::make_shared<TestUser>();
    data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
    auto& robot = *data->ri_ptr;
    auto& user = data->uc_ptr->command;
    humanoid_std::Run1State run1(RobotName::CR1STANDARD, "run_1", data, argv[1]);

    user.target_mode = RobotMotionState::Run1;
    robot.command = MatXf::Constant(21, 5, 2.f);
    run1.OnEnter();
    assert(StateBase::msfb_.GetCurrentState() == RobotMotionState::Run1);
    const int before = robot.publishes;
    for (int tick = 0; tick < 45; ++tick) run1.Run();
    assert(robot.publishes == before + 45 && robot.command.allFinite());
    assert(run1.GetNextStateName() == StateName::kRun1);

    user.safe_control_mode = 1;
    const int stopped = robot.publishes;
    run1.Run();
    assert(robot.publishes == stopped && run1.GetNextStateName() == StateName::kJointDamping);
    user.safe_control_mode = 0;
    user.target_mode = RobotMotionState::JointDamping;
    run1.Run();
    assert(robot.publishes == stopped && run1.GetNextStateName() == StateName::kJointDamping);
    run1.OnExit();

    user = UserCommand{};
    user.target_mode = RobotMotionState::Run1;
    robot.command = MatXf::Constant(21, 5, 2.f);
    run1.OnEnter();
    robot.q(0) = std::numeric_limits<float>::quiet_NaN();
    const int invalid = robot.publishes;
    run1.Run();
    assert(robot.publishes == invalid && run1.GetNextStateName() == StateName::kJointDamping);
    std::cout << "PASS: Run1 legacy inference, state feedback, safety stop and invalid feedback\n";
}
