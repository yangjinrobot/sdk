// Uses real 1106 ONNX inference with isolated robot/user adapters.
#include "humanoid_std/run3_state.hpp"
#include <cassert>
#include <limits>
#include <string>

int main(int argc, char** argv) {
    assert(argc == 2);
    assert(std::string(humanoid_std::Run3State::kDefaultPolicyPath) ==
           "/opt/robot/share/motion_master/policy/1106/run_1106.onnx");
    auto data = std::make_shared<ControllerData>();
    data->ri_ptr = std::make_shared<TestRobot>();
    data->uc_ptr = std::make_shared<TestUser>();
    data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
    auto& robot = *data->ri_ptr;
    auto& user = data->uc_ptr->command;
    humanoid_std::Run3State run3(RobotName::CR1STANDARD, "run_3", data, argv[1]);

    for (uint32_t source : {uint32_t(RobotMotionState::RLControlMode),
                            uint32_t(RobotMotionState::NirenWalk),
                            uint32_t(RobotMotionState::Restand),
                            uint32_t(RobotMotionState::Run),
                            uint32_t(RobotMotionState::Run2),
                            uint32_t(RobotMotionState::Run4),
                            uint32_t(RobotMotionState::Sit2StandMimic)}) {
        StateBase::msfb_.UpdateCurrentState(source);
        robot.command = MatXf::Constant(21, 5, 2.f);
        user = UserCommand{};
        user.target_mode = RobotMotionState::Run3;
        const auto start = ArmEntryTransition::Clock::now();
        run3.OnEnter();
        assert(StateBase::msfb_.GetCurrentState() == RobotMotionState::Run3);
        const int before = robot.publishes;
        for (int tick = 0; tick < 45; ++tick) run3.Run();
        assert(robot.publishes == before + 45 && robot.command.allFinite());
        const float s = std::clamp(
            float(std::chrono::duration<double>(ArmEntryTransition::Clock::now() - start).count()),
            0.f, 1.f);
        assert((robot.command.block(1, 0, 8, 5).array() - 2.f).abs().maxCoeff() <=
               (s * s * (3 - 2 * s)) * 200.f + 1e-4f);
        assert(run3.GetNextStateName() == StateName::kRun3);
        for (const auto& route : std::vector<std::pair<uint32_t, StateName>>{
                 {RobotMotionState::RLControlMode, StateName::kRLControl},
                 {RobotMotionState::NirenWalk, StateName::kNirenWalk},
                 {RobotMotionState::Run, StateName::kRun},
                 {RobotMotionState::Run2, StateName::kRun2},
                 {RobotMotionState::Run4, StateName::kRun4},
                 {RobotMotionState::Restand, StateName::kRestand},
                 {RobotMotionState::Stand2SitMimic, StateName::kStand2SitMimic},
                 {RobotMotionState::ZeroPos, StateName::kZeroPos}}) {
            user.target_mode = route.first;
            assert(run3.GetNextStateName() == route.second);
        }
        user.safe_control_mode = 1;
        const int stopped = robot.publishes;
        run3.Run();
        assert(robot.publishes == stopped && run3.GetNextStateName() == StateName::kJointDamping);
        user.safe_control_mode = 0;
        user.target_mode = RobotMotionState::JointDamping;
        run3.Run();
        assert(robot.publishes == stopped && run3.GetNextStateName() == StateName::kJointDamping);
        run3.OnExit();
    }

    user = UserCommand{};
    user.target_mode = RobotMotionState::Run3;
    robot.command = MatXf::Constant(21, 5, 2.f);
    run3.OnEnter();
    robot.q(0) = std::numeric_limits<float>::quiet_NaN();
    int before = robot.publishes;
    run3.Run();
    assert(robot.publishes == before && run3.GetNextStateName() == StateName::kJointDamping);
    robot.q.setZero();
    robot.command(0, 0) = std::numeric_limits<float>::quiet_NaN();
    run3.OnEnter();
    before = robot.publishes;
    run3.Run();
    assert(robot.publishes == before && run3.GetNextStateName() == StateName::kJointDamping);
    std::cout << "PASS: Run3 inference, arm entry, re-entry, routes, safety stop and invalid feedback\n";
}
