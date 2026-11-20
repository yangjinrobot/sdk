#include "humanoid_std/restand_state.hpp"
#include "json.hpp" // 测试进程通过 stdin 传递参考数据；策略本身不依赖 JSON。
#include <limits>

static void Check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
static VecXf Vector(const nlohmann::json& value) {
    auto data = value.get<std::vector<float>>();
    return Eigen::Map<const VecXf>(data.data(), data.size());
}
int main(int argc, char** argv) {
    try {
        Check(argc == 2, "expected ONNX path");
        static_assert(RobotMotionState::Restand == 0x1100B && StateName::kRestand == 0x1100B);
        RLStandPolicyRunner policy(RobotName::CR1STANDARD, argv[1]);
        Check(policy.decimation_ == 20, "wrong control decimation");
        nlohmann::json input;
        std::cin >> input;
        const std::array<int, 21> mapping{{9,10,11,12,13,14,15,16,17,18,19,20,0,1,2,3,4,5,6,7,8}};
        const VecXf defaults = Vector(input["config"]["default_angles"]);
        const VecXf kp = Vector(input["config"]["kp"]), kd = Vector(input["config"]["kd"]);
        float max_error = 0;
        for (const auto& sequence : input["sequences"]) {
            policy.OnEnter();
            for (const auto& frame : sequence) {
                RobotBasicState robot;
                robot.joint_pos = Vector(frame["q"]);
                robot.joint_vel = Vector(frame["dq"]);
                robot.base_omega = Vector(frame["omega"]);
                robot.base_rpy = Vector(frame["rpy"]);
                robot.base_rot_mat = RpyToRm(robot.base_rpy);
                UserCommand command{};
                command.forward_vel_scale = frame["command"];
                command.side_vel_scale = 1;
                command.turnning_vel_scale = -1;
                const auto result = policy.GetRobotAction(robot, command);
                max_error = std::max(max_error, (policy.History() - Vector(frame["history"])).cwiseAbs().maxCoeff());
                Check(policy.History().isApprox(Vector(frame["history"]), 2e-5f), "Python/C++ history mismatch");
                Check(policy.Action().isApprox(Vector(frame["action"]), 3e-5f), "Python/C++ action mismatch");
                for (int i = 0; i < 21; ++i) {
                    const float expected = (i == 16 || i == 20) ? -0.4f :
                        defaults(i) + (i < 13 ? 0.25f * frame["action"][i].get<float>() : 0);
                    Check(std::fabs(result.goal_joint_pos(mapping[i]) - expected) < 2e-5f, "joint target mapping mismatch");
                    Check(result.kp(mapping[i]) == kp(i) && result.kd(mapping[i]) == kd(i), "PD mapping mismatch");
                }
            }
        }
        auto data = std::make_shared<ControllerData>();
        data->ri_ptr = std::make_shared<TestRobot>();
        data->uc_ptr = std::make_shared<TestUser>();
        data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
        auto& robot = *data->ri_ptr;
        auto& user = data->uc_ptr->command;
        for (int i = 0; i < 21; ++i) robot.q(mapping[i]) = defaults(i);
        robot.q.segment(1, 8).array() += .2f;
        // Last RL target deliberately differs from the measured arm pose.
        robot.command = MatXf::Zero(21, 5);
        robot.command.col(0).setConstant(100.f);
        robot.command.col(1) = robot.q.array() + .1f;
        robot.command.col(2).setConstant(5.f);
        robot.command.col(3).setConstant(.02f);
        robot.command.col(4).setConstant(.3f);
        const MatXf entry_arms = robot.command.block(1, 0, 8, 5);
        humanoid_std::RestandState state(RobotName::CR1STANDARD, "restand", data, argv[1]);
        user.target_mode = RobotMotionState::Restand;
        user.forward_vel_scale = 1;
        state.OnEnter();
        Check(user.forward_vel_scale == 0, "entry retained velocity command");
        Check(StateBase::msfb_.GetCurrentState() == RobotMotionState::Restand, "missing state feedback");
        RLStandPolicyRunner stationary(RobotName::CR1STANDARD, argv[1]);
        RobotBasicState feedback;
        feedback.joint_pos = robot.q;
        feedback.joint_vel = robot.dq;
        feedback.base_rot_mat = RpyToRm(robot.rpy);
        feedback.base_omega = robot.omega;
        MatXf stationary_command;
        MatXf first, previous;
        for (int tick = 0; tick <= 2100; ++tick) {
            robot.time = tick * .001;
            // 模拟手柄持续刷新，并在运行中改变摇杆方向。
            user.forward_vel_scale = tick < 1000 ? 1 : -1;
            user.side_vel_scale = -1;
            user.turnning_vel_scale = 1;
            state.Run();
            if (tick % 20 == 0)
                stationary_command = stationary.GetRobotAction(feedback, UserCommand{}).ConvertToMat();
            for (int joint : {0, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20})
                Check(robot.command.row(joint).isApprox(stationary_command.row(joint), 1e-6f),
                      "restand accepted joystick velocity");
            if (tick == 0) {
                first = robot.command;
                Check(robot.command.block(1, 0, 8, 5).isApprox(entry_arms), "arm entry jumped");
            }
            if (tick % 20 != 0) {
                Check(robot.command.block(9, 1, 12, 1).isApprox(previous.block(9, 1, 12, 1)), "wrong inference timing");
            }
            const float s = std::min(1.0f, tick / 1000.0f), weight = s*s*(3-2*s);
            const MatXf expected_arms = (1-weight)*entry_arms + weight*stationary_command.block(1, 0, 8, 5);
            Check(robot.command.block(1, 0, 8, 5).isApprox(expected_arms, 1e-6f),
                  "arm position/PD/velocity/torque blend mismatch");
            if (tick == 0) {
                for (int i = 0; i < 8; ++i) {
                    const float old_tau = entry_arms(i,0)*(entry_arms(i,1)-robot.q(1+i)) +
                        entry_arms(i,2)*(entry_arms(i,3)-robot.dq(1+i)) + entry_arms(i,4);
                    const auto row = robot.command.row(1+i);
                    const float new_tau = row(0)*(row(1)-robot.q(1+i)) +
                        row(2)*(row(3)-robot.dq(1+i)) + row(4);
                    Check(std::fabs(old_tau-new_tau) < 1e-6f, "entry lost PD support torque");
                }
            }
            previous = robot.command;
        }
        state.OnExit();
        robot.command = first;
        state.OnEnter();
        state.Run();
        Check(robot.command.isApprox(first, 1e-6f), "re-entry retained policy history");
        state.OnExit();
        // Return to RL to catch a stale target across consecutive entries.
        for (uint32_t source : {uint32_t(RobotMotionState::RLControlMode),
                                uint32_t(RobotMotionState::Sit2StandMimic),
                                uint32_t(RobotMotionState::Run),
                                uint32_t(RobotMotionState::RLControlMode)}) {
            const float elbow = -.4f;
            StateBase::msfb_.UpdateCurrentState(source);
            const MatXf entry = robot.command.block(1, 0, 8, 5);
            user.target_mode = RobotMotionState::Restand;
            robot.time = 0;
            stationary.OnEnter();
            state.OnEnter();
            Check(StateBase::msfb_.GetLastState() == source, "source feedback lost");
            for (int tick = 0; tick <= 1200; ++tick) {
                robot.time = tick * .001;
                state.Run();
                if (tick % 20 == 0) {
                    stationary_command = stationary.GetRobotAction(feedback, UserCommand{}).ConvertToMat();
                    stationary_command(4, 1) = stationary_command(8, 1) = elbow;
                }
                const float s = std::min(1.f, tick / 1000.f), w = s*s*(3-2*s);
                MatXf expected = stationary_command;
                expected.block(1, 0, 8, 5) = (1-w)*entry + w*stationary_command.block(1, 0, 8, 5);
                Check(robot.command.isApprox(expected, 1e-6f), "source posture or smooth transition mismatch");
                if (tick >= 1000)
                    Check(robot.command(4, 1) == elbow && robot.command(8, 1) == elbow,
                          "stand failed to hold source-selected elbow posture");
            }
            state.OnExit();
        }
        // Stay in Stand while receiving the same DDS command every millisecond.
        // Retarget midway and on a tick between inferences: elbow positions must
        // start from the last published command without disturbing other joints.
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        user.target_mode = RobotMotionState::Restand;
        robot.time = 0;
        state.OnEnter();
        stationary.OnEnter();
        int pose_tick = 0;
        auto run_tick = [&] {
            robot.time = pose_tick * .001;
            if (pose_tick % 20 == 0)
                stationary_command = stationary.GetRobotAction(feedback, UserCommand{}).ConvertToMat();
            state.Run();
            ++pose_tick;
        };
        for (int i = 0; i < 1213; ++i) run_tick();
        for (auto request : {std::pair<uint32_t, int>{StandElbowMinus04, 1201},
                             {StandElbowPlus12, 401}, {StandElbowMinus04, 301},
                             {StandElbowPlus12, 1201}}) {
            const float target = request.first == StandElbowPlus12 ? 1.2f : -.4f;
            const Vec2f entry(robot.command(4, 1), robot.command(8, 1));
            for (int i = 0; i < request.second; ++i) {
                user.target_mode = request.first;
                run_tick();
                const float s = std::min(1.f, i / 1000.f), w = s*s*(3-2*s);
                for (int side = 0; side < 2; ++side) {
                    const int joint = side == 0 ? 4 : 8;
                    Check(std::fabs(robot.command(joint, 1) - ((1-w)*entry(side) + w*target)) < 1e-6f,
                          "elbow DDS command jumped, restarted or missed target");
                }
                for (int joint = 0; joint < 21; ++joint)
                    if (joint != 4 && joint != 8)
                        Check(robot.command.row(joint).isApprox(stationary_command.row(joint), 1e-6f),
                              "elbow command changed another joint");
                Check(state.GetNextStateName() == kRestand &&
                      StateBase::msfb_.GetCurrentState() == RobotMotionState::Restand,
                      "elbow command left Stand");
            }
        }
        state.OnExit();
        for (auto pair : {std::pair<uint32_t, StateName>{RobotMotionState::RLControlMode, kRLControl},
                          {RobotMotionState::Run, kRun}, {RobotMotionState::Stand2SitMimic, kStand2SitMimic},
                          {RobotMotionState::ZeroPos, kZeroPos}, {RobotMotionState::JointDamping, kJointDamping}}) {
            user.target_mode = pair.first;
            Check(state.GetNextStateName() == pair.second, "exit route missing");
        }
        for (int mode : {0, 1, 2}) {
            user.safe_control_mode = 0;
            user.target_mode = RobotMotionState::Restand;
            state.OnEnter();
            if (mode == 0) user.safe_control_mode = 1;
            if (mode == 1) user.target_mode = RobotMotionState::JointDamping;
            if (mode == 2) robot.q(0) = std::numeric_limits<float>::quiet_NaN();
            int publishes = robot.publishes;
            state.Run();
            Check(robot.publishes == publishes && state.GetNextStateName() == kJointDamping, "fault published invalid action");
        }
        bool rejected = false;
        try { RLStandPolicyRunner pro(RobotName::CR1PRO, argv[1]); }
        catch (const std::exception&) { rejected = true; }
        Check(rejected, "Pro robot accepted STD actor");
        std::cout << "PASS: Python/C++ ONNX agreement (max history error=" << max_error
                  << "), joint mapping, timing, arm blending, reset, exits, fault priority\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
