#include "humanoid_std/mimic_control_kneel_mode_state.hpp"
#include "humanoid_std/mimic_control_kneel_stand_up_mode_state.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

static void Check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

static MatXf ArmBlend(const MatXf& entry_arms, const MatXf& policy, double elapsed) {
    MatXf result = policy;
    const float s = static_cast<float>(std::clamp(elapsed, 0.0, 1.0));
    const float w = s * s * (3.0f - 2.0f * s);
    result.block(1, 0, 8, 5) =
        (1.0f - w) * entry_arms + w * policy.block(1, 0, 8, 5);
    return result;
}

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "expected policy root");
        const auto root = std::filesystem::path(argv[1]);
        setenv("MIMIC_CONTROL_KNEEL_MODE_POLICY_DIR", (root / "dunxia_policy").c_str(), 1);
        setenv("MIMIC_CONTROL_KNEEL_STAND_UP_MODE_POLICY_DIR", (root / "zhanli_policy").c_str(), 1);
        static_assert(AmpControl_NiRenWalk_Mode == 0x11 &&
                      AmpControl_GunShootWalk_Mode == 0x1100B);
        static_assert(RLControl_Stand_Mode == 0x1100C && AmpControl_Walk2Run_Mode == 0x1100D);
        static_assert(MimicControl_ChairDown_Mode == 0x1100E && MimicControl_ChairUp_Mode == 0x1100F);
        static_assert(MimicControl_Kneel_Mode == 0x11010 &&
                      MimicControl_Kneel_StandUp_Mode == 0x11011);
        auto data = std::make_shared<ControllerData>();
        data->ri_ptr = std::make_shared<TestRobot>();
        data->uc_ptr = std::make_shared<TestUser>();
        data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
        auto& robot = *data->ri_ptr;
        auto& user = data->uc_ptr->command;
        humanoid_std::MimicControlKneelModeState down(
            CR1STANDARD, "mimic_control_kneel_mode", data);
        humanoid_std::MimicControlKneelStandUpModeState up(
            CR1STANDARD, "mimic_control_kneel_stand_up_mode", data);
        MimicControlKneelModePolicyRunner down_reference;
        MimicControlKneelStandUpModePolicyRunner up_reference;
        robot.q = down_reference.InitialJointPosition();
        robot.rpy = Vec3f(.01f, -.02f, .3f);
        RobotBasicState feedback{};
        feedback.base_rot_mat = RpyToRm(robot.rpy);
        feedback.base_omega = robot.omega;
        MatXf expected;
        // Repeat the complete chain to catch history, frame and heading leakage.
        for (int episode = 0; episode < 2; ++episode) {
            robot.command = MatXf::Constant(21, 5, .25f);
            robot.time = 10.0;
            StateBase::msfb_.UpdateCurrentState(episode == 0 ? AmpControl_NiRenWalk_Mode : AmpControl_Walk2Run_Mode);
            user.target_mode = MimicControl_Kneel_Mode;
            user.forward_vel_scale = 1;
            const VecXf initial = robot.q;
            const MatXf arm_entry = robot.command.block(1, 0, 8, 5);
            const double arm_entry_time = robot.time;
            down.OnEnter();
            Check(robot.q.isApprox(initial) && user.forward_vel_scale == 0, "squat entry changed pose or retained velocity");
            down_reference.OnEnter();
            down_reference.AlignHeading(feedback.base_rot_mat);
            const int ticks = down_reference.FrameCount() * 20;
            for (int tick = 0; tick < ticks + 2000; ++tick) {
                robot.time = arm_entry_time + tick * .001;
                robot.q(0) = initial(0) + .01f * std::sin(tick * .01f);
                feedback.joint_pos = robot.q;
                feedback.joint_vel = robot.dq;
                if (tick % 20 == 0) expected = down_reference.GetRobotAction(feedback, user).ConvertToMat();
                down.Run();
                const MatXf blended = ArmBlend(arm_entry, expected, robot.time - arm_entry_time);
                Check(robot.command.isApprox(blended, 1e-6f),
                      "squat arm-only entry or live inference mismatch");
                user.target_mode = MimicControl_Kneel_StandUp_Mode;
                const bool complete_period = tick + 1 >= ticks && (tick + 1) % 20 == 0;
                Check(down.GetNextStateName() == (complete_period ? kMimicControlKneelStandUpMode : kMimicControlKneelMode),
                      "stand-up accepted before complete final squat action period");
                user.target_mode = AmpControl_NiRenWalk_Mode;
                Check(down.GetNextStateName() == kMimicControlKneelMode, "squat switched directly to walking");
                user.target_mode = MimicControl_Kneel_Mode;
            }
            down.OnExit();
            user.target_mode = MimicControl_Kneel_StandUp_Mode;
            const VecXf seated = robot.q;
            const MatXf up_arm_entry = robot.command.block(1, 0, 8, 5);
            const double up_arm_entry_time = robot.time;
            up.OnEnter();
            Check(robot.q.isApprox(seated), "stand-up teleported joint feedback");
            up_reference.OnEnter();
            up_reference.AlignHeading(feedback.base_rot_mat);
            const int up_ticks = up_reference.FrameCount() * 20;
            for (int tick = 0; tick < up_ticks + 2000; ++tick) {
                robot.time = up_arm_entry_time + tick * .001;
                robot.q(0) = seated(0) + .01f * std::sin(tick * .01f);
                feedback.joint_pos = robot.q;
                feedback.joint_vel = robot.dq;
                if (tick % 20 == 0) expected = up_reference.GetRobotAction(feedback, user).ConvertToMat();
                up.Run();
                const MatXf blended = ArmBlend(up_arm_entry, expected, robot.time - up_arm_entry_time);
                Check(robot.command.isApprox(blended, 1e-6f),
                      "stand-up arm-only entry or live inference mismatch");
                const bool ready = tick + 1 >= up_ticks && (tick + 1) % 20 == 0;
                for (auto route : {std::pair<uint32_t, StateName>{AmpControl_NiRenWalk_Mode, kRLControlStandMode},
                                   {AmpControl_GunShootWalk_Mode, kRLControlStandMode},
                                   {RLControl_Stand_Mode, kRLControlStandMode}, {AmpControl_Walk2Run_Mode, kAmpControlWalk2RunMode},
                                   {MimicControl_Kneel_Mode, kMimicControlKneelMode}, {MimicControl_ChairDown_Mode, kMimicControlChairDownMode}}) {
                    user.target_mode = route.first;
                    Check(up.GetNextStateName() == (ready ? route.second : kMimicControlKneelStandUpMode), "wrong stand-up exit gate");
                }
                user.target_mode = MimicControl_Kneel_StandUp_Mode;
            }
            up.OnExit();
        }
        // MimicControl_Kneel_StandUp_Mode can never be entered from another standing/seated/basic state.
        for (uint32_t source : {uint32_t(AmpControl_NiRenWalk_Mode),
                                uint32_t(RLControl_Stand_Mode), uint32_t(AmpControl_Walk2Run_Mode),
                                uint32_t(MimicControl_ChairUp_Mode), uint32_t(MimicControl_ChairDown_Mode), uint32_t(WaitingForStand)}) {
            StateBase::msfb_.UpdateCurrentState(source);
            up.OnEnter();
            const int before = robot.publishes;
            up.Run();
            Check(robot.publishes == before && up.GetNextStateName() == kJointDamping,
                  "stand-up accepted a source other than MimicControl_Kneel_Mode");
        }
        for (int fault = 0; fault < 3; ++fault) {
            StateBase::msfb_.UpdateCurrentState(AmpControl_NiRenWalk_Mode);
            user.target_mode = MimicControl_Kneel_Mode;
            user.safe_control_mode = 0;
            down.OnEnter();
            if (fault == 0) user.safe_control_mode = 1;
            if (fault == 1) user.target_mode = JointDamping;
            if (fault == 2) robot.q(0) = std::numeric_limits<float>::quiet_NaN();
            const int before = robot.publishes;
            down.Run();
            Check(robot.publishes == before && down.GetNextStateName() == kJointDamping,
                  "squat safety did not override pending action");
            robot.q(0) = 0;
        }
        user.safe_control_mode = 0;
        user.target_mode = MimicControl_Kneel_Mode;
        StateBase::msfb_.UpdateCurrentState(AmpControl_NiRenWalk_Mode);
        down.OnEnter();
        user.target_mode = ZeroPos;
        const int before = robot.publishes;
        down.Run();
        Check(robot.publishes == before && down.GetNextStateName() == kZeroPos, "squat zero exit blocked");

        // RL -> MimicControl_Kneel_Mode blends arms only; trunk and legs consume frame zero directly.
        robot.q = down_reference.InitialJointPosition();
        robot.dq.setZero();
        robot.rpy.setZero();
        robot.time += 10.0;
        const MatXf entry = MatXf::Constant(21, 5, .25f);
        robot.command = entry;
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::AmpControl_NiRenWalk_Mode);
        user.safe_control_mode = 0;
        user.target_mode = MimicControl_Kneel_Mode;
        feedback.joint_pos = robot.q;
        feedback.joint_vel = robot.dq;
        feedback.base_rot_mat = RpyToRm(robot.rpy);
        feedback.base_omega = robot.omega;
        down_reference.OnEnter();
        down_reference.AlignHeading(feedback.base_rot_mat);
        expected = down_reference.GetRobotAction(feedback, user).ConvertToMat();
        down.OnEnter();
        down.Run();
        Check(robot.command.block(1, 0, 8, 5).isApprox(entry.block(1, 0, 8, 5), 1e-6f) &&
              robot.command.topRows(1).isApprox(expected.topRows(1), 1e-6f) &&
              robot.command.bottomRows(12).isApprox(expected.bottomRows(12), 1e-6f),
              "squat entry must hold previous arms and use live trunk/legs");
        robot.time += 1.0;
        down.Run();
        Check(robot.command.isApprox(expected, 1e-6f),
              "squat arm blend did not finish after one second");
        down.OnExit();
        std::cout << "PASS: 2 full squat/rise cycles, arm-only mimic entry, live final hold, 50 Hz timing, source/exit gates and safety\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
