#include "humanoid_std/sit2stand_mimic_state.hpp"
#include "humanoid_std/squat_preparation_state.hpp"
#include "humanoid_std/stand2sit_mimic_state.hpp"
#include <limits>

static void Check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

static void CheckPreparedRise(const std::shared_ptr<ControllerData>& data,
                              humanoid_std::Sit2StandMimicState& rise,
                              Sit2StandMimicPolicyRunner& reference) {
    humanoid_std::SquatPreparationState squat(RobotName::CR1STANDARD, "squat_preparation", data);
    auto& robot = *data->ri_ptr;
    auto& user = data->uc_ptr->command;
    auto& feedback = StateBase::msfb_;
    const uint32_t ready = MotionStateFeedback::kHandsetReadyFlag;
    const uint32_t prep = RobotMotionState::SquatPreparation;
    feedback.UpdateCurrentState(RobotMotionState::WaitingForStand);
    user.target_mode = prep;
    user.safe_control_mode = 0;
    robot.q = VecXf::Constant(21, .05f);
    robot.dq.setZero();
    robot.rpy.setZero();
    robot.omega.setZero();
    robot.time = 30;
    squat.OnEnter();
    squat.Run();
    Check(robot.command.col(1).isApprox(robot.q) && robot.command.col(0).isZero(),
          "Squat preparation must ramp from live feedback and zero gain");
    Check(feedback.GetHandsetStatus() == prep && squat.GetNextStateName() == kSquatPreparation,
          "Preparation status announced readiness before motion");

    const VecXf pro = (VecXf(29) << 0.f, 0.f, 0.f,
        -.296706f, .122173f, -.122173f, .558505f, -.226893f, -.052360f, .191986f,
        -.383972f, -.261799f, .069813f, .575959f, .139626f, -.017453f, -.226893f,
        -1.431170f, 0.f, 0.f, 1.884956f, -.349066f, 0.f,
        -1.431170f, 0.f, 0.f, 1.884956f, -.349066f, 0.f).finished();
    VecXf standard(21);
    standard << pro(0), pro.segment(3, 4), pro.segment(10, 4), pro.tail(12);
    robot.time = 31;
    squat.Run();
    Check(robot.command.col(1).isApprox(standard, 1e-6f), "Wrong 29-to-21 joint mapping");
    const VecXf effort = robot.command.col(0).cwiseProduct(robot.command.col(1) - robot.q) +
        robot.command.col(2).cwiseProduct(robot.command.col(3) - robot.dq) + robot.command.col(4);
    Check((effort.cwiseAbs() - data->cp_ptr->torque_limit_).maxCoeff() <= .001f,
          "Preparation must respect combined effort limits");
    user.target_mode = RobotMotionState::Sit2StandMimic;
    Check(squat.GetNextStateName() == kSquatPreparation, "Rise accepted before measured arrival");
    user.target_mode = prep;
    robot.q = standard;
    robot.dq(0) = .31f;
    robot.time = 31.2;
    squat.Run();
    Check(feedback.GetHandsetStatus() == prep, "Moving joints reported ready");
    robot.dq.setZero();
    squat.Run();
    robot.time = 31.3;
    squat.Run();
    Check(feedback.GetHandsetStatus() == prep, "Readiness skipped the settle interval");
    robot.time = 31.4;
    squat.Run();
    Check(feedback.GetHandsetStatus() == (prep | ready) && squat.GetNextStateName() == kSquatPreparation,
          "Stable squat must hold until confirmed");
    robot.q(9) += .2f;
    squat.Run();
    Check(feedback.GetHandsetStatus() == prep, "Loss of posture did not clear readiness");
    robot.q = standard;
    squat.Run();
    robot.time = 31.6;
    squat.Run();
    user.target_mode = RobotMotionState::Sit2StandMimic;
    Check(squat.GetNextStateName() == kSit2StandMimic, "Confirmed stable squat failed to start rise");
    user.safe_control_mode = 1;
    Check(squat.GetNextStateName() == kJointDamping, "Fault must override rise confirmation");
    user.safe_control_mode = 0;
    squat.OnExit();
    Check(feedback.GetHandsetStatus() == prep, "Preparation readiness leaked to the next state");

    rise.OnEnter();
    RobotBasicState observation{};
    observation.joint_pos = standard;
    observation.joint_vel = robot.dq;
    observation.base_rot_mat = Mat3f::Identity();
    observation.base_omega = robot.omega;
    reference.OnEnter();
    reference.AlignHeading(observation.base_rot_mat);
    const MatXf first = reference.GetRobotAction(observation, user).ConvertToMat();
    user.target_mode = RobotMotionState::RLControlMode;
    const int ticks = reference.FrameCount() * reference.decimation_;
    for (int i = 0; i < ticks; ++i) {
        rise.Run();
        if (i == 0) Check(robot.command.isApprox(first, 1e-6f), "Prepared rise skipped the first ONNX frame");
        Check(rise.GetNextStateName() == (i + 1 == ticks ? kRLControl : kSit2StandMimic),
              "Prepared rise must play the full clip before walking");
    }
    Check(feedback.GetHandsetStatus() == (RobotMotionState::Sit2StandMimic | ready),
          "Completed rise did not report readiness");
    rise.OnExit();
}

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "expected sit2stand policy directory");
        setenv("SIT2STAND_MIMIC_POLICY_DIR", argv[1], 1);
        const auto sitting = std::filesystem::path(argv[1]).parent_path() / "stand2sit_policy";
        setenv("STAND2SIT_MIMIC_POLICY_DIR", sitting.c_str(), 1);
        auto data = std::make_shared<ControllerData>();
        data->ri_ptr = std::make_shared<TestRobot>();
        data->uc_ptr = std::make_shared<TestUser>();
        data->cp_ptr = std::make_shared<ControlParameters>(RobotName::CR1STANDARD);
        auto& robot = *data->ri_ptr;
        auto& user = data->uc_ptr->command;
        humanoid_std::Stand2SitMimicState down(RobotName::CR1STANDARD, "stand2sit", data);
        humanoid_std::Sit2StandMimicState up(RobotName::CR1STANDARD, "sit2stand", data);
        Stand2SitMimicPolicyRunner down_reference;
        Sit2StandMimicPolicyRunner reference;
        robot.q = down_reference.InitialJointPosition();
        robot.dq.setZero();
        robot.rpy.setZero();
        down.OnEnter();
        user.target_mode = RobotMotionState::Sit2StandMimic;
        Check(down.GetNextStateName() == kStand2SitMimic, "Y must wait for sitting completion");
        const int down_ticks = down_reference.FrameCount() * 20;
        for (int tick = 0; tick < down_ticks; ++tick) {
            down.Run();
            Check(down.GetNextStateName() == (tick + 1 == down_ticks ? kSit2StandMimic : kStand2SitMimic),
                  "Y must wait until final seated action has run for 20 ms");
        }
        down.OnExit();
        // Test live feedback and yaw alignment, distinct from the standing offset.
        robot.q = reference.InitialJointPosition();
        robot.rpy = Vec3f(.01f, -.02f, .3f);
        const VecXf initial = robot.q;
        RobotBasicState feedback{};
        feedback.base_rot_mat = RpyToRm(robot.rpy);
        feedback.base_omega = robot.omega;
        reference.AlignHeading(feedback.base_rot_mat);
        user.forward_vel_scale = .5f;
        user.side_vel_scale = -.3f;
        user.turnning_vel_scale = .2f;
        up.OnEnter();
        Check(robot.q.isApprox(initial), "entry changed measured pose");
        Check(user.forward_vel_scale == 0 && user.side_vel_scale == 0 && user.turnning_vel_scale == 0,
              "entry retained walking velocity");
        Check(StateBase::msfb_.GetCurrentState() == RobotMotionState::Sit2StandMimic, "state feedback missing");
        MatXf expected, first;
        const int up_ticks = reference.FrameCount() * 20;
        const int switch_ticks = std::min(200, reference.FrameCount()) * 20;
        for (int tick = 0; tick < up_ticks + 200; ++tick) {
            robot.q[0] = initial[0] + .01f * std::sin(tick * .01f);
            feedback.joint_pos = robot.q;
            feedback.joint_vel = robot.dq;
            if (tick % 20 == 0) expected = reference.GetRobotAction(feedback, user).ConvertToMat();
            up.Run();
            Check(robot.command.isApprox(expected, 1e-6f), "wrong live action, 50 Hz timing or final hold");
            if (tick == 0) first = robot.command;
            // Query a pending C, then retain the state to verify live final hold.
            user.target_mode = RobotMotionState::RLControlMode;
            const bool ready = tick + 1 >= switch_ticks && (tick + 1) % 20 == 0;
            Check(up.GetNextStateName() == (ready ? kRLControl : kSit2StandMimic),
                  "C must wait for 200 complete frame periods");
            user.target_mode = RobotMotionState::Run;
            Check(up.GetNextStateName() == (ready ? kRun : kSit2StandMimic),
                  "V (Run) must wait for 200 complete frame periods");
            user.target_mode = RobotMotionState::Restand;
            Check(up.GetNextStateName() == (ready ? kRestand : kSit2StandMimic),
                  "restand must wait for 200 complete frame periods");
            user.target_mode = RobotMotionState::Stand2SitMimic;
            Check(up.GetNextStateName() == (ready ? kStand2SitMimic : kSit2StandMimic),
                  "T must wait for 200 complete frame periods");
            if (ready) {
                user.safe_control_mode = 1;
                Check(up.GetNextStateName() == kJointDamping, "fault must override pending T");
                user.safe_control_mode = 0;
            }
            user.target_mode = RobotMotionState::Sit2StandMimic;
            Check(up.GetNextStateName() == kSit2StandMimic, "hold must not start walking automatically");
        }
        up.OnExit();
        robot.q = initial;
        up.OnEnter();
        user.target_mode = RobotMotionState::RLControlMode;
        Check(up.GetNextStateName() == kSit2StandMimic, "re-entry retained transition eligibility");
        user.target_mode = RobotMotionState::Sit2StandMimic;
        up.Run();
        Check(robot.command.isApprox(first, 1e-6f), "re-entry retained reference/action history");
        for (int mode : {int(RobotMotionState::ZeroPos), int(RobotMotionState::JointDamping), -1}) {
            user.target_mode = RobotMotionState::Sit2StandMimic;
            user.safe_control_mode = 0;
            up.OnEnter();
            if (mode == -1) user.safe_control_mode = 1;
            else user.target_mode = mode;
            const int before = robot.publishes;
            up.Run();
            Check(robot.publishes == before, "exit published another mimic action");
            Check(up.GetNextStateName() == (mode == RobotMotionState::ZeroPos ? kZeroPos : kJointDamping),
                  "Z/R/fault failed to override motion");
        }
        user.safe_control_mode = 0;
        user.target_mode = RobotMotionState::Sit2StandMimic;
        up.OnEnter();
        robot.q[0] = std::numeric_limits<float>::quiet_NaN();
        up.Run();
        Check(up.GetNextStateName() == kJointDamping, "invalid feedback accepted");

        // Repeat the same cancellation/failure tests for stand2sit.
        robot.q = down_reference.InitialJointPosition();
        for (int mode : {int(RobotMotionState::ZeroPos), int(RobotMotionState::JointDamping), -1}) {
            user.target_mode = RobotMotionState::Stand2SitMimic;
            user.safe_control_mode = 0;
            down.OnEnter();
            if (mode == -1) user.safe_control_mode = 1;
            else user.target_mode = mode;
            const int before = robot.publishes;
            down.Run();
            Check(robot.publishes == before, "stand2sit exit published a command");
            Check(down.GetNextStateName() == (mode == RobotMotionState::ZeroPos ? kZeroPos : kJointDamping),
                  "stand2sit cancellation/fault transition failed");
        }
        user.safe_control_mode = 0;
        user.target_mode = RobotMotionState::Stand2SitMimic;
        down.OnEnter();
        robot.q[0] = std::numeric_limits<float>::quiet_NaN();
        down.Run();
        Check(down.GetNextStateName() == kJointDamping, "stand2sit invalid feedback accepted");

        // RL -> Stand2Sit must blend all 21 joints, not only arms.
        robot.q = down_reference.InitialJointPosition();
        robot.dq.setZero();
        robot.rpy.setZero();
        robot.time += 10.0;
        const MatXf entry = MatXf::Constant(21, 5, .25f);
        robot.command = entry;
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        down.OnEnter();
        down.Run();
        Check(robot.command.isApprox(entry),
              "RL stand2sit entry did not start from the complete previous command");
        down.OnExit();
        CheckPreparedRise(data, up, reference);
        std::cout << "PASS: 29-to-21 squat preparation, measured readiness, bounded effort and full ONNX rise\n";
        std::cout << "PASS: state timing, pending transitions, live hold, full-body RL entry, reset, cancellation and faults\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
