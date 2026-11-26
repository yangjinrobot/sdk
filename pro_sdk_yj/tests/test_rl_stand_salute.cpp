#include "rl_stand_salute_policy_runner.hpp"
#include "amp_walking_policy_runner.hpp"
#include <array>
#include <limits>

static void Require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
static RobotBasicState Feedback(int tick) {
    RobotBasicState state{};
    state.base_rot_mat = RpyToRm(Vec3f(.025f*std::sin(tick*.05f),-.03f,.1f));
    state.base_omega << .01f,-.02f,.03f;
    state.joint_pos = VecXf::LinSpaced(31,-.1f,.2f);
    state.joint_vel = VecXf::Constant(31,.01f);
    state.joint_tau = VecXf::Zero(31);
    return state;
}
static float Quintic(float time) {
    const float t = std::clamp(time,0.f,1.f);
    return t*t*t*(t*(t*6.f-15.f)+10.f);
}
static void CheckRaiseContinuity() {
    const Vec7f start=(Vec7f()<<0.f,-.22f,0.f,1.2f,0.f,0.f,0.f).finished();
    constexpr double h=.002;
    Require(RLStandSalutePolicyRunner::RightArmRaisePose(start,0.).isApprox(start,1e-7f),
            "Continuous salute changed its measured entry pose");
    Require(RLStandSalutePolicyRunner::RightArmRaisePose(start,3.).isApprox(
            RLStandSalutePolicyRunner::RightArmPose(),1e-7f),"Continuous salute changed its endpoint");
    constexpr float radians = 3.14159265358979323846f/180.f;
    const auto first_half=RLStandSalutePolicyRunner::RightArmRaisePose(start,.6);
    Require(start(1)-first_half(1)>6.f*radians,
            "Shoulder roll barely moves while the elbow folds during the first segment");
    Require(first_half(3)<start(3),"Elbow did not fold together with the shoulder");
    for (int tick=10;tick<=290;++tick) {
        const double time=tick*.01;
        const auto before=RLStandSalutePolicyRunner::RightArmRaisePose(start,time-.005);
        const auto after=RLStandSalutePolicyRunner::RightArmRaisePose(start,time+.005);
        Require((after(1)-before(1))/.01f<-.001f,
                "Shoulder roll stopped or reversed before reaching the salute endpoint");
    }
    for (double time : {1.2,2.2}) {
        const auto before2=RLStandSalutePolicyRunner::RightArmRaisePose(start,time-2*h);
        const auto before=RLStandSalutePolicyRunner::RightArmRaisePose(start,time-h);
        const auto at=RLStandSalutePolicyRunner::RightArmRaisePose(start,time);
        const auto after=RLStandSalutePolicyRunner::RightArmRaisePose(start,time+h);
        const auto after2=RLStandSalutePolicyRunner::RightArmRaisePose(start,time+2*h);
        const Vec7f incoming=(at-before)/float(h), outgoing=(after-at)/float(h);
        Require(((after-before)/float(2*h)).norm()>.3f,"Salute stopped at an intermediate waypoint");
        Require((incoming-outgoing).cwiseAbs().maxCoeff()<.005f,"Salute waypoint velocity is discontinuous");
        const Vec7f a_before=(at-2.f*before+before2)/float(h*h);
        const Vec7f a_after=(after2-2.f*after+at)/float(h*h);
        Require((a_before-a_after).cwiseAbs().maxCoeff()<1.f,"Salute waypoint acceleration is discontinuous");
    }
    std::cout<<"Continuous salute: early shoulder/elbow coordination, continuous shoulder abduction, C2 joins and fixed endpoints passed\n";
}
static void CheckSalute() {
    RLStandPolicyRunner reference("salute-reference",RobotName::CR1PRO,-0.23f,-0.10f);
    RLStandSalutePolicyRunner salute("salute-test",RobotName::CR1PRO);
    Require(reference.IsReady() && salute.IsReady(),"RL Stand/salute model did not load");
    Require(reference.decimation_ == salute.decimation_,"Salute changed the RL Stand inference period");
    Require(salute.GetDefaultJointPosition()(21) == -0.23f &&
            salute.GetDefaultJointPosition()(27) == -0.23f,"Salute ankle pitch defaults are incorrect");
    Require(salute.GetDefaultJointPosition()(17) == -0.10f &&
            salute.GetDefaultJointPosition()(23) == -0.10f,"Salute hip pitch defaults are incorrect");
    const Vec7f target = (Vec7f()<<-.383972435f,-1.745329252f,-1.099557429f,-.610865238f,
                                 -1.466076572f,.034906585f,.331612558f).finished();
    constexpr float radians = 3.14159265358979323846f/180.f;
    const Vec7f chest = (Vec7f()<<-10.f,-40.f,20.f,-20.f,-30.f,0.f,0.f).finished()*radians;
    const Vec7f lift = (Vec7f()<<-20.f,-75.f,-15.f,-30.f,-65.f,1.f,10.f).finished()*radians;
    for (int entry=0;entry<2;++entry) {
        salute.OnEnter(); reference.OnEnter();
        auto initial = Feedback(0);
        initial.joint_pos.segment(3,14).array() += entry*.15f;
        const Vec7f left = initial.joint_pos.segment<7>(3), right = initial.joint_pos.segment<7>(10);
        Vec7f previous = right;
        for (int tick=0;tick<700;++tick) {
            auto state = Feedback(tick);
            state.joint_pos.segment(3,14).array() += entry*.15f + .02f*std::sin(tick*.03f);
            UserCommand command{};
            command.forward_vel_scale=.8f; command.side_vel_scale=-.5f; command.turnning_vel_scale=.4f;
            command.arm_preset=1+tick/120; command.arm_preset_seq=1+tick/120;
            command.disable_arm_motion_planning=tick%2;
            UserCommand standing{}; standing.disable_arm_motion_planning=true;
            const auto expected=reference.GetRobotAction(state,standing);
            auto actual=salute.GetRobotAction(state,command);
            Require(salute.IsReady() && reference.IsReady(),"Salute inference failed");
            Require(actual.goal_joint_pos.head(3).isApprox(expected.goal_joint_pos.head(3),1e-6f) &&
                    actual.goal_joint_pos.segment(17,12).isApprox(expected.goal_joint_pos.segment(17,12),1e-6f),
                    "Salute changed RL Stand waist/leg inference");
            Require(actual.kp == expected.kp && actual.kd == expected.kd &&
                    actual.goal_joint_vel.isZero() && actual.tau_ff.isZero(),"Salute changed RL Stand PD fields");
            Require(actual.goal_joint_pos.segment<7>(3).isApprox(left,1e-6f),"Salute did not hold the starting left arm pose");
            const Vec7f current=actual.goal_joint_pos.segment<7>(10);
            Require((current-previous).cwiseAbs().maxCoeff() <= .04f,"Salute arm target jumped");
            if (tick==120) Require(current.isApprox(chest,2e-5f),"Salute did not fold the hand in front of the chest");
            if (tick==220) Require(current.isApprox(lift,2e-5f),"Salute did not pass the upward lift waypoint");
            const Vec7f from=tick<=120?right:tick<=220?chest:lift;
            const Vec7f to=tick<=120?chest:tick<=220?lift:target;
            const Vec7f low=from.cwiseMin(to), high=from.cwiseMax(to);
            Require((current.array()>=low.array()-2e-5f).all() &&
                    (current.array()<=high.array()+2e-5f).all(),"Salute overshot a stage's joint envelope");
            previous=current;
            if (!tick) Require(current.isApprox(right,1e-6f),"Salute entry did not start from actual arm angles");
            if (tick>=300) Require(current.isApprox(target,1e-6f),"Salute changed the prescribed final pose");
        }
        salute.OnExit(); reference.OnExit();
    }
    salute.OnEnter();
    auto invalid=Feedback(0); invalid.joint_pos(13)=std::numeric_limits<float>::quiet_NaN();
    const auto damping=salute.GetRobotAction(invalid,UserCommand{});
    Require(!salute.IsReady() && damping.kp.isZero(),"Salute invalid feedback did not latch damping");
    salute.OnEnter();
    salute.GetRobotAction(Feedback(0),UserCommand{});
    Require(salute.IsReady(),"Salute reentry did not reset transient inference failure");
    salute.OnExit();
    std::cout<<"Salute: 1400 waist/leg/PD comparisons, trajectory, hold, reentry and fault checks passed\n";
}
static void CheckLeftArmAndGestureHandoff() {
    RLStandSalutePolicyRunner continuous("salute-uninterrupted",RobotName::CR1PRO);
    RLStandSalutePolicyRunner shared("salute-shared",RobotName::CR1PRO);
    constexpr float radians = 3.14159265358979323846f/180.f;
    const Vec7f target=(Vec7f()<<-27.f,11.f,11.f,15.f,-83.f,-2.f,69.f).finished()*radians;
    Require(RLStandSalutePolicyRunner::LeftArmPose().isApprox(target,1e-7f),
            "Left arm target does not match the requested degrees/order");
    continuous.OnEnter();shared.OnEnter();
    Vec7f left_start=Vec7f::Zero(), previous=Vec7f::Zero(), left_hold=Vec7f::Zero();
    Vec7f right_hold=Vec7f::Zero(), right_restart=Vec7f::Zero();
    for (int tick=0;tick<500;++tick) {
        auto state=Feedback(tick);
        state.joint_pos.segment(3,14).array() += .03f*std::sin(tick*.02f);
        if (tick==100 || tick==450) {
            shared.PreservePolicyOnExit(true);
            shared.OnExit();
            shared.SetLeftArmMotionEnabled(tick==100);
            shared.OnEnter();
            if (tick==100) {
                left_start=state.joint_pos.segment<7>(3);
                right_hold=state.joint_pos.segment<7>(10);
            } else {
                left_hold=state.joint_pos.segment<7>(3);
                right_restart=state.joint_pos.segment<7>(10);
            }
        }
        UserCommand command{};
        command.target_mode=tick>=100 && tick<450?RLControl_StandSaluteLeftArm_Mode:RLControl_StandSalute_Mode;
        const auto expected=continuous.GetRobotAction(state,command);
        const auto actual=shared.GetRobotAction(state,command);
        const Vec7f right_target=tick<100?Vec7f(expected.goal_joint_pos.segment<7>(10)):
            tick<450?right_hold:RLStandSalutePolicyRunner::RightArmRaisePose(right_restart,(tick-450)*.01);
        Require(actual.goal_joint_pos.segment<7>(10).isApprox(right_target,1e-6f),
                "Selected arm mode did not hold the other arm or restart from measured feedback");
        Require(actual.goal_joint_pos.head(3).isApprox(expected.goal_joint_pos.head(3),1e-6f) &&
                actual.goal_joint_pos.segment(17,12).isApprox(expected.goal_joint_pos.segment(17,12),1e-6f),
                "Gesture handoff reset ONNX history or changed waist/leg output");
        Require(actual.kp==expected.kp && actual.kd==expected.kd && actual.goal_joint_vel.isZero(),
                "Left arm gesture changed the standing PD fields");
        const Vec7f current=actual.goal_joint_pos.segment<7>(3);
        if (tick==100)
            Require(current.isApprox(left_start,1e-6f),"Left gesture did not start at the current measured pose");
        if (tick>100 && tick<450) {
            Require((current-previous).cwiseAbs().maxCoeff()<.025f,"Left arm trajectory jumped");
            Require((current.array()>=left_start.cwiseMin(target).array()-1e-6f).all() &&
                    (current.array()<=left_start.cwiseMax(target).array()+1e-6f).all(),
                    "Left arm overshot its joint envelope");
        }
        if (tick>=400 && tick<450)
            Require(current.isApprox(target,1e-6f),"Left arm did not reach and hold its target in three seconds");
        if (tick>=450)
            Require(current.isApprox(left_hold,1e-6f),"Returning to salute did not hold the measured left arm");
        previous=current;
    }
    continuous.OnExit();shared.OnExit();
    shared.SetLeftArmMotionEnabled(true);
    shared.OnEnter();
    const auto direct=Feedback(0);
    for (int tick=0;tick<320;++tick) {
        const auto action=shared.GetRobotAction(direct,UserCommand{});
        Require(action.goal_joint_pos.segment<7>(10).isApprox(direct.joint_pos.segment<7>(10),1e-6f),
                "Entering left-arm mode directly also started a right-arm salute");
        if (tick>=300)
            Require(action.goal_joint_pos.segment<7>(3).isApprox(target,1e-6f),"Direct left-arm mode missed its target");
    }
    shared.OnExit();
    shared.SetLeftArmMotionEnabled(false);
    shared.OnEnter();
    const auto initial=Feedback(0);
    const auto reset=shared.GetRobotAction(initial,UserCommand{});
    Require(reset.goal_joint_pos.segment<7>(3).isApprox(initial.joint_pos.segment<7>(3),1e-6f) &&
            reset.goal_joint_pos.segment<7>(10).isApprox(initial.joint_pos.segment<7>(10),1e-6f),
            "A real exit/reentry retained a gesture transition");
    shared.OnExit();
    std::cout<<"Arm modes: direct left entry, measured starts, inactive-arm holds, independent plans and shared ONNX history passed\n";
}
static void CheckLowerBeforeWalking() {
    RLStandSalutePolicyRunner salute("salute-lowering",RobotName::CR1PRO);
    RLStandPolicyRunner reference("lowering-reference",RobotName::CR1PRO,-0.23f,-0.10f);
    auto state=Feedback(0);
    state.joint_pos.segment<7>(10)=RLStandSalutePolicyRunner::RightArmPose();
    const Vec7f raised=state.joint_pos.segment<7>(10);
    const Vec7f lowered=(Vec7f()<<0.f,-.22f,0.f,1.2f,0.f,0.f,0.f).finished();
    salute.OnEnter();reference.OnEnter();
    salute.GetRobotAction(state,UserCommand{});reference.GetRobotAction(state,UserCommand{});
    Vec7f previous=raised;
    for(int tick=0;tick<270;++tick) {
        state.joint_pos.segment<7>(10)=previous;
        salute.RequestArmLowering(); // Repeated 0x11 requests must not restart.
        UserCommand standing{};standing.disable_arm_motion_planning=true;
        const auto expected=reference.GetRobotAction(state,standing);
        const auto action=salute.GetRobotAction(state,standing);
        const Vec7f target=raised+Quintic(tick*.01f/2.4f)*(lowered-raised);
        Require(action.goal_joint_pos.segment<7>(10).isApprox(target,2e-5f),"Lowering did not follow the standing quintic trajectory");
        Require(action.goal_joint_pos.head(3).isApprox(expected.goal_joint_pos.head(3),1e-6f) &&
                action.goal_joint_pos.segment(17,12).isApprox(expected.goal_joint_pos.segment(17,12),1e-6f),
                "Lowering changed the RL Stand waist/leg policy");
        if(tick<240) Require(!salute.IsArmLoweringComplete(),"Lowering completed before 2.4 seconds");
        if(tick>241) Require(salute.IsArmLoweringComplete(),"Tracked lowering never completed");
        previous=action.goal_joint_pos.segment<7>(10);
    }
    salute.OnExit();reference.OnExit();
    salute.OnEnter();state.joint_pos.segment<7>(10)=raised;
    salute.RequestArmLowering();
    for(int tick=0;tick<320;++tick)salute.GetRobotAction(state,UserCommand{});
    Require(!salute.IsArmLoweringComplete(),"A timer alone completed lowering while the real arm stayed raised");
    state.joint_pos.segment<7>(10)=lowered;
    salute.GetRobotAction(state,UserCommand{});
    Require(salute.IsArmLoweringComplete(),"Actual lowered feedback did not complete lowering");
    salute.OnExit();salute.OnEnter();
    Require(!salute.IsArmLoweringComplete(),"Salute reentry retained completed lowering");
    std::cout<<"Standing lowering: repeated requests, 2.4s trajectory, actual pose gate and reset passed\n";
}
static void CheckWalkingRelease() {
    AmpWalkingPolicyRunner reference("walking-reference",RobotName::CR1PRO);
    AmpWalkingPolicyRunner walking("walking-release",RobotName::CR1PRO);
    const Vec7f left=(Vec7f()<<0,.2f,0,1.2f,0,0,0).finished();
    const Vec7f right=RLStandSalutePolicyRunner::RightArmPose();
    VecXf start(14);start<<left,right;
    walking.SetArmReleaseFrom(left,right);
    walking.OnEnter(); reference.OnEnter();
    for (int tick=0;tick<180;++tick) {
        auto state=Feedback(tick); state.joint_pos.segment(3,14)=start;
        auto expected=reference.GetRobotAction(state,UserCommand{});
        auto actual=walking.GetRobotAction(state,UserCommand{});
        const float blend=Quintic(tick*.02f/2.4f);
        const VecXf goal=start+blend*(expected.goal_joint_pos.segment(3,14)-start);
        Require(actual.goal_joint_pos.segment(3,14).isApprox(goal,2e-5f),"Walking arm release interpolation mismatch");
        Require(actual.goal_joint_pos.head(3).isApprox(expected.goal_joint_pos.head(3),1e-6f) &&
                actual.goal_joint_pos.segment(17,12).isApprox(expected.goal_joint_pos.segment(17,12),1e-6f),
                "Arm release changed the walking waist/leg actions");
        Require(actual.kp == expected.kp && actual.kd == expected.kd,"Arm release changed walking PD gains");
        if (tick>=121) Require(actual.ConvertToMat().isApprox(expected.ConvertToMat(),1e-6f),"Walking did not return to full policy control");
    }
    walking.OnExit(); reference.OnExit();
    walking.OnEnter(); reference.OnEnter();
    auto actual=walking.GetRobotAction(Feedback(0),UserCommand{});
    auto expected=reference.GetRobotAction(Feedback(0),UserCommand{});
    Require(actual.ConvertToMat().isApprox(expected.ConvertToMat(),1e-6f),"Walking reentry retained the arm release overlay");
    bool rejected=false;
    try {walking.SetArmReleaseFrom(left,right,0.);}catch(const std::invalid_argument&){rejected=true;}
    Require(rejected,"Invalid walking arm release duration was accepted");
    std::cout<<"Walking release: 180 action comparisons, smooth handoff and reset passed\n";
}
int main() {
    try {CheckRaiseContinuity();CheckSalute();CheckLeftArmAndGestureHandoff();CheckLowerBeforeWalking();CheckWalkingRelease();return 0;}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
