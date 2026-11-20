#include "humanoid_pro/bounded_policy_command.hpp"
#include <random>

static void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
int main() {
    try {
        ControlParameters cp(types::CR1PRO);
        humanoid_pro::BoundedPolicyCommand adapter(cp);
        types::RobotBasicState state{};
        state.joint_pos=types::VecXf::Zero(31);state.joint_vel=types::VecXf::Zero(31);
        types::MatXf command=types::MatXf::Zero(31,5);
        command(26,0)=100; command(26,1)=-2.18f;command(26,2)=3;
        state.joint_pos(26)=.49f;
        auto bounded=adapter.Apply(command,state);
        Require(std::abs(bounded(26,1)-cp.leg_joint_lower_(3))<1e-6f,"Knee target not bounded");
        Require(std::abs(bounded(26,4)-100*(-2.18f-cp.leg_joint_lower_(3)))<1e-3f,"Virtual PD effort lost");
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> angle(-5,5),speed(-30,30),gain(0,350),damping(0,15),effort(-800,800);
        for(int sample=0;sample<2000;++sample) {
            for(int i=0;i<29;++i) {
                state.joint_pos(i)=angle(rng);state.joint_vel(i)=speed(rng);
                command.row(i)<<gain(rng),angle(rng),damping(rng),speed(rng),effort(rng);
            }
            bounded=adapter.Apply(command,state);
            Require(bounded.allFinite()&&bounded.bottomRows(2).isZero(),"Invalid output or neck command");
            for(int i=0;i<29;++i) {
                const double q=state.joint_pos(i),dq=state.joint_vel(i),limit=cp.torque_limit_(i);
                const double raw=command(i,0)*(command(i,1)-q)+command(i,2)*(command(i,3)-dq)+command(i,4);
                const double actual=bounded(i,0)*(bounded(i,1)-q)+bounded(i,2)*(bounded(i,3)-dq)+bounded(i,4);
                Require(std::abs(actual-std::clamp(raw,-limit,limit))<.002,"Bounded total effort mismatch");
                Require(std::abs(bounded(i,4))<=limit,"Feedforward limit exceeded");
                Require(bounded(i,0)>=0&&bounded(i,0)<=command(i,0)&&bounded(i,2)>=0&&bounded(i,2)<=command(i,2),"Gains increased");
                Require(std::abs(bounded(i,3))<=cp.joint_vel_limit_(i),"Velocity limit exceeded");
            }
            Require(bounded(4,1)>=cp.arm_joint_lower_(1)&&bounded(4,1)<=cp.arm_joint_upper_(1),"Left arm limit mismatch");
            Require(bounded(11,1)>=-cp.arm_joint_upper_(1)&&bounded(11,1)<=-cp.arm_joint_lower_(1),"Mirrored right arm limit mismatch");
        }
        command(0,1)=std::numeric_limits<float>::quiet_NaN();
        bool rejected=false;try{adapter.Apply(command,state);}catch(const std::exception&){rejected=true;}
        Require(rejected,"Nonfinite command accepted");
        std::cout<<"Bounded policy commands: knee compensation, 2000 randomized effort/bounds cases and invalid input passed\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
