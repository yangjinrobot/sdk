#include "amp_moe_15dof_policy_runner.hpp"
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sys/socket.h>

static void Require(bool ok, const char* msg) { if (!ok) throw std::runtime_error(msg); }
static int BindLoopback(sockaddr_in& address) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    address = {}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    Require(fd >= 0 && bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "bind loopback");
    socklen_t len = sizeof(address); Require(getsockname(fd, reinterpret_cast<sockaddr*>(&address), &len) == 0, "getsockname");
    return fd;
}
int main() {
    try {
        sockaddr_in destination{}, sink{};
        int reservation = BindLoopback(destination); close(reservation);
        int test_socket = BindLoopback(sink);
        UdpNode node("127.0.0.1", ntohs(destination.sin_port), "127.0.0.1", ntohs(sink.sin_port));
        Require(node.start(), "start local UDP receiver");
        AmpMoe15DofPolicyRunner::SetUdpNode(&node);
        AmpMoe15DofPolicyRunner runner("udp-audit", types::RobotName::CR1PRO);
        Require(runner.IsReady(), "load MoE model");
        types::RobotBasicState state{};
        state.base_rot_mat.setIdentity(); state.base_omega.setZero();
        state.joint_pos = types::VecXf::Zero(31); state.joint_pos.head(29) = runner.GetDefaultJointPosition();
        state.joint_vel = state.joint_tau = types::VecXf::Zero(31);
        types::UserCommand command{};
        std::array<float,14> target{};
        for (int i=0;i<14;++i) target[i] = state.joint_pos(i+3);
        target[2] = 2.f; target[3] = 0.f; target[4] = .6f;
        auto send = [&] {
            const auto count = node.getTargetDataUpdateCount();
            Require(sendto(test_socket, target.data(), sizeof(target), 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination)) == sizeof(target), "send loopback target");
            const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(1);
            while (node.getTargetDataUpdateCount() == count && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            Require(node.getTargetDataUpdateCount() > count, "receive loopback target");
        };
        runner.OnEnter(); send();
        types::RobotAction action;
        auto started=std::chrono::steady_clock::now();
        for (int step=0;step<100;++step) {
            action=runner.GetRobotAction(state,command);
            Require(runner.IsReady(),"inference");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        const float yaw=action.goal_joint_pos(5), elbow=action.goal_joint_pos(6), wrist=action.goal_joint_pos(7);
        Require(std::abs(yaw-2.f)<1e-4f && std::abs(elbow)<1e-4f && std::abs(wrist-.6f)<1e-4f, "reproduce training-range deviations");
        float previous=action.goal_joint_pos(5);
        target[2]=-2.f; send();
        float previous_velocity=0;
        for (int k=0;k<3;++k) {
            action=runner.GetRobotAction(state,command);
            previous_velocity=(action.goal_joint_pos(5)-previous)/.02f;
            previous=action.goal_joint_pos(5);
        }
        target[2]=2.f; send(); action=runner.GetRobotAction(state,command);
        const float velocity=(action.goal_joint_pos(5)-previous)/.02f;
        const float acceleration=(velocity-previous_velocity)/.02f;
        std::cout << std::setprecision(6) << "AUDIT {\"one_packet_age_s\":"<<age
            <<",\"shoulder_yaw_goal_rad\":"<<yaw<<",\"elbow_goal_rad\":"<<elbow
            <<",\"wrist_yaw_goal_rad\":"<<wrist<<",\"velocity_before_reversal\":"<<previous_velocity
            <<",\"velocity_after_reversal\":"<<velocity<<",\"finite_difference_acceleration_rad_s2\":"<<acceleration<<"}\n";
        Require(acceleration>149.f, "reproduce UDP velocity discontinuity");
        runner.OnExit(); AmpMoe15DofPolicyRunner::SetUdpNode(nullptr); node.stop(); close(test_socket);
    } catch (const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
