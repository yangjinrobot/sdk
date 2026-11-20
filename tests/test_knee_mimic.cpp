#include "knee_mimic_policy_runner.h"
#include <limits>
#include <sstream>
#include <type_traits>
#include <arpa/inet.h>
#include <unistd.h>

static void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
static std::vector<std::string> Split(const std::string& text) {
    std::istringstream stream(text);
    std::vector<std::string> result;
    std::string word;
    while (std::getline(stream, word, ',')) result.push_back(word);
    return result;
}

template <typename Runner>
static void Compare(const std::string& path, int last_frame) {
    Runner runner("test_knee", RobotName::CR1PRO);
    Require(runner.IsReady(), "Knee model failed to load");
    Require(runner.decimation_ == 20, "Knee policy must run at 50 Hz");
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "independent_knee_reference");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Ort::Session session(env, path.c_str(), options);
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::AllocatorWithDefaultOptions allocator;
    auto metadata = session.GetModelMetadata();
    auto strings = [&](const char* key) {
        auto text = metadata.LookupCustomMetadataMapAllocated(key, allocator);
        Require(bool(text), "Missing reference metadata");
        return Split(text.get());
    };
    auto vector = [&](const char* key) {
        const auto values = strings(key);
        VecXf result(values.size());
        for (size_t i = 0; i < values.size(); ++i) result(i) = std::stof(values[i]);
        return result;
    };
    const auto names = strings("joint_names");
    const auto sdk = Split("waist_yaw_joint,waist_roll_joint,waist_pitch_joint,"
        "left_shoulder_pitch_joint,left_shoulder_roll_joint,left_shoulder_yaw_joint,left_elbow_pitch_joint,"
        "left_wrist_yaw_joint,left_wrist_pitch_joint,left_wrist_roll_joint,"
        "right_shoulder_pitch_joint,right_shoulder_roll_joint,right_shoulder_yaw_joint,right_elbow_pitch_joint,"
        "right_wrist_yaw_joint,right_wrist_pitch_joint,right_wrist_roll_joint,"
        "left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,left_knee_pitch_joint,left_ankle_pitch_joint,left_ankle_roll_joint,"
        "right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,right_knee_pitch_joint,right_ankle_pitch_joint,right_ankle_roll_joint");
    const VecXf defaults = vector("default_joint_pos"), scale = vector("action_scale"),
                kp = vector("joint_stiffness"), kd = vector("joint_damping");
    std::vector<int> order;
    for (const auto& name : names) {
        const auto found = std::find(sdk.begin(), sdk.end(), name);
        Require(found != sdk.end(), "Unknown reference joint");
        order.push_back(found - sdk.begin());
    }
    const char* input_names[]{"obs", "time_step"};
    const char* output_names[]{"actions", "joint_pos", "joint_vel"};
    const int64_t shape[]{1,151}, frame_shape[]{1,1};
    auto infer = [&](VecXf& obs, int frame) {
        float time = frame;
        std::vector<Ort::Value> input;
        input.push_back(Ort::Value::CreateTensor<float>(memory, obs.data(), 151, shape, 2));
        input.push_back(Ort::Value::CreateTensor<float>(memory, &time, 1, frame_shape, 2));
        return session.Run(Ort::RunOptions{nullptr}, input_names, input.data(), 2, output_names, 3);
    };
    RobotBasicState state{};
    state.joint_pos = VecXf::LinSpaced(31, -.2f, .3f);
    state.joint_vel = VecXf::LinSpaced(31, -.3f, .1f);
    state.joint_tau = VecXf::Zero(31);
    state.base_rot_mat = RpyToRm(Vec3f(.2f, -.3f, .7f));
    state.base_omega << .1f,-.2f,.3f;
    UserCommand command{};
    VecXf previous = VecXf::Zero(29), observation = VecXf::Zero(151);
    const VecXf initial_q = state.joint_pos, initial_dq = state.joint_vel;
    const VecXf held_arms = state.joint_pos.segment(3,14);
    const std::array<float,7> arm_kp{300,100,100,100,40,40,40}, arm_kd{15,3,3,3,2,2,2};
    MatXf first;
    runner.OnEnter();
    for (int tick = 0; tick <= last_frame + 100; ++tick) {
        const bool terminal_arms = std::is_same_v<Runner, Stand2KneePolicyRunner> && tick >= last_frame;
        if (terminal_arms && tick > last_frame) {
            // Physical arms can move while model arm observations stay fixed.
            state.joint_pos.segment(3,14).array() += .003f;
            state.joint_vel.segment(3,14).setConstant(.4f);
        }
        const std::vector<Ort::Value> reference = infer(observation, std::min(tick, last_frame));
        observation.head(29) = Eigen::Map<const VecXf>(reference[1].GetTensorData<float>(), 29);
        observation.segment(29,29) = Eigen::Map<const VecXf>(reference[2].GetTensorData<float>(), 29);
        observation.segment<3>(58) = state.base_rot_mat.transpose() * Vec3f(0,0,-1);
        observation.segment<3>(61) = state.base_omega;
        for (int i = 0; i < 29; ++i) {
            observation(64+i) = state.joint_pos(order[i]) - defaults(i);
            observation(93+i) = state.joint_vel(order[i]);
        }
        observation.tail(29) = previous;
        if (terminal_arms) {
            for (int i = 0; i < 29; ++i) {
                if (order[i] < 3 || order[i] >= 17) continue;
                observation(64+i) = observation(93+i) = observation(122+i) = 0.f;
            }
        }
        const std::vector<Ort::Value> expected = infer(observation, std::min(tick, last_frame));
        previous = Eigen::Map<const VecXf>(expected[0].GetTensorData<float>(),29);
        auto actual = runner.GetRobotAction(state, command);
        Require(runner.IsReady(), "Knee inference faulted");
        for (int i = 0; i < 29; ++i) {
            if (terminal_arms && order[i] >= 3 && order[i] < 17) {
                previous(i) = 0.f;
                Require(std::abs(actual.goal_joint_pos(order[i]) - held_arms(order[i]-3)) < 1e-6f,
                        "Terminal arm action was not replaced by independent hold");
                Require(actual.kp(order[i]) == arm_kp[(order[i]-3)%7] &&
                        actual.kd(order[i]) == arm_kd[(order[i]-3)%7], "Independent arm gains differ from RL Stand");
                continue;
            }
            Require(std::abs(actual.goal_joint_pos(order[i]) - (defaults(i)+scale(i)*previous(i))) < 2e-5f,
                    "Knee action differs from independent simtosim observation/reference inference");
            Require(actual.kp(order[i]) == kp(i) && actual.kd(order[i]) == kd(i), "Knee gain order mismatch");
        }
        Require(actual.goal_joint_vel.isZero() && actual.tau_ff.isZero(), "Unexpected velocity/effort action");
        Require(runner.run_cnt_ == std::min(tick+1, last_frame), "Knee reference time did not clamp");
        if (!tick) first = actual.ConvertToMat();
    }
    runner.OnExit();
    runner.OnEnter();
    state.joint_pos = initial_q;
    state.joint_vel = initial_dq;
    state.joint_pos.tail(2).setConstant(100.f);
    Require(runner.GetRobotAction(state, command).ConvertToMat().isApprox(first, 1e-6f),
            "Knee reentry did not reset history/reference or virtual neck affected policy");
    state.joint_pos(0) = std::numeric_limits<float>::quiet_NaN();
    auto damping = runner.GetRobotAction(state, command);
    Require(!runner.IsReady() && runner.HasInferenceFault() && damping.kp.isZero() && damping.kd.isOnes(),
            "Knee invalid observation did not latch damping");
    runner.OnEnter();
    Require(runner.IsReady(), "Knee entry did not clear transient fault");
    Runner missing("missing", RobotName::CR1PRO, "/tmp/nonexistent_motion_master_knee.onnx");
    Require(!missing.IsReady(), "Knee missing model was accepted");
    Runner wrong("wrong_schema", RobotName::CR1PRO,
                 pro_policy::ResolveModelPath("mimic_policy/sit2stand.onnx"));
    Require(!wrong.IsReady(), "Knee runner accepted the old 154-dimensional schema");
    std::cout << path << ": full trajectory + 100 hold frames match independent inference; reentry and faults passed\n";
}

static void CheckTerminalArmPresets() {
    Stand2KneePolicyRunner knee("knee-arms", RobotName::CR1PRO);
    Require(knee.IsReady(), "Arm comparison model did not load");
    RobotBasicState state{};
    state.base_rot_mat.setIdentity();
    state.base_omega.setZero();
    state.joint_pos = VecXf::Zero(31);
    state.joint_vel = state.joint_tau = VecXf::Zero(31);
    state.joint_pos.segment(3,14).setConstant(.12f);
    UserCommand command{};
    knee.OnEnter();
    for (int tick = 0; tick < 383; ++tick) knee.GetRobotAction(state, command);
    VecXf current = state.joint_pos.segment(3,14), start = current, goal = current;
    const Vec7f kp = (Vec7f()<<300.f,100.f,100.f,100.f,40.f,40.f,40.f).finished();
    const Vec7f kd = (Vec7f()<<15.f,3.f,3.f,3.f,2.f,2.f,2.f).finished();
    int start_tick = 0;
    bool interpolating = false;
    for (int tick = 0; tick < 1050; ++tick) {
        if (tick % 150 == 10 && tick < 900) {
            command.arm_preset = tick/150 + 1;
            ++command.arm_preset_seq;
            goal.setZero();
            if (command.arm_preset == 1) {
                goal(0) = goal(7) = -1.57079632679f;
                goal(3) = goal(10) = 1.57079632679f;
            } else if (command.arm_preset == 2) {
                goal(1) = 1.57079632679f; goal(8) = -1.57079632679f;
                goal(3) = goal(10) = 1.57079632679f;
            } else if (command.arm_preset == 3 || command.arm_preset == 6) {
                goal(1) = .15f; goal(8) = -.15f;
            } else if (command.arm_preset == 5) {
                goal(1) = .3f; goal(8) = -.3f;
            }
            if (command.arm_preset == 6) goal(3) = goal(10) = 1.4f;
            start = current; start_tick = tick; interpolating = true;
        }
        if (tick == 950) command.disable_arm_motion_planning = true;
        if (tick == 955) command.disable_arm_motion_planning = false;
        if (command.disable_arm_motion_planning) {
            current = state.joint_pos.segment(3,14);
            interpolating = false;
        } else if (interpolating) {
            const float t = std::clamp((tick-start_tick)*.02f/2.4f,0.f,1.f);
            const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
            current = start + blend*(goal-start);
        }
        const auto actual = knee.GetRobotAction(state, command);
        Require(knee.IsReady(), "Independent arm inference failed");
        // The planner accumulates a float clock; the independent tick clock
        // may differ by a few 1e-5 radians over this 21-second sequence.
        if ((actual.goal_joint_pos.segment(3,14)-current).cwiseAbs().maxCoeff() > 1e-4f)
            throw std::runtime_error("Terminal quintic mismatch at tick " + std::to_string(tick) +
                ", max error=" + std::to_string((actual.goal_joint_pos.segment(3,14)-current).cwiseAbs().maxCoeff()));
        Require(actual.kp.segment<7>(3) == kp && actual.kp.segment<7>(10) == kp &&
                actual.kd.segment<7>(3) == kd && actual.kd.segment<7>(10) == kd,
                "Independent arm gains mismatch");
    }
    knee.OnExit(); knee.OnEnter();
    auto action = knee.GetRobotAction(state, UserCommand{});
    Require(knee.run_cnt_ == 1 && action.kp(3) == 50.f, "Reentry retained terminal arm mode");
    std::cout << "Terminal arms: 1050 independent forward/spread/legacy preset, disable and reentry checks passed\n";
}

static void CheckTerminalUdp() {
    struct Socket {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in address{};
        Socket() {
            Require(fd >= 0, "Loopback socket creation failed");
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            Require(bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
                    "Loopback bind failed");
            socklen_t size = sizeof(address);
            Require(getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) == 0,
                    "Loopback address lookup failed");
        }
        ~Socket() { if (fd >= 0) close(fd); }
    } peer;
    sockaddr_in destination{};
    {
        Socket reservation;
        destination = reservation.address;
    }
    UdpNode node("127.0.0.1", ntohs(destination.sin_port), "127.0.0.1", ntohs(peer.address.sin_port));
    Require(node.start(), "Terminal arm loopback receiver failed to start");
    Stand2KneePolicyRunner knee("knee-udp", RobotName::CR1PRO, "", &node);
    Require(knee.IsReady(), "UDP test model did not load");
    std::array<float,14> targets;
    targets.fill(.2f);
    Require(sendto(peer.fd, targets.data(), sizeof(targets), 0,
                   reinterpret_cast<sockaddr*>(&destination), sizeof(destination)) == sizeof(targets),
            "Loopback target packet send failed");
    for (int tick = 0; tick < 500 && !node.hasNewTargetData(); ++tick)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    Require(node.hasNewTargetData(), "Loopback target packet was not received");
    RobotBasicState state{};
    state.base_rot_mat.setIdentity(); state.base_omega.setZero();
    state.joint_pos = VecXf::Constant(31, .1f);
    state.joint_vel = state.joint_tau = VecXf::Zero(31);
    knee.OnEnter();
    for (int tick = 0; tick < 383; ++tick) knee.GetRobotAction(state, UserCommand{});
    Require(node.hasNewTargetData(), "UDP arms took over before the terminal frame");
    VecXf previous = state.joint_pos.segment(3,14);
    for (int tick = 0; tick < 100; ++tick) {
        auto action = knee.GetRobotAction(state, UserCommand{});
        Require(knee.IsReady(), "UDP terminal inference faulted");
        const VecXf current = action.goal_joint_pos.segment(3,14);
        for (int joint = 0; joint < 14; ++joint)
            Require(std::abs(current(joint)-previous(joint)) <= (joint%7 < 3 ? .03001f : .04001f),
                    "UDP arm target exceeded the RL Stand planner velocity bound");
        previous = current;
    }
    Require(!node.hasNewTargetData(), "Terminal frame did not consume the UDP target");
    Require(previous.isApprox(VecXf::Constant(14,.2f),1e-6f), "UDP target was not applied to both arms");
    float actual = 0;
    Require(node.getCurrentPosition(0,actual) && actual == state.joint_pos(3),
            "UDP feedback was replaced by the frozen model observation");
    knee.OnExit(); node.stop();
    std::cout << "Terminal UDP: loopback target deferral, both arms, velocity limits and real feedback passed\n";
}
int main() {
    try {
        Compare<Stand2KneePolicyRunner>(pro_policy::ResolveModelPath("mimic_policy/stand2knee/model_stand2knee_1028.onnx"), 383);
        Compare<Knee2StandPolicyRunner>(pro_policy::ResolveModelPath("mimic_policy/knee2stand/model_knee2stand_1028.onnx"), 312);
        CheckTerminalArmPresets();
        CheckTerminalUdp();
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
