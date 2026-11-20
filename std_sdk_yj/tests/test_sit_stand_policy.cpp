// Offline check against independently extracted ONNX constants.
#if defined(TEST_SIT2STAND_MIMIC)
#include "sit2stand_mimic_policy_runner.hpp"
using TestedMimicRunner = Sit2StandMimicPolicyRunner;
#else
#define TEST_STAND2SIT_MIMIC
#include "stand2sit_mimic_policy_runner.hpp"
using TestedMimicRunner = Stand2SitMimicPolicyRunner;
#endif
#include "json.hpp"
#include <fstream>
#include <limits>

#ifdef TEST_STAND2SIT_MIMIC
static constexpr int kObservationSize = 111;
static constexpr int kAngularVelocityOffset = 45;
#else
static constexpr int kObservationSize = 114;
static constexpr int kAngularVelocityOffset = 48;
#endif

static void Check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class F> static void Throws(F f, const char* message) {
    try { f(); } catch (const std::exception&) { return; }
    throw std::runtime_error(message);
}
static VecXf Vector(const nlohmann::json& j) {
    auto v = j.get<std::vector<float>>();
    return Eigen::Map<VecXf>(v.data(), v.size());
}

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "usage: test_sit_stand_policy POLICY_DIRECTORY");
        nlohmann::json motion;
        std::cin >> motion;
        TestedMimicRunner runner(RobotName::CR1STANDARD, argv[1]);
        Check(runner.decimation_ == 20, "expected 50 Hz on 1 kHz FSM");
        Check(runner.FrameCount() == motion.at("frames").get<int>(), "NPZ frame count differs");
        Check(motion.at("fps").get<int>() == 50, "NPZ fps differs");
        Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "mimic_test");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        std::ifstream manifest_file(std::filesystem::path(argv[1]) / "motion.json");
        nlohmann::json manifest;
        manifest_file >> manifest;
        const auto path = std::filesystem::path(argv[1]) / manifest.at("policy_file").get<std::string>();
        Ort::Session session(env, path.c_str(), options);
        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::AllocatorWithDefaultOptions allocator;
        const auto metadata = session.GetModelMetadata();
        auto names_text = metadata.LookupCustomMetadataMapAllocated("joint_names", allocator);
        std::stringstream stream(names_text.get());
        std::vector<std::string> names;
        std::string name;
        while (std::getline(stream, name, ',')) names.push_back(name);
        Check(names == motion.at("joint_names").get<std::vector<std::string>>(), "NPZ/ONNX joint order differs");
        const int order[] = {9, 15, 0, 10, 16, 1, 5, 11, 17, 2, 6, 12, 18, 3, 7, 13, 19, 4, 8, 14, 20};
        VecXf defaults(21), kp(21), kd(21), effort(21);
        defaults << 0, .18f, .1f, 0, 1, .18f, -.1f, 0, 1,
                    -.1f, 0, 0, .2f, -.1f, 0, -.1f, 0, 0, .2f, -.1f, 0;
#if defined(TEST_STAND2SIT_MIMIC) || defined(TEST_SIT2STAND_MIMIC) || defined(TEST_CRAWL_MIMIC)
        // model_69000.onnx / env(11).yaml default_joint_pos, SDK order.
        defaults << 0, .18f, .1f, 0, .8f, .18f, -.1f, 0, .8f,
                    -.15f, 0, 0, .3f, -.15f, 0, -.15f, 0, 0, .3f, -.15f, 0;
#endif
        kp << 100, 50, 50, 50, 50, 50, 50, 50, 50,
              100, 100, 100, 100, 70, 30, 100, 100, 100, 100, 70, 30;
        kd << 3, 5, 5, 5, 5, 5, 5, 5, 5, 3, 3, 3, 3, 2, 1, 3, 3, 3, 3, 2, 1;
        effort << 137, 137, 137, 137, 137, 137, 137, 137, 137,
                  363, 363, 137, 363, 137, 137, 363, 363, 137, 363, 137, 137;
#if defined(TEST_STAND2SIT_MIMIC) || defined(TEST_SIT2STAND_MIMIC) || defined(TEST_CRAWL_MIMIC)
        Check(runner.EffortLimits().isApprox(effort), "Training effort limits differ");
#endif
        VecXf scale = .25f * effort.cwiseQuotient(kp);
        Check(runner.JointKp().isApprox(kp) && runner.JointKd().isApprox(kd), "B3 PD gains differ");
        Check(runner.ActionScale().isApprox(scale), "B3 action scaling differs");
        Check(runner.DefaultJointPosition().isApprox(defaults), "Training default offsets differ");
        Check(!runner.InitialJointPosition().isApprox(defaults, 1e-3f),
              "Reference reset pose must not be replaced with training defaults");
        // Match the uploaded sim2sim validate_reference: check all body channels,
        // not just base rotation, in ONNX body-name order for every frame.
        auto body_text = metadata.LookupCustomMetadataMapAllocated("body_names", allocator);
        std::stringstream body_stream(body_text.get());
        const auto npz_bodies = motion.at("body_names").get<std::vector<std::string>>();
        std::vector<int> body_indices;
        while (std::getline(body_stream, name, ',')) {
            const auto it = std::find(npz_bodies.begin(), npz_bodies.end(), name);
            Check(it != npz_bodies.end(), "ONNX body missing from NPZ");
            body_indices.push_back(static_cast<int>(it - npz_bodies.begin()));
        }
        for (int frame = 0; frame < runner.FrameCount(); ++frame) {
            VecXf zero = VecXf::Zero(kObservationSize);
            float time_step = frame;
            const int64_t obs_shape[] = {1,kObservationSize}, step_shape[] = {1,1};
            std::array<Ort::Value,2> inputs{
                Ort::Value::CreateTensor<float>(memory, zero.data(), kObservationSize, obs_shape, 2),
                Ort::Value::CreateTensor<float>(memory, &time_step, 1, step_shape, 2)};
            const char* inputs_names[] = {"obs", "time_step"};
            const char* outputs_names[] = {"body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w", "joint_pos", "joint_vel"};
            auto outputs = session.Run(Ort::RunOptions{nullptr}, inputs_names, inputs.data(), 2, outputs_names, 6);
            for (int k=0; k<4; ++k) {
                const int width = k == 1 ? 4 : 3;
                Check(outputs[k].GetTensorTypeAndShapeInfo().GetShape() ==
                      std::vector<int64_t>({1, static_cast<int64_t>(body_indices.size()), width}), "Reference body shape differs");
                const auto expected = Vector(motion[outputs_names[k]][frame]);
                const float* actual = outputs[k].GetTensorData<float>();
                for (size_t b=0; b<body_indices.size(); ++b)
                    for (int c=0; c<width; ++c)
                        Check(std::abs(actual[b*width+c] - expected[body_indices[b]*width+c]) < 1e-5f,
                              "Embedded body reference differs from NPZ");
            }
        }
        RobotBasicState robot{};
        robot.base_rot_mat = (Eigen::AngleAxisf(.2f, Vec3f::UnitZ()) *
                              Eigen::AngleAxisf(-.03f, Vec3f::UnitY()) *
                              Eigen::AngleAxisf(.02f, Vec3f::UnitX())).toRotationMatrix();
        robot.base_omega = Vec3f(.03f, -.02f, .01f);
        robot.joint_pos = defaults;
        robot.joint_vel = VecXf::Zero(21);
        UserCommand user{};
        VecXf previous = VecXf::Zero(21), first_target;
        float max_observation_error = 0, max_action_error = 0;
        for (int step = 0; step < runner.FrameCount() + 100; ++step) {
            const int frame = std::min(step, runner.FrameCount() - 1);
            Check(runner.Frame() == frame, "reference frame did not clamp at clip end");
            const auto q = Vector(motion["joint_pos"][frame]);
            const auto dq = Vector(motion["joint_vel"][frame]);
            const auto quat = Vector(motion["anchor_quat"][frame]);
            const auto rotation = Eigen::Quaternionf(quat[0], quat[1], quat[2], quat[3]).normalized().toRotationMatrix();
            Check(runner.ReferenceAt(frame).position.isApprox(q, 1e-6f), "embedded reference positions differ from NPZ");
            Check(runner.ReferenceAt(frame).velocity.isApprox(dq, 1e-6f), "embedded reference velocities differ from NPZ");
            Check(runner.ReferenceAt(frame).anchor_rotation.isApprox(rotation, 1e-6f), "embedded reference anchor differs from NPZ");
            VecXf mapped_reference(21);
            for (int j = 0; j < 21; ++j) mapped_reference[order[j]] = q[j];
            Check(runner.ReferenceJointPosition(frame).isApprox(mapped_reference, 1e-6f),
                  "reference robot mapping differs from NPZ");
            if (frame == 0) {
                VecXf initial(21);
                for (int j = 0; j < 21; ++j) initial[order[j]] = q[j];
                Check(runner.InitialJointPosition().isApprox(initial), "frame-zero robot mapping differs");
            }
            for (int j = 0; j < 21; ++j) {
                robot.joint_pos[j] = defaults[j] + .01f * std::sin(.03f * step + j);
                robot.joint_vel[j] = .02f * std::cos(.03f * step + j);
            }
            VecXf obs(kObservationSize);
            obs.head(21) = q;
            obs.segment(21, 21) = dq;
#ifdef TEST_STAND2SIT_MIMIC
            // Independent analytic inverse rotation of unit gravity for the RPY above.
            obs.segment<3>(42) << std::sin(-.03f), -std::sin(.02f) * std::cos(-.03f),
                                 -std::cos(.02f) * std::cos(-.03f);
#else
            Mat3f relative = robot.base_rot_mat.transpose() * rotation;
            obs.segment<6>(42) << relative(0,0), relative(0,1), relative(1,0), relative(1,1), relative(2,0), relative(2,1);
#endif
            obs.segment<3>(kAngularVelocityOffset) = robot.base_omega;
            for (int j = 0; j < 21; ++j) {
                obs[kAngularVelocityOffset + 3 + j] = robot.joint_pos[order[j]] - defaults[order[j]];
                obs[kAngularVelocityOffset + 24 + j] = robot.joint_vel[order[j]];
            }
            obs.tail(21) = previous;
            float time_step = frame;
            const int64_t obs_shape[] = {1,kObservationSize}, step_shape[] = {1,1};
            std::array<Ort::Value,2> inputs{
                Ort::Value::CreateTensor<float>(memory, obs.data(), kObservationSize, obs_shape, 2),
                Ort::Value::CreateTensor<float>(memory, &time_step, 1, step_shape, 2)};
            const char* input_names[] = {"obs", "time_step"};
            const char* output_names[] = {"actions"};
            auto output = session.Run(Ort::RunOptions{nullptr}, input_names, inputs.data(), 2, output_names, 1);
            auto action = runner.GetRobotAction(robot, user);
            Check(runner.IsMotionFinished() == (step >= runner.FrameCount() - 1),
                  "completion must be signaled only after final-frame inference");
            VecXf expected = defaults;
            const float* raw = output[0].GetTensorData<float>();
            for (int j = 0; j < 21; ++j) expected[order[j]] += scale[order[j]] * raw[j];
            max_observation_error = std::max(max_observation_error, (runner.Observation()-obs).cwiseAbs().maxCoeff());
            max_action_error = std::max(max_action_error, (action.goal_joint_pos-expected).cwiseAbs().maxCoeff());
            Check(action.kp.isApprox(kp) && action.kd.isApprox(kd), "action gains differ");
            Check(action.goal_joint_vel.isZero() && action.tau_ff.isZero(), "unexpected feedforward");
            for (int j = 0; j < 21; ++j)
                previous[j] = (action.goal_joint_pos[order[j]]-defaults[order[j]]) / scale[order[j]];
            if (step == 0) first_target = action.goal_joint_pos;
        }
        Check(max_observation_error < 1e-5f, "observation assembly differs");
        Check(max_action_error < 1e-4f, "ONNX action postprocessing differs");
        Check(runner.HoldingLastFrame() && runner.IsMotionFinished(), "clip completion was not latched");
        runner.OnEnter();
        Check(!runner.HoldingLastFrame() && !runner.IsMotionFinished() && runner.Frame() == 0,
              "re-entry did not reset frame/completion");
        for(int j=0;j<21;++j) {
            robot.joint_pos[j] = defaults[j] + .01f * std::sin(float(j));
            robot.joint_vel[j] = .02f * std::cos(float(j));
        }
        Check(runner.GetRobotAction(robot,user).goal_joint_pos.isApprox(first_target,1e-6f), "re-entry did not reset action history");
        runner.OnEnter();
        robot.base_rot_mat = Eigen::AngleAxisf(.7f,Vec3f::UnitZ()).toRotationMatrix() * runner.ReferenceAt(0).anchor_rotation;
        runner.AlignHeading(robot.base_rot_mat);
        VecXf preparation_previous = VecXf::Zero(21);
        for (int i=0;i<100;++i) {
            auto action = runner.GetPreparationAction(robot);
            Check(runner.Frame()==0 && !runner.IsMotionFinished(), "preparation advanced the reference clock");
            Check(runner.Observation().head(21).isApprox(runner.ReferenceAt(0).position), "preparation reference differs");
            Check(runner.Observation().tail(21).isApprox(preparation_previous,1e-5f), "preparation action history did not advance");
            for (int j=0;j<21;++j)
                preparation_previous[j]=(action.goal_joint_pos[order[j]]-defaults[order[j]])/scale[order[j]];
        }
        runner.GetRobotAction(robot,user);
        Check(runner.Frame()==1, "playback skipped frame zero after preparation");
        Throws([&] { runner.GetPreparationAction(robot); }, "preparation allowed after playback advanced");
#ifdef TEST_STAND2SIT_MIMIC
        const Vec3f gravity = -robot.base_rot_mat.row(2).transpose();
        Check(runner.Observation().segment<3>(42).isApprox(gravity,1e-5f), "heading alignment changed gravity");
        runner.OnEnter();
        const auto yaw_target = runner.GetRobotAction(robot,user).goal_joint_pos;
        const VecXf yaw_observation = runner.Observation();
        runner.OnEnter();
        robot.base_rot_mat = Eigen::AngleAxisf(1.2f,Vec3f::UnitZ()).toRotationMatrix() * robot.base_rot_mat;
        runner.AlignHeading(robot.base_rot_mat);
        Check(runner.GetRobotAction(robot,user).goal_joint_pos.isApprox(yaw_target,1e-5f), "pure yaw changed gravity policy action");
        Check(runner.Observation().isApprox(yaw_observation,1e-5f), "pure yaw changed observation");
        runner.OnEnter();
        robot.base_rot_mat.setIdentity();
        runner.GetRobotAction(robot,user);
        Check(runner.Observation().segment<3>(42).isApprox(Vec3f(0,0,-1),1e-6f), "upright gravity is not unit negative Z");
#else
        Vec6f identity; identity << 1,0,0,1,0,0;
        Check(runner.Observation().segment<6>(42).isApprox(identity,1e-5f), "heading alignment differs");
#endif
        robot.joint_pos[0] = std::numeric_limits<float>::quiet_NaN();
        Throws([&] { runner.GetRobotAction(robot,user); }, "invalid feedback accepted");
        robot.joint_pos.setZero(29);
        Throws([&] { runner.GetRobotAction(robot,user); }, "29 DOF feedback accepted");
        Throws([&] { TestedMimicRunner bad(RobotName::CR1PRO,argv[1]); }, "unsupported robot accepted");
        std::cout << "PASS: " << runner.FrameCount() << " reference frames; max obs error="
                  << max_observation_error << ", target error=" << max_action_error
                  << "; 100 extra last-frame inferences, B3 gains/scales, reset/heading/error guards\n";
    } catch(const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
