#include "mimic_policy_runner.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace {

uint16_t ReadU16(const std::vector<uint8_t>& bytes, size_t offset) {
    if (offset + 2 > bytes.size()) throw std::runtime_error("Truncated NPZ file");
    return static_cast<uint16_t>(bytes[offset]) |
           (static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

uint32_t ReadU32(const std::vector<uint8_t>& bytes, size_t offset) {
    if (offset + 4 > bytes.size()) throw std::runtime_error("Truncated NPZ file");
    return static_cast<uint32_t>(bytes[offset]) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

std::vector<uint8_t> ReadFile(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("Cannot open file: " + path.string());
    stream.seekg(0, std::ios::end);
    const auto length = stream.tellg();
    if (length <= 0) throw std::runtime_error("Empty file: " + path.string());
    stream.seekg(0, std::ios::beg);
    std::vector<uint8_t> bytes(static_cast<size_t>(length));
    stream.read(reinterpret_cast<char*>(bytes.data()), length);
    if (!stream) throw std::runtime_error("Cannot read file: " + path.string());
    return bytes;
}

// NumPy produced this deployment NPZ with ZIP method 0 (stored). Reading the
// central directory keeps the runtime independent of Python, cnpy and zlib.
std::unordered_map<std::string, std::vector<uint8_t>> ReadStoredNpz(
    const std::filesystem::path& path) {
    const auto bytes = ReadFile(path);
    constexpr uint32_t kEndSignature = 0x06054b50;
    constexpr uint32_t kCentralSignature = 0x02014b50;
    constexpr uint32_t kLocalSignature = 0x04034b50;

    size_t end_offset = std::string::npos;
    const size_t search_begin = bytes.size() > 65557 ? bytes.size() - 65557 : 0;
    for (size_t offset = bytes.size() - 4;; --offset) {
        if (ReadU32(bytes, offset) == kEndSignature) {
            end_offset = offset;
            break;
        }
        if (offset == search_begin) break;
    }
    if (end_offset == std::string::npos) throw std::runtime_error("Invalid NPZ end record");

    const uint16_t entry_count = ReadU16(bytes, end_offset + 10);
    size_t central_offset = ReadU32(bytes, end_offset + 16);
    std::unordered_map<std::string, std::vector<uint8_t>> entries;
    for (uint16_t entry = 0; entry < entry_count; ++entry) {
        if (ReadU32(bytes, central_offset) != kCentralSignature)
            throw std::runtime_error("Invalid NPZ central directory");
        const uint16_t method = ReadU16(bytes, central_offset + 10);
        const uint32_t compressed_size = ReadU32(bytes, central_offset + 20);
        const uint32_t uncompressed_size = ReadU32(bytes, central_offset + 24);
        const uint16_t name_length = ReadU16(bytes, central_offset + 28);
        const uint16_t extra_length = ReadU16(bytes, central_offset + 30);
        const uint16_t comment_length = ReadU16(bytes, central_offset + 32);
        const uint32_t local_offset = ReadU32(bytes, central_offset + 42);
        if (central_offset + 46 + name_length + extra_length + comment_length > bytes.size())
            throw std::runtime_error("Truncated NPZ central entry");
        const std::string name(reinterpret_cast<const char*>(bytes.data() + central_offset + 46),
                               name_length);
        if (method != 0 || compressed_size != uncompressed_size)
            throw std::runtime_error("Mofang NPZ entries must use ZIP stored mode");
        if (ReadU32(bytes, local_offset) != kLocalSignature)
            throw std::runtime_error("Invalid NPZ local entry");
        const uint16_t local_name_length = ReadU16(bytes, local_offset + 26);
        const uint16_t local_extra_length = ReadU16(bytes, local_offset + 28);
        const size_t data_offset = local_offset + 30 + local_name_length + local_extra_length;
        if (data_offset + uncompressed_size > bytes.size())
            throw std::runtime_error("Truncated NPZ array: " + name);
        entries[name] = std::vector<uint8_t>(bytes.begin() + data_offset,
                                             bytes.begin() + data_offset + uncompressed_size);
        central_offset += 46 + name_length + extra_length + comment_length;
    }
    return entries;
}

struct NpyArray {
    std::string descriptor;
    std::vector<size_t> shape;
    const uint8_t* data{nullptr};
    size_t byte_count{0};
};

NpyArray ParseNpy(const std::vector<uint8_t>& bytes, const std::string& name) {
    const uint8_t magic[] = {0x93, 'N', 'U', 'M', 'P', 'Y'};
    if (bytes.size() < 10 || !std::equal(std::begin(magic), std::end(magic), bytes.begin()))
        throw std::runtime_error("Invalid NPY array: " + name);
    const uint8_t major = bytes[6];
    const size_t length_bytes = major == 1 ? 2 : 4;
    const size_t header_start = 8 + length_bytes;
    const size_t header_length = major == 1 ? ReadU16(bytes, 8) : ReadU32(bytes, 8);
    if (major < 1 || major > 3 || header_start + header_length > bytes.size())
        throw std::runtime_error("Unsupported NPY header: " + name);
    const std::string header(reinterpret_cast<const char*>(bytes.data() + header_start), header_length);
    std::smatch match;
    if (!std::regex_search(header, match, std::regex("'descr'\\s*:\\s*'([^']+)'")))
        throw std::runtime_error("NPY descriptor missing: " + name);
    NpyArray result;
    result.descriptor = match[1];
    if (header.find("'fortran_order': True") != std::string::npos)
        throw std::runtime_error("Fortran-order NPY is unsupported: " + name);
    if (!std::regex_search(header, match, std::regex("'shape'\\s*:\\s*\\(([^)]*)\\)")))
        throw std::runtime_error("NPY shape missing: " + name);
    const std::string shape_text = match[1];
    const std::regex dimension_pattern("[0-9]+");
    for (std::sregex_iterator it(shape_text.begin(), shape_text.end(), dimension_pattern), end;
         it != end; ++it) {
        result.shape.push_back(static_cast<size_t>(std::stoull(it->str())));
    }
    if (result.shape.empty()) throw std::runtime_error("Invalid NPY shape: " + name);
    result.data = bytes.data() + header_start + header_length;
    result.byte_count = bytes.size() - header_start - header_length;
    return result;
}

std::vector<float> ReadFloatArray(const NpyArray& array, const std::string& name) {
    if (array.descriptor != "<f4" && array.descriptor != "=f4")
        throw std::runtime_error("Expected float32 NPY array: " + name);
    size_t count = 1;
    for (const size_t dimension : array.shape) {
        if (dimension > std::numeric_limits<size_t>::max() / count)
            throw std::runtime_error("NPY array is too large: " + name);
        count *= dimension;
    }
    if (array.byte_count != count * sizeof(float))
        throw std::runtime_error("NPY data size mismatch: " + name);
    std::vector<float> values(count);
    std::memcpy(values.data(), array.data, array.byte_count);
    if (!std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); }))
        throw std::runtime_error("Non-finite NPY data: " + name);
    return values;
}

std::vector<std::string> Split(const std::string& text) {
    std::vector<std::string> result;
    std::stringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ',')) result.push_back(value);
    return result;
}

std::string ReplaceAll(std::string text, const std::string& from, const std::string& to) {
    size_t position = 0;
    while ((position = text.find(from, position)) != std::string::npos) {
        text.replace(position, from.size(), to);
        position += to.size();
    }
    return text;
}

std::string PolicyNameToSdkName(std::string name) {
    name = ReplaceAll(name, "knee_pitch", "knee");
    name = ReplaceAll(name, "elbow_pitch", "elbow");
    name = ReplaceAll(name, "_pitch_", "_y_");
    name = ReplaceAll(name, "_roll_", "_x_");
    name = ReplaceAll(name, "_yaw_", "_z_");
    return name;
}

}  // namespace

std::filesystem::path MimicPolicyRunner::DefaultPolicyDirectory() {
    if (const char* override_directory = std::getenv("MOFANG_POLICY_DIR"))
        return override_directory;
    const std::filesystem::path installed("/opt/robot/share/motion_master/policy/mimic");
    if (std::filesystem::exists(installed / "model_11000.onnx") &&
        std::filesystem::exists(installed / "shooting_2.npz")) return installed;
    return std::filesystem::path(__FILE__).parent_path() / "../policy/mimic";
}

MimicPolicyRunner::MimicPolicyRunner(RobotName robot_name,
                                     const std::filesystem::path& policy_directory)
    : PolicyRunnerBase("mofang", robot_name) {
    if (robot_name != RobotName::CR1STANDARD)
        throw std::runtime_error("Mofang policy requires the 21-DoF CR1STANDARD robot");
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    const auto model_path = std::filesystem::canonical(policy_directory / "model_11000.onnx");
    session_ = Ort::Session(env_, model_path.c_str(), session_options_);
    std::cout << "loaded " << policy_name_ << " ONNX model from: "
              << model_path << std::endl;
    ValidateTensor(true, "obs", {1, kObservationSize});
    ValidateTensor(true, "time_step", {1, 1});
    ValidateTensor(false, "actions", {1, kJointCount});
    if (Metadata("observation_names") !=
            "command,motion_anchor_ori_b,base_ang_vel,joint_pos,joint_vel,actions" ||
        Metadata("anchor_body_name") != "base_link" || Metadata("command_names") != "motion")
        throw std::runtime_error("Unsupported Mofang observation metadata");

    const std::vector<std::string> robot_order{
        "waist_z_joint",
        "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint", "left_elbow_joint",
        "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint", "right_elbow_joint",
        "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint", "left_knee_joint",
        "left_ankle_y_joint", "left_ankle_x_joint",
        "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint", "right_knee_joint",
        "right_ankle_y_joint", "right_ankle_x_joint"};
    const auto policy_names = Split(Metadata("joint_names"));
    if (policy_names.size() != kJointCount)
        throw std::runtime_error("Mofang policy must contain 21 joint names");
    std::vector<bool> seen(kJointCount, false);
    for (const auto& policy_name : policy_names) {
        const auto sdk_name = PolicyNameToSdkName(policy_name);
        const auto found = std::find(robot_order.begin(), robot_order.end(), sdk_name);
        if (found == robot_order.end()) throw std::runtime_error("Unknown Mofang joint: " + policy_name);
        const int index = static_cast<int>(found - robot_order.begin());
        if (seen[index]) throw std::runtime_error("Duplicate Mofang joint: " + policy_name);
        seen[index] = true;
        policy_to_robot_.push_back(index);
    }

    // Explicit Mofang policy parameters in ONNX policy order:
    // L/R hip pitch, waist yaw, L/R hip roll, L/R shoulder pitch,
    // L/R hip yaw, L/R shoulder roll, L/R knee pitch,
    // L/R shoulder yaw, L/R ankle pitch, L/R elbow pitch,
    // L/R ankle roll.
    default_joint_position_ = VecXf::Zero(kJointCount);
    default_joint_position_ <<
        -0.2f, -0.2f, 
         0.0f,
         0.0f, 0.0f,
         0.0f,  0.0f, 
         0.0f, 0.0f,
         0.15f, -0.15f,
         0.4f,  0.4f,
         0.0f, 0.0f, 
        -0.2f, -0.2f,
         1.2f,  1.2f, 
         0.0f, 0.0f;

    kp_ = VecXf::Zero(kJointCount);
    kp_ <<
        100.0f, 100.0f, 100.0f, 100.0f, 100.0f,
         50.0f,  50.0f,  80.0f,  80.0f,  50.0f,  50.0f,
        100.0f, 100.0f,  50.0f,  50.0f,  70.0f,  70.0f,
         50.0f,  50.0f,  30.0f,  30.0f;
    kp_ =kp_ * 0.8;
    kd_ = VecXf::Zero(kJointCount);
    kd_ <<
        3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
        5.0f, 5.0f, 3.0f, 3.0f, 5.0f, 5.0f,
        3.0f, 3.0f, 5.0f, 5.0f, 2.0f, 2.0f,
        5.0f, 5.0f, 1.0f, 1.0f;

    action_scale_ = MetadataVector("action_scale");
    if ((kp_.array() <= 0).any() || (kd_.array() < 0).any() ||
        (action_scale_.array() <= 0).any())
        throw std::runtime_error("Invalid Mofang gains or action scale");
    LoadReference(std::filesystem::canonical(policy_directory / "shooting_2.npz"));
    decimation_ = 20;  // 1 kHz state-machine loop, 50 Hz policy/reference.
    OnEnter();
}

std::string MimicPolicyRunner::Metadata(const char* key) const {
    Ort::AllocatorWithDefaultOptions allocator;
    auto value = session_.GetModelMetadata().LookupCustomMetadataMapAllocated(key, allocator);
    if (!value) throw std::runtime_error(std::string("Missing Mofang metadata: ") + key);
    return value.get();
}

VecXf MimicPolicyRunner::MetadataVector(const char* key) const {
    const auto values = Split(Metadata(key));
    if (values.size() != kJointCount)
        throw std::runtime_error(std::string("Invalid Mofang metadata: ") + key);
    VecXf result(kJointCount);
    for (int index = 0; index < kJointCount; ++index) {
        size_t used = 0;
        result(index) = std::stof(values[index], &used);
        if (used != values[index].size() || !std::isfinite(result(index)))
            throw std::runtime_error(std::string("Invalid Mofang numeric metadata: ") + key);
    }
    return result;
}

void MimicPolicyRunner::ValidateTensor(bool input, const char* name,
                                       const std::vector<int64_t>& shape) const {
    Ort::AllocatorWithDefaultOptions allocator;
    const size_t count = input ? session_.GetInputCount() : session_.GetOutputCount();
    for (size_t index = 0; index < count; ++index) {
        auto actual_name = input ? session_.GetInputNameAllocated(index, allocator)
                                 : session_.GetOutputNameAllocated(index, allocator);
        if (std::string(actual_name.get()) != name) continue;
        auto type_info = input ? session_.GetInputTypeInfo(index) : session_.GetOutputTypeInfo(index);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        if (tensor_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            tensor_info.GetShape() != shape)
            throw std::runtime_error(std::string("Invalid Mofang tensor: ") + name);
        return;
    }
    throw std::runtime_error(std::string("Missing Mofang tensor: ") + name);
}

void MimicPolicyRunner::LoadReference(const std::filesystem::path& npz_path) {
    const auto entries = ReadStoredNpz(npz_path);
    const auto require = [&entries](const char* name) -> const std::vector<uint8_t>& {
        const auto found = entries.find(name);
        if (found == entries.end()) throw std::runtime_error(std::string("Missing NPZ array: ") + name);
        return found->second;
    };
    const auto fps_array = ParseNpy(require("fps.npy"), "fps.npy");
    if (fps_array.descriptor != "<i8" || fps_array.shape != std::vector<size_t>{1} ||
        fps_array.byte_count != sizeof(int64_t))
        throw std::runtime_error("Mofang fps.npy must be int64[1]");
    int64_t fps = 0;
    std::memcpy(&fps, fps_array.data, sizeof(fps));
    if (fps != 50) throw std::runtime_error("Mofang reference must run at 50 Hz");

    const auto position_array = ParseNpy(require("joint_pos.npy"), "joint_pos.npy");
    const auto velocity_array = ParseNpy(require("joint_vel.npy"), "joint_vel.npy");
    const auto quaternion_array = ParseNpy(require("body_quat_w.npy"), "body_quat_w.npy");
    if (position_array.shape.size() != 2 || position_array.shape[1] != kJointCount ||
        velocity_array.shape != position_array.shape || quaternion_array.shape.size() != 3 ||
        quaternion_array.shape[0] != position_array.shape[0] || quaternion_array.shape[2] != 4 ||
        quaternion_array.shape[1] < 1)
        throw std::runtime_error("Unexpected Mofang NPZ array shapes");
    const auto position = ReadFloatArray(position_array, "joint_pos.npy");
    const auto velocity = ReadFloatArray(velocity_array, "joint_vel.npy");
    const auto quaternion = ReadFloatArray(quaternion_array, "body_quat_w.npy");
    const size_t frames = position_array.shape[0];
    const size_t body_count = quaternion_array.shape[1];
    reference_.reserve(frames);
    for (size_t frame = 0; frame < frames; ++frame) {
        ReferenceFrame reference_frame;
        reference_frame.joint_position = Eigen::Map<const VecXf>(
            position.data() + frame * kJointCount, kJointCount);
        reference_frame.joint_velocity = Eigen::Map<const VecXf>(
            velocity.data() + frame * kJointCount, kJointCount);
        // shooting_2.npz is a stripped export without body_names. Its exporter
        // contract places base_link first; only that anchor quaternion is used.
        const float* values = quaternion.data() + frame * body_count * 4;
        Eigen::Quaternionf rotation(values[0], values[1], values[2], values[3]);
        if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6f)
            throw std::runtime_error("Invalid base quaternion in Mofang NPZ");
        reference_frame.anchor_rotation = rotation.normalized().toRotationMatrix();
        reference_.push_back(std::move(reference_frame));
    }
    if (reference_.empty()) throw std::runtime_error("Mofang NPZ has no frames");
}

VecXf MimicPolicyRunner::ToRobotOrder(const VecXf& policy_order) const {
    VecXf robot_order(kJointCount);
    for (int index = 0; index < kJointCount; ++index)
        robot_order(policy_to_robot_[index]) = policy_order(index);
    return robot_order;
}

void MimicPolicyRunner::DisplayPolicyInfo() {
    std::cout << "Mofang: " << kObservationSize << " observations, " << kJointCount
              << " actions, " << reference_.size() << " NPZ frames at 50 Hz ("
              << reference_.size() / 50.0 << " s)" << std::endl;
}

void MimicPolicyRunner::OnEnter() {
    frame_ = 0;
    run_cnt_ = 0;
    previous_action_ = VecXf::Zero(kJointCount);
    observation_.setZero();
    heading_alignment_.setIdentity();
}

void MimicPolicyRunner::OnExit() {}

void MimicPolicyRunner::AlignHeading(const Mat3f& actual_rotation) {
    const Mat3f& reference_rotation = reference_.front().anchor_rotation;
    const float yaw = std::atan2(actual_rotation(1, 0), actual_rotation(0, 0)) -
                      std::atan2(reference_rotation(1, 0), reference_rotation(0, 0));
    heading_alignment_ = Eigen::AngleAxisf(yaw, Vec3f::UnitZ()).toRotationMatrix();
}

VecXf MimicPolicyRunner::InitialJointPosition() const {
    return ToRobotOrder(reference_.front().joint_position);
}

VecXf MimicPolicyRunner::JointKp() const { return ToRobotOrder(kp_); }
VecXf MimicPolicyRunner::JointKd() const { return ToRobotOrder(kd_); }
bool MimicPolicyRunner::Finished() const { return frame_ >= static_cast<int>(reference_.size()); }
int MimicPolicyRunner::Frame() const { return frame_; }
int MimicPolicyRunner::FrameCount() const { return static_cast<int>(reference_.size()); }

RobotAction MimicPolicyRunner::GetRobotAction(const RobotBasicState& robot, const UserCommand&) {
    if (Finished()) throw std::runtime_error("Mofang reference has finished");
    if (robot.joint_pos.size() != kJointCount || robot.joint_vel.size() != kJointCount ||
        !robot.base_rot_mat.allFinite() || !robot.base_omega.allFinite() ||
        !robot.joint_pos.allFinite() || !robot.joint_vel.allFinite())
        throw std::runtime_error("Invalid Mofang robot observation");
    const auto& reference = reference_[frame_];
    observation_.segment(0, kJointCount) = reference.joint_position;
    observation_.segment(kJointCount, kJointCount) = reference.joint_velocity;
    const Mat3f relative_rotation = robot.base_rot_mat.transpose() * heading_alignment_ *
                                    reference.anchor_rotation;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 2; ++column)
            observation_(42 + row * 2 + column) = relative_rotation(row, column);
    observation_.segment<3>(48) = robot.base_omega;
    for (int index = 0; index < kJointCount; ++index) {
        observation_(51 + index) = robot.joint_pos(policy_to_robot_[index]) -
                                   default_joint_position_(index);
        observation_(72 + index) = robot.joint_vel(policy_to_robot_[index]);
    }
    observation_.tail(kJointCount) = previous_action_;

    float time_step = static_cast<float>(frame_);
    const std::array<int64_t, 2> observation_shape{1, kObservationSize};
    const std::array<int64_t, 2> time_shape{1, 1};
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(memory_info_, observation_.data(), observation_.size(),
                                        observation_shape.data(), observation_shape.size()),
        Ort::Value::CreateTensor<float>(memory_info_, &time_step, 1,
                                        time_shape.data(), time_shape.size())};
    const char* output_names[]{"actions"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, inputs.data(), inputs.size(),
                                output_names, 1);
    VecXf action = Eigen::Map<const VecXf>(outputs[0].GetTensorData<float>(), kJointCount);
    if (!action.allFinite()) throw std::runtime_error("Non-finite Mofang action");

    RobotAction result;
    result.goal_joint_pos = ToRobotOrder(default_joint_position_ +
                                         action_scale_.cwiseProduct(action));
    result.goal_joint_vel = VecXf::Zero(kJointCount);
    result.tau_ff = VecXf::Zero(kJointCount);
    result.kp = JointKp();
    result.kd = JointKd();
    previous_action_ = action;
    ++frame_;
    ++run_cnt_;
    return result;
}
