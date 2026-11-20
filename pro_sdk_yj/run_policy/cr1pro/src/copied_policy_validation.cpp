#include "copied_policy_validation.hpp"

namespace pro_policy {
// The mimic exports include a reference trajectory as well as the actions.
void ValidateCopiedModel(const Ort::Session& session, int observation_dim, bool mimic) {
    if (!mimic) {
        ValidateModelSchema(session, observation_dim, 29, "obs", "actions");
        return;
    }
    if (session.GetInputCount() != 2 || session.GetOutputCount() != 7)
        throw std::runtime_error("Mimic model requires two inputs and seven outputs");
    const char* inputs[] = {"obs", "time_step"};
    const char* outputs[] = {"actions", "joint_pos", "joint_vel", "body_pos_w",
                             "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"};
    Ort::AllocatorWithDefaultOptions allocator;
    for (int i = 0; i < 9; ++i) {
        const bool input = i < 2;
        const int index = input ? i : i - 2;
        auto name = input ? session.GetInputNameAllocated(index, allocator)
                          : session.GetOutputNameAllocated(index, allocator);
        auto type = input ? session.GetInputTypeInfo(index) : session.GetOutputTypeInfo(index);
        const auto tensor = type.GetTensorTypeAndShapeInfo();
        const auto shape = tensor.GetShape();
        const int width = input ? (index == 0 ? observation_dim : 1)
                                : (index < 3 ? 29 : (index == 4 ? 4 : 3));
        const bool body = !input && index >= 3;
        if (std::string(name.get()) != (input ? inputs[index] : outputs[index]) ||
            tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            shape.size() != (body ? 3u : 2u) || (shape[0] != 1 && shape[0] != -1) ||
            shape.back() != width || (body && shape[1] != -1 && shape[1] < 1))
            throw std::runtime_error("Mimic model schema mismatch: " + std::string(name.get()));
    }
}

void ValidateMimicOutputs(const std::vector<Ort::Value>& outputs) {
    if (outputs.size() != 7) throw std::runtime_error("Mimic output count mismatch");
    int64_t body_count = 0;
    for (int i = 0; i < 7; ++i) {
        if (!outputs[i].IsTensor()) throw std::runtime_error("Mimic output is not a tensor");
        const auto info = outputs[i].GetTensorTypeAndShapeInfo();
        const auto shape = info.GetShape();
        const int width = i < 3 ? 29 : (i == 4 ? 4 : 3);
        const bool body = i >= 3;
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            shape.size() != (body ? 3u : 2u) || shape[0] != 1 || shape.back() != width ||
            (body && shape[1] < 1))
            throw std::runtime_error("Mimic output shape/type mismatch");
        if (body) {
            if (body_count && body_count != shape[1]) throw std::runtime_error("Mimic body count mismatch");
            body_count = shape[1];
        }
        const Eigen::Map<const types::VecXf> values(outputs[i].GetTensorData<float>(), info.GetElementCount());
        if (!values.allFinite()) throw std::runtime_error("Mimic output contains non-finite values");
    }
}
}  // namespace pro_policy
