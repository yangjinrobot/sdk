#ifndef CR1_SHOOT_REFERENCE_LOADER_HPP_
#define CR1_SHOOT_REFERENCE_LOADER_HPP_

#include "policy_runner_base.hpp"
#include <filesystem>

struct ShootReferenceData {
    VecXf joint_position;
    VecXf joint_velocity;
    Mat3f anchor_rotation;
};

// Reads the stored-NPZ reference export used by shoot_down/up.  It is a data
// loader only; each policy runner owns its ONNX session and control logic.
std::vector<ShootReferenceData> LoadShootReferenceNpz(const std::filesystem::path& path);

#endif
