#include "joint_protection.hpp"
#include "parameters/control_parameters.h"
#include <limits>
#include <stdexcept>
#include <iostream>

static void Check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
int main(int argc, char** argv) {
    try {
        Check(argc == 2, "expected TOML config");
        auto config = toml::parse(argv[1]);
        auto& section = config["joint_protection"];
        section["enabled"] = true;
        section["enable_fault_report"] = false;
        section["enable_damping_mode"] = false;
        ControlParameters cp(RobotName::CR1STANDARD);
        JointProtectionChecker checker;
        checker.init(21, &cp, config, nullptr);
        Eigen::Matrix<float, Eigen::Dynamic, 5> nominal = Eigen::Matrix<float, Eigen::Dynamic, 5>::Zero(21, 5);
        nominal.col(0).setConstant(100); nominal.col(2).setConstant(3);
        auto command = nominal;
        command(8, 1) = 10; // finite L1: clamp to the unchanged elbow soft limit
        command(1, 0) = -5; // must also check subsequent columns of this joint
        command(1, 1) = 10;
        command(1, 2) = -3;
        command(1, 3) = 1000;
        command(1, 4) = 10000;
        Check(checker.check(command) == ProtectionResult::PASS, "suppressed finite L1 should clamp");
        Check(command(8, 1) == cp.arm_joint_upper_(3), "L1 elbow escaped clamp");
        Check(command(1, 1) == cp.arm_joint_upper_(0), "L1 skipped subsequent dimension");
        Check(command(1, 0) == 0 && command(1, 2) == 0, "negative gain escaped clamp");
        Check(command(1, 3) == cp.joint_vel_limit_(1) && command(1, 4) == cp.torque_limit_(1), "velocity/torque escaped clamp");
        for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity()}) {
            command = nominal; command(8, 1) = bad;
            Check(checker.check(command) == ProtectionResult::L1_FATAL, "nonfinite L1 must reject despite suppression");
        }
        section["enable_damping_mode"] = true;
        checker.init(21, &cp, config, nullptr);
        command = nominal; command(8, 1) = 10;
        Check(checker.check(command) == ProtectionResult::L1_FATAL, "enabled L1 rejection changed");
        std::cout << "PASS: finite L1 clamped under suppression; remaining dimensions checked; NaN/Inf and enabled L1 rejected\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
