// Exercise the production PRO offset implementation with an isolated ECAN base.
// No motor initialization, DDS participant or real hardware is created.
#include "basic_function.hpp"
#include <atomic>
#include <mutex>
#include <stdexcept>
#include <cstring>
#include <unistd.h>
using namespace types;

#define ECAN_HARDWARE_INTERFACE_HPP_
class EcanHardwareInterface {
protected:
    std::recursive_mutex command_mutex_, feedback_mutex_;
    int dof_num_ = 31;
    float pos_offset_[31]{}, joint_dir_[31]{};
    bool data_updated_[31]{};
    struct JointConfig { float dir = 0, offset = 0; } joint_config_[31];
    VecXf joint_pos_ = VecXf::Zero(31), joint_pos_raw_ = VecXf::Zero(31);
    double time_stamp_ = 0;
    std::atomic<bool> stop_recover_{false};
    void ResetJointError() {}
    void DoSetJointCommand(const MatXf&) {}
    bool IsDataUpdatedFinished() const { return true; }
    void RefreshRobotData() {}
public:
    EcanHardwareInterface(const std::string&, int, int, int, int, int) {}
    virtual void OnHardStopRecoveredAllJoints() {}
    virtual void WrapPositionOffset() {}
    VecXf GetJointPosition() {
        std::lock_guard<std::recursive_mutex> lock(feedback_mutex_);
        return joint_pos_;
    }
};
#include "hardware/cr1pro_hardware_interface.hpp"

class TestProHardware : public CR1ProHardwareInterface {
public:
    TestProHardware() : CR1ProHardwareInterface("offline-test") {}
    using CR1ProHardwareInterface::WrapPositionOffset;
    float Offset(int joint) const { return pos_offset_[joint]; }
    void SetOffset(int joint, float degrees) {
        pos_offset_[joint] = degrees;
        joint_config_[joint].offset = Deg2Rad(degrees);
    }
    void SetFeedback(int joint, float raw_degrees, float cached_degrees) {
        joint_pos_raw_(joint) = Deg2Rad(raw_degrees);
        joint_pos_(joint) = Deg2Rad(cached_degrees);
    }
};
static void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    try {
        TestProHardware robot;
        const VecXf defaults = robot.GetJointPosition();
        for (int joint = 0; joint < 31; ++joint) {
            robot.SetOffset(joint, robot.Offset(joint) + 10.f);
            robot.SetFeedback(joint, 0.f, 999.f);
        }
        // Reset each individual joint and preserve every unselected offset.
        for (int selected = 0; selected < 31; ++selected) {
            const VecXf before = robot.GetJointPosition();
            robot.InitPositionOffset(uint64_t{1} << selected);
            const VecXf after = robot.GetJointPosition();
            Require(std::abs(after(selected) - defaults(selected)) < 1e-5f,
                    "selected joint retained stale position after offset reset");
            for (int other = 0; other < 31; ++other) {
                if (other == selected) continue;
                Require(after(other) == before(other), "offset reset changed an unselected joint position");
                const float expected = Rad2Deg(defaults(other)) + (other > selected ? 10.f : 0.f);
                Require(std::abs(robot.Offset(other) - expected) < 1e-3f,
                        "offset reset changed an unselected joint offset");
            }
        }
        // New raw encoder data can arrive while the derived position is stale.
        robot.SetFeedback(3, 0.f, 460.f);
        robot.InitPositionOffset(uint64_t{1} << 3);
        robot.WrapPositionOffset();
        Require(std::abs(robot.Offset(3) - 90.f) < 1e-5f, "wrap used pre-calibration coordinates");

        robot.SetFeedback(3, 370.f, 460.f);
        robot.WrapPositionOffset();
        Require(std::abs(robot.Offset(3) + 270.f) < 1e-5f, "wrap missed a full encoder turn");
        Require(std::abs(robot.GetJointPosition()(3) - Deg2Rad(100.f)) < 1e-5f,
                "wrap did not refresh the cached joint position");
        robot.WrapPositionOffset();
        Require(std::abs(robot.Offset(3) + 270.f) < 1e-5f, "repeated wrap subtracted a second turn");
        std::cout << "31 single-joint offset masks and immediate position refresh passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
