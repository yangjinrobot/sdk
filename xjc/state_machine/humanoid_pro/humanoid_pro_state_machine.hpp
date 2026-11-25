#pragma once

#include "state_machine_base.hpp"
#include "humanoid_pro/pro_controllers.hpp"
#include <memory>

class UdpNode;

namespace humanoid {

class HumanoidStateMachine final : public StateMachineBase {
public:
    HumanoidStateMachine(RobotName robot_name, RemoteCommandType remote_cmd_type,
                         ControlMode control_mode = ControlMode::kWholeBody);
    ~HumanoidStateMachine() override;
    void Start() override;
    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state) override;
    void Stop() override;

    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;
    const ControlMode control_mode_;

private:
    static void SetSharedUdp(UdpNode* node);
    void ConfigureUdp();

    ProControllers controllers_;
    std::shared_ptr<MotionStateStore> motion_state_store_;
    std::unique_ptr<UdpNode> shared_udp_node_;
    bool started_ = false;
    bool controller_entered_ = false;
};

} // namespace humanoid
