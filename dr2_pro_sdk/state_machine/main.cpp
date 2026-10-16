#include <iostream>
#include <string>

#include "humanoid_state_machine.hpp"

using namespace types;

namespace {

constexpr const char* kAnsiBoldRed = "\033[1;31m";
constexpr const char* kAnsiReset = "\033[0m";

#ifndef USE_SIMULATION
void ConfirmRealRobotSdkMode(ControlMode mode) {
  const char* required_mode = mode == ControlMode::kUpperBody ? "Upper Body" : "Whole Body";

  std::cout << kAnsiBoldRed
            << "\n[提示] 当前程序配置要求设备进入 " << required_mode << " SDK 模式。\n"
            << kAnsiReset;
  std::cout << "请确认设备模式已匹配后按回车继续，或按 Ctrl+C 取消启动。" << std::endl;

  std::string ignored_line;
  std::getline(std::cin, ignored_line);
}
#endif

}  // namespace

int main() {
  std::cout << "State Machine Start Running" << std::endl;
  rclcpp::init(0, 0);
  const ControlMode mode = ControlMode::kWholeBody;

#ifndef USE_SIMULATION
  ConfirmRealRobotSdkMode(mode);
#endif

  std::shared_ptr<humanoid::HumanoidStateMachine> fsm =
      std::make_shared<humanoid::HumanoidStateMachine>(RobotName::DR2PRO, mode);
  fsm->Start();
  fsm->Run();
  fsm->Stop();

  rclcpp::shutdown();
  return 0;
}
