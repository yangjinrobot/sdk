#pragma once

#include "state_base.hpp"
#include <map>
#include <memory>

namespace humanoid {

using ProControllers = std::map<StateName, std::shared_ptr<StateBase>>;

ProControllers MakeProControllers(RobotName robot_name, const std::shared_ptr<ControllerData>& data_ptr);

} // namespace humanoid
