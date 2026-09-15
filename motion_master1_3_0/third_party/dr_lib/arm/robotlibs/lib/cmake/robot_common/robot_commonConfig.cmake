# robot_common CMake Config File

include("${CMAKE_CURRENT_LIST_DIR}/robot_commonTargets.cmake")

# 向后兼容变量
set(robot_common_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}/../../../include
  ${CMAKE_CURRENT_LIST_DIR}/../../../include/robot_common
)

set(robot_common_LIBRARY_DIR ${CMAKE_CURRENT_LIST_DIR}/../../)
set(robot_common_LIBRARIES "-L${robot_common_LIBRARY_DIR} -lrobot_common")

mark_as_advanced(robot_common_INCLUDE_DIRS robot_common_LIBRARIES)
