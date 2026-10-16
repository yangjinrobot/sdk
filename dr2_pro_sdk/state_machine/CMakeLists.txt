set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/interface
  ${CMAKE_CURRENT_SOURCE_DIR}/run_policy
  ${CMAKE_CURRENT_SOURCE_DIR}/state
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/types
  ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_INSTALL_RPATH
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
)

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")

add_executable(state_machine main.cpp)

ament_target_dependencies(state_machine
  rclcpp
  drdds
  std_msgs
  sensor_msgs
)

target_link_libraries(state_machine
  onnxruntime
  pthread
  m
  rt
  dl
  ${OpenCV_LIBS}
)

install(TARGETS state_machine
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
