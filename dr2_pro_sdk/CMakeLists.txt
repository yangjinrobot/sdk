cmake_minimum_required(VERSION 3.10)

project(dr2_pro_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-fPIC)
add_compile_options(-w)
add_compile_options(-Wno-psabi)

set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")

if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
endif()

option(BUILD_SIM OFF)
if (BUILD_SIM)
    add_definitions(-DUSE_SIMULATION)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(state_machine)
add_subdirectory(example)

install(PROGRAMS
  simulation/joystick_velocity_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
