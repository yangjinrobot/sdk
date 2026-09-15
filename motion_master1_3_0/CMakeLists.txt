cmake_minimum_required(VERSION 3.10)

project(motion_master)
set(CMAKE_BUILD_TYPE Release)
# add_definitions(-g)
add_definitions(-w) # warning ignore
add_definitions(-Wno-psabi) # note ignore

add_compile_options(-fPIC)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -Og -fno-omit-frame-pointer")
#add_compile_options(-fsanitize=thread)
#add_compile_options(-fsanitize=address)

# ============  控制选项  ==============
set(BUILD_PLATFORM "" CACHE STRING "select build cpu type (x86 or arm).")
set(ROBOT "cr1" CACHE STRING "select robot series to link policy runner (only cr1).")
option(BUILD_SIM OFF)
option(SEND_REMOTE OFF)

# 机型选择校验：仅允许 cr1
string(TOLOWER "${ROBOT}" ROBOT)
if(NOT (ROBOT STREQUAL "cr1"))
    message(FATAL_ERROR "无效的 ROBOT=${ROBOT}，仅支持 cr1")
endif()
message(STATUS "RunPolicy 机型: ${ROBOT}")

# 检测当前系统架构
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH)
if(ARCH MATCHES "x86_64|amd64|i386|i686")
    set(CURRENT_PLATFORM "x86")
    message(STATUS "当前系统架构是 x86")
elseif(ARCH MATCHES "arm|aarch64")
    set(CURRENT_PLATFORM "arm")
    message(STATUS "当前系统架构是 ARM")
else()
    set(CURRENT_PLATFORM "unknown")
    message("未知系统架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 如果没有传入 BUILD_PLATFORM，则使用当前平台
if("${BUILD_PLATFORM}" STREQUAL "")
    set(BUILD_PLATFORM "${CURRENT_PLATFORM}")
    message(STATUS "未指定 BUILD_PLATFORM，使用当前平台: ${BUILD_PLATFORM}")
else()
    message(STATUS "指定 BUILD_PLATFORM: ${BUILD_PLATFORM}")
endif()

# 检查是否需要交叉编译
if(NOT "${BUILD_PLATFORM}" STREQUAL "${CURRENT_PLATFORM}")
    message(STATUS "交叉编译: 从 ${CURRENT_PLATFORM} 编译到 ${BUILD_PLATFORM}")
    if(BUILD_PLATFORM STREQUAL "arm")
        set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
        set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
        message(STATUS "设置 ARM 交叉编译器")
    endif()
else()
    message(STATUS "原生编译: ${BUILD_PLATFORM}")
endif()

# 平台特定设置
if (BUILD_PLATFORM STREQUAL arm)
    message("目标平台: ARM")
else ()
    message("目标平台: x86")
endif ()

set(CMAKE_CXX_STANDARD 17)

#include(config.cmake)
if (SEND_REMOTE)
    set(BUILD_SIM OFF)
endif ()

message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")
message("ROBOT:          ${ROBOT}")

get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)
set(PATH_LIB "../third_party")

set(DR_LIB_PATH ${THIRD_PARTY}/dr_lib/${BUILD_PLATFORM})
# message(${DR_LIB_PATH})
include_directories(
        ${DR_LIB_PATH}/drdds/include/dridl
        ${DR_LIB_PATH}/drdds/include
        ${DR_LIB_PATH}/drdds/
        ${DR_LIB_PATH}/ecanmaster/
        ${DR_LIB_PATH}/log4cplus/include/
        ${DR_LIB_PATH}/log4cplus/include/log4cplus_dr/
)
set(CMAKE_PREFIX_PATH ${DR_LIB_PATH}/drdds/lib/)

# ## 2. set all include folders
link_directories(
        ${DR_LIB_PATH}/drdds/
        ${DR_LIB_PATH}/drdds/lib
        ${DR_LIB_PATH}/log4cplus/lib
)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libfastcdr.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libfastrtps.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libfoonathan_memory-0.7.3.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libssl.so)
list(APPEND dr_lib ${DR_LIB_PATH}/drdds/lib/libcrypto.so)

list(APPEND dr_lib ${DR_LIB_PATH}/drdds/libdrdds.so)
list(APPEND dr_lib ${DR_LIB_PATH}/log4cplus/lib/liblog4cplus.so)

message("${dr_lib}")

include_directories(
        ${THIRD_PARTY}/eigen/
        ${THIRD_PARTY}/gamepad/include
        types
        utils
        ${DR_LIB_PATH}/robotlibs/include
        interface/robot
        interface/user_command
        interface/data_upload
        state_machine
        run_policy
        run_policy/${ROBOT}/include
        robot_exe/
        robot_exe/recorder
        ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
        ${THIRD_PARTY}/toml/
        model_inference
        ${THIRD_PARTY}/yaml/include
)

link_directories(
        ${DR_LIB_PATH}/drdds/lib/
        ${DR_LIB_PATH}/ecanmaster/
        ${DR_LIB_PATH}/robotlibs/lib/
        ${THIRD_PARTY}/yaml
)


add_subdirectory(interface)

file(GLOB RUN_POLICY_SRC "run_policy/${ROBOT}/src/*.cpp")

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH
        ${PATH_LIB}/onnxruntime/arm/lib/
        ${PATH_LIB}/onnxruntime/
        # /home/user/motion_master/lib/
)

if (SEND_REMOTE)
    set(DR_PATH_LIB ${PATH_LIB}/dr_lib/${BUILD_PLATFORM})
endif()

if (BUILD_SIM)
    add_definitions(-DUSE_SIMULATION)
    message("build simulation")
    file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")
else()
    list(APPEND dr_lib ${DR_LIB_PATH}/ecanmaster/libecanmaster.so)
    list(APPEND dr_lib ${DR_LIB_PATH}/robotlibs/lib/libdem_client.so)
    list(APPEND dr_lib ${DR_LIB_PATH}/robotlibs/lib/libdem.so)
    list(APPEND dr_lib ${DR_LIB_PATH}/drdds/libdrdds.so)
    file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*" "state_machine/humanoid/*.c*" "interface/robot/hardware/*.c*" "utils/*.c*" "robot_exe/*.c*" "robot_exe/recorder/*.c*")
    list(FILTER STATE_MACHINE_SRC EXCLUDE REGEX ".*test_.*\\.c")
endif ()

add_executable(motion_master main.cpp ${STATE_MACHINE_SRC} ${RUN_POLICY_SRC} ${RECORDER_PATH})

if (BUILD_PLATFORM STREQUAL arm)
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")
else ()
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/libonnxruntime.so")
endif ()

message("${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/")
target_link_libraries(motion_master interface -lpthread -lm -lrt -ldl -lstdc++ ${dr_lib} onnxruntime yaml-cpp)

# --disable-new-dtags: 使用旧式 RPATH 而非 RUNPATH
# --no-keep-memory / --reduce-memory-overheads: 降低 GNU ld 链接时内存占用，避免虚拟机最后链接阶段 OOM 卡死
# --hash-style=gnu: 使用更紧凑的 hash 表，减少链接内存与产物体积
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags -Wl,--no-keep-memory -Wl,--reduce-memory-overheads -Wl,--hash-style=gnu")

if (SEND_REMOTE)
    add_custom_target(push_to_robot ALL DEPENDS motion_master)
    add_custom_command(
            TARGET push_to_robot POST_BUILD
            COMMAND ./scripts/sftp_to_remote.sh
            WORKING_DIRECTORY ..
            COMMENT "run sftp_to_remote.sh"
    )
endif ()

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION 2.3.2)
set(CPACK_PACKAGE_NAME cr1_deploy)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY " Project CR1")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}.v${CPACK_PACKAGE_VERSION}.${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_PACKAGE_CONTACT "xqp")
message(${CPACK_PACKAGE_FILE_NAME})
set(CPACK_SKIP_PREINSTALL_COMMAND ON) 

# 转移到config.cmake中进行安装配置
# install(FILES build/motion_master
#         DESTINATION /opt/robot/share/motion_master/bin/)

# install(FILES run_policy/lib/${BUILD_PLATFORM}/libRunPolicy.so
#         DESTINATION /opt/robot/lib/)

# install(FILES third_party/yaml/libyaml-cpp.so
#         DESTINATION /opt/robot/lib/)

# install(DIRECTORY data/
#         DESTINATION /opt/robot/share/motion_master/data/)   

# install(DIRECTORY third_party/onnxruntime/${BUILD_PLATFORM}/lib/
#         DESTINATION /opt/robot/lib/)
#include(CPack)
include(config.cmake)
