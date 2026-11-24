find_package(Git)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short=6 HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_SHORT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE GIT_COMMIT_SHORT_RESULT
    )
    execute_process(
      COMMAND git log -1 --format=%ci
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE GIT_COMMIT_DATE
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE GIT_COMMIT_DATE_RESULT
    )
    execute_process(
      COMMAND git rev-parse --abbrev-ref HEAD
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE CURRENT_GIT_BRANCH
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE CURRENT_GIT_BRANCH_RESULT
    )
else()
    message(WARNING "Git not found, will use 'unknown' as package version info.")
endif()

# 非 git 仓库或 git 信息获取失败时使用兜底值，避免 string() 命令报错
if(NOT GIT_FOUND OR NOT GIT_COMMIT_SHORT_RESULT EQUAL 0)
    set(GIT_COMMIT_SHORT "unknown")
endif()
if(NOT GIT_FOUND OR NOT GIT_COMMIT_DATE_RESULT EQUAL 0)
    set(GIT_COMMIT_DATE "unknown")
endif()
if(NOT GIT_FOUND OR NOT CURRENT_GIT_BRANCH_RESULT EQUAL 0)
    set(CURRENT_GIT_BRANCH "unknown")
endif()

message(STATUS "Current Git branch: ${CURRENT_GIT_BRANCH}, git hash = ${GIT_COMMIT_SHORT}, commit time = ${GIT_COMMIT_DATE}")
string(TOLOWER ${CURRENT_GIT_BRANCH} LOWERCASE_GIT_BRANCH)

set(DEV_TYPE "dev")  
set(DEFAULT_TYPE "")
string(REGEX MATCH "(^|/)dev(/|$)" MATCHED_DEV "${LOWERCASE_GIT_BRANCH}")

if(MATCHED_DEV)
    set(BRANCH_TYPE .${DEV_TYPE})
else()
    set(BRANCH_TYPE ${DEFAULT_TYPE})
endif()


SET(DEB_NAME motion_master)
SET(CMAKE_INSTALL_PREFIX "/opt/robot/share/motion_master")

set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/change_log.md VERSION_FILE)

string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" DEEPRCS_VERSION_STRING "${VERSION_FILE}")

string(REGEX MATCHALL "[A-Za-z0-9_]+" VERSION_PARTS "${DEEPRCS_VERSION_STRING}")
list(GET VERSION_PARTS 0 VERSION_MAJOR)
list(GET VERSION_PARTS 1 VERSION_MINOR)
list(GET VERSION_PARTS 2 VERSION_PATCH)
message("${PROJECT_NAME} Version: ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

string(TIMESTAMP CURRENT_TIME "%y%m%d%H%M")
message(STATUS "Current Make Time: ${CURRENT_TIME}")
execute_process(
    COMMAND date +%s
    OUTPUT_VARIABLE CURRENT_TIME_UNIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Current Make Time: ${CURRENT_TIME_UNIX}")

set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_PACKAGE_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})
# 根据目标平台判断（兼容原生 ARM / 原生 x86 / x86 交叉编译 ARM）
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SYS_PROC)
string(TOLOWER "${CMAKE_C_COMPILER}" C_COMP)
string(TOLOWER "${CMAKE_CXX_COMPILER}" CXX_COMP)
if( SYS_PROC MATCHES "aarch64|arm64"                        # ARM 原生
    OR
    C_COMP MATCHES "aarch64|arm64|arm-linux"         # 交叉编译器路径
    OR
    CXX_COMP MATCHES "aarch64|arm64|arm-linux"
)
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
else()
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
endif()
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_DEBIAN_PACKAGE_NAME ${DEB_NAME})
# set(PACKAGE_ID "97")

add_compile_definitions(
   CURRENT_GIT_BRANCH="${CURRENT_GIT_BRANCH}" 
   GIT_COMMIT_SHORT="${GIT_COMMIT_SHORT}" 
   GIT_COMMIT_DATE="${GIT_COMMIT_DATE}" 
   CURRENT_TIME="${CURRENT_TIME}"   
   CURRENT_TIME_UNIX="${CURRENT_TIME_UNIX}"  
   CPACK_PACKAGE_VERSION="${CPACK_PACKAGE_VERSION}"  
   BRANCH_TYPE="${BRANCH_TYPE}"
   CPACK_PACKAGE_NAME="${CPACK_PACKAGE_NAME}"
  #  package_id int类型
  # #  PACKAGE_ID="${PACKAGE_ID}"
   )
SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/interface/deb_pack/postinst" "${CMAKE_CURRENT_SOURCE_DIR}/interface/deb_pack/preinst")
set(CPACK_DEBIAN_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}.v${CPACK_PACKAGE_VERSION}+${GIT_COMMIT_SHORT}${BRANCH_TYPE}.${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}.${CURRENT_TIME}.std_yj")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${CPACK_PACKAGE_NAME} Project built using CMake. Pushed v${CPACK_PACKAGE_VERSION} on ${GIT_COMMIT_DATE}((${CURRENT_TIME_UNIX})) on branch ${CURRENT_GIT_BRANCH}(${GIT_COMMIT_SHORT})")
set(CPACK_PACKAGE_CONTACT "deeprobotics.cn")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "deeprobotics.cn")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_GENERATOR "DEB")

#添加deb包名
set(GENERATED_DEB_FILE_PATH "${CPACK_PACKAGE_FILE_NAME}.deb")

INCLUDE(CPack)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/interface/deb_pack/lib ${CMAKE_CURRENT_SOURCE_DIR}/interface/deb_pack/scripts DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/conf/ DESTINATION /var/opt/robot/conf/motion_master)
#install(DIRECTORY ./interface/deb_pack/log ./interface/deb_pack/data DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/change_log.md DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/conf/log.properties DESTINATION /var/opt/robot/conf/motion_master)

install(TARGETS motion_master
        RUNTIME DESTINATION /opt/robot/share/motion_master/bin/)

# 静态库已编译进 motion_master，运行时无需单独部署
# install(FILES build/run_policy/lib/${BUILD_PLATFORM}/libRunPolicy.so
#         DESTINATION /opt/robot/lib/)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml/libyaml-cpp.so
        DESTINATION /opt/robot/lib/)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/
        DESTINATION /opt/robot/share/motion_master/data/)   

# 按源码中的 amp、rl、mimic 目录结构安装 CR1 策略。
install(DIRECTORY "${CR1_POLICY_DIR}/policy/"
        DESTINATION /opt/robot/share/motion_master/policy/)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/${BUILD_PLATFORM}/lib/
        DESTINATION /opt/robot/lib/)

# install(FILES third_party/onnxruntime/arm/lib/* DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/onnxruntime)
# ===== 打包前准备 =====
add_custom_target(pre_package
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/conf
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/conf ${CMAKE_BINARY_DIR}/conf

    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/scripts
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/scripts ${CMAKE_BINARY_DIR}/scripts

    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/data
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/data ${CMAKE_BINARY_DIR}/data

    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/run_policy
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CR1_POLICY_DIR}/policy" "${CMAKE_BINARY_DIR}/run_policy/${ROBOT}/policy"

    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/interface/deb_pack
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/interface/deb_pack ${CMAKE_BINARY_DIR}/interface/deb_pack

    COMMAND ${CMAKE_COMMAND} -E echo "Pre-package resources copied to build directory"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    VERBATIM
)

# ===== 打包入口 =====
add_custom_target(normal_package
    COMMAND ${CMAKE_COMMAND} -E echo "Starting normal packaging process..."
    COMMAND ${CMAKE_CPACK_COMMAND} -G DEB -C $<CONFIG>
    DEPENDS pre_package motion_master
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    VERBATIM
)
