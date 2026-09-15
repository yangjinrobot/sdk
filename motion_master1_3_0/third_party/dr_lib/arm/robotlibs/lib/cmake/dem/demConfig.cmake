# dem CMake Config File

# 查找传递依赖 drdds
find_package(drdds QUIET)

include("${CMAKE_CURRENT_LIST_DIR}/demTargets.cmake")

# 设置传递依赖的 include 路径到 IMPORTED 目标上
# 确保 target_link_libraries 能自动传播头文件路径
set(_dm_inc_dirs
  ${CMAKE_CURRENT_LIST_DIR}/../../../include
  ${CMAKE_CURRENT_LIST_DIR}/../../../include/dem
  ${drdds_INCLUDE_DIRS}
)

if(TARGET dem::dem_client)
  set_target_properties(dem::dem_client PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_dm_inc_dirs}"
  )
endif()
if(TARGET dem::dem)
  set_target_properties(dem::dem PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_dm_inc_dirs}"
  )
endif()

unset(_dm_inc_dirs)

# 定义 dem_INCLUDE_DIRS（向后兼容）
set(dem_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}/../../../include
  ${CMAKE_CURRENT_LIST_DIR}/../../../include/dem
  ${drdds_INCLUDE_DIRS}
)

set(dem_LIBRARY_DIR ${CMAKE_CURRENT_LIST_DIR}/../../)
set(dem_LIBRARIES "-L${dem_LIBRARY_DIR} -ldem_client -ldem ${drdds_LIBRARIES}")

mark_as_advanced(dem_INCLUDE_DIRS dem_LIBRARIES)
