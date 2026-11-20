#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robot_common::robot_common" for configuration "Release"
set_property(TARGET robot_common::robot_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robot_common::robot_common PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobot_common.so.1.1.0"
  IMPORTED_SONAME_RELEASE "librobot_common.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS robot_common::robot_common )
list(APPEND _IMPORT_CHECK_FILES_FOR_robot_common::robot_common "${_IMPORT_PREFIX}/lib/librobot_common.so.1.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
