#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dem::dem_client" for configuration "Release"
set_property(TARGET dem::dem_client APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dem::dem_client PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdem_client.so.1.1.0"
  IMPORTED_SONAME_RELEASE "libdem_client.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS dem::dem_client )
list(APPEND _IMPORT_CHECK_FILES_FOR_dem::dem_client "${_IMPORT_PREFIX}/lib/libdem_client.so.1.1.0" )

# Import target "dem::dem" for configuration "Release"
set_property(TARGET dem::dem APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(dem::dem PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdem.so.1.1.0"
  IMPORTED_SONAME_RELEASE "libdem.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS dem::dem )
list(APPEND _IMPORT_CHECK_FILES_FOR_dem::dem "${_IMPORT_PREFIX}/lib/libdem.so.1.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
