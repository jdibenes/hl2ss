#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hl2comm_idl_geometry" for configuration "Release"
set_property(TARGET hl2comm_idl_geometry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hl2comm_idl_geometry PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/hl2comm_idl_geometry.lib"
  )

list(APPEND _cmake_import_check_targets hl2comm_idl_geometry )
list(APPEND _cmake_import_check_files_for_hl2comm_idl_geometry "${_IMPORT_PREFIX}/lib/hl2comm_idl_geometry.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
