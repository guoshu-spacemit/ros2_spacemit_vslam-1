#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "spacemit_mapping::spacemit_mapping" for configuration "Release"
set_property(TARGET spacemit_mapping::spacemit_mapping APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_mapping::spacemit_mapping PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_mapping.so"
  IMPORTED_SONAME_RELEASE "libspacemit_mapping.so"
  )

list(APPEND _cmake_import_check_targets spacemit_mapping::spacemit_mapping )
list(APPEND _cmake_import_check_files_for_spacemit_mapping::spacemit_mapping "${_IMPORT_PREFIX}/lib/libspacemit_mapping.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
