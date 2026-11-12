#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "spacemit_vslam::spacemit_vslam" for configuration "Release"
set_property(TARGET spacemit_vslam::spacemit_vslam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spacemit_vslam::spacemit_vslam PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspacemit_vslam.so"
  IMPORTED_SONAME_RELEASE "libspacemit_vslam.so"
  )

list(APPEND _cmake_import_check_targets spacemit_vslam::spacemit_vslam )
list(APPEND _cmake_import_check_files_for_spacemit_vslam::spacemit_vslam "${_IMPORT_PREFIX}/lib/libspacemit_vslam.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
