
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was spacemit_vslamConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(spacemit_vslam_FOUND TRUE)
set(spacemit_vslam_functional_FOUND ON)
set(spacemit_vslam_TARGET_SYSTEM "Linux")
set(spacemit_vslam_TARGET_PROCESSOR "riscv64")
set(spacemit_vslam_WITH_LMDB OFF)
set(spacemit_vslam_WITH_RVV ON)
set(spacemit_vslam_cpu_FOUND TRUE)
set(spacemit_vslam_shared_FOUND TRUE)
set(spacemit_vslam_map_io_FOUND OFF)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _spacemit_vslam_consumer_processor)
string(TOLOWER "${spacemit_vslam_TARGET_PROCESSOR}" _spacemit_vslam_sdk_processor)
foreach(_processor_var _spacemit_vslam_consumer_processor _spacemit_vslam_sdk_processor)
  if("${${_processor_var}}" MATCHES "^(amd64|x86_64)$")
    set(${_processor_var} x86_64)
  endif()
endforeach()
if(NOT CMAKE_SYSTEM_NAME STREQUAL spacemit_vslam_TARGET_SYSTEM OR
   NOT _spacemit_vslam_consumer_processor STREQUAL _spacemit_vslam_sdk_processor)
  set(spacemit_vslam_FOUND FALSE)
  set(spacemit_vslam_NOT_FOUND_MESSAGE "SDK target ${spacemit_vslam_TARGET_SYSTEM}/${spacemit_vslam_TARGET_PROCESSOR} does not match the consumer toolchain")
  return()
endif()

foreach(_component IN LISTS spacemit_vslam_FIND_COMPONENTS)
  if(NOT spacemit_vslam_${_component}_FOUND)
    set(spacemit_vslam_NOT_FOUND_MESSAGE "Unsupported or disabled SDK component: ${_component}")
  endif()
endforeach()
check_required_components(spacemit_vslam)
if(NOT spacemit_vslam_FOUND)
  return()
endif()

set_and_check(spacemit_vslam_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set_and_check(spacemit_vslam_PUBLIC_HEADER
  "${spacemit_vslam_INCLUDE_DIR}/spacemit_vslam.h")

# Keep the binary's OpenCV dependencies private to the imported shared library.
include(CMakeFindDependencyMacro)
set(_spacemit_vslam_opencv_components
  core imgcodecs video dnn calib3d features2d imgproc flann)
find_dependency(OpenCV 4.10 CONFIG COMPONENTS ${_spacemit_vslam_opencv_components})
if(NOT OpenCV_VERSION VERSION_LESS "4.11")
  set(spacemit_vslam_FOUND FALSE)
  set(spacemit_vslam_NOT_FOUND_MESSAGE
    "SDK requires OpenCV 4.10 (SONAME .410), but found ${OpenCV_VERSION}")
  return()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/spacemit_vslamTargets.cmake")
list(TRANSFORM _spacemit_vslam_opencv_components PREPEND "opencv_")
set_property(TARGET spacemit_vslam::spacemit_vslam
  PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE
  "${_spacemit_vslam_opencv_components}")
unset(_spacemit_vslam_opencv_components)
