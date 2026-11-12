include_guard(GLOBAL)

# Locate SDK build interfaces. Package dependencies are declared in package.xml.
get_filename_component(_spacemit_sdk_default "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
while(NOT EXISTS "${_spacemit_sdk_default}/build/envsetup.sh")
  get_filename_component(_spacemit_sdk_parent "${_spacemit_sdk_default}" DIRECTORY)
  if(_spacemit_sdk_parent STREQUAL _spacemit_sdk_default)
    break()
  endif()
  set(_spacemit_sdk_default "${_spacemit_sdk_parent}")
endwhile()
if(DEFINED ENV{SROBOTIS_ROOT} AND NOT "$ENV{SROBOTIS_ROOT}" STREQUAL "")
  set(_spacemit_sdk_default "$ENV{SROBOTIS_ROOT}")
endif()
set(SROBOTIS_ROOT "${_spacemit_sdk_default}" CACHE PATH "SpacemiT Robot SDK workspace")
if(NOT EXISTS "${SROBOTIS_ROOT}/build/envsetup.sh")
  message(FATAL_ERROR "SROBOTIS_ROOT must identify the SpacemiT Robot SDK workspace")
endif()

set(MPP_ROOT "${SROBOTIS_ROOT}/components/multimedia/mpp" CACHE PATH "SDK MPP source")
get_filename_component(SPACEMIT_VSLAM_PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(SPACEMIT_CAMERA_ROOT "${SPACEMIT_VSLAM_PROJECT_ROOT}/modules/camera" CACHE PATH "Native camera module")
set(CAMERA_SDK_ROOT "${SPACEMIT_CAMERA_ROOT}" CACHE PATH "Native camera source")
set(IMU_DDS_ROOT "${SROBOTIS_ROOT}/middleware/ros2/peripherals/imu" CACHE PATH "SDK shared IMU DDS")
set(LAS2_ROOT "${SROBOTIS_ROOT}/components/perception/stereo_matching" CACHE PATH "SDK LAS2 component")
set(LAS2_BUILD_DIR "${LAS2_ROOT}/lib" CACHE PATH "SDK LAS2 external-input library directory")
set(SPACEMIT_VSLAM_ROOT "${SPACEMIT_VSLAM_PROJECT_ROOT}/modules/vslam" CACHE PATH "Native VSLAM module")
set(SPACEMIT_MAPPING_ROOT "${SPACEMIT_VSLAM_PROJECT_ROOT}/modules/mapping" CACHE PATH "Native mapping module")
set(SPACEMIT_SDK_PREFIX "${SROBOTIS_ROOT}/output/staging" CACHE PATH "Installed SDK dependency prefix")
list(PREPEND CMAKE_PREFIX_PATH "${SPACEMIT_SDK_PREFIX}" "${CMAKE_INSTALL_PREFIX}")

function(spacemit_find_mpp)
  find_path(MPP_INCLUDE_DIR NAMES sys/vb_api.h
    PATHS "${MPP_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
  find_library(MPP_LIBRARY NAMES mpp
    PATHS "${CMAKE_INSTALL_PREFIX}/lib" "${SPACEMIT_SDK_PREFIX}/lib"
      "${MPP_ROOT}/build-k3/lib" "${MPP_ROOT}/lib"
    NO_DEFAULT_PATH REQUIRED)
  set(MPP_INCLUDE_DIRS
    "${MPP_INCLUDE_DIR}" "${MPP_INCLUDE_DIR}/demux" "${MPP_INCLUDE_DIR}/sys"
    "${MPP_INCLUDE_DIR}/uvc" "${MPP_INCLUDE_DIR}/vdec" "${MPP_INCLUDE_DIR}/v2d"
    "${MPP_ROOT}/utils/include" "${MPP_ROOT}/al/include" PARENT_SCOPE)
  set(MPP_INCLUDE_DIR "${MPP_INCLUDE_DIR}" PARENT_SCOPE)
  set(MPP_LIBRARY "${MPP_LIBRARY}" PARENT_SCOPE)
endfunction()
