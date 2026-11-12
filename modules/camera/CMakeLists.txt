cmake_minimum_required(VERSION 3.22)
project(spacemit_camera LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CAMERA_SDK_WITH_MPP "Build native SDK MPP capture" OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(_camera_v4l2_default ON)
else()
  set(_camera_v4l2_default OFF)
endif()
option(CAMERA_SDK_WITH_V4L2 "Build native Linux quad capture" ${_camera_v4l2_default})
get_filename_component(_camera_sdk_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../.." ABSOLUTE)
if(DEFINED ENV{SROBOTIS_ROOT} AND NOT "$ENV{SROBOTIS_ROOT}" STREQUAL "")
  set(_camera_sdk_root "$ENV{SROBOTIS_ROOT}")
endif()
set(MPP_ROOT "${_camera_sdk_root}/components/multimedia/mpp" CACHE PATH "SDK MPP source")
set(MPP_BUILD_DIR "${CMAKE_INSTALL_PREFIX}" CACHE PATH "Matching MPP prefix")
set(MPP_LIBRARY "" CACHE FILEPATH "Matching MPP library")

function(spacemit_camera_public_target target export_name output_name)
  set_target_properties(${target} PROPERTIES EXPORT_NAME ${export_name}
    OUTPUT_NAME ${output_name} POSITION_INDEPENDENT_CODE ON)
  add_library(spacemit_camera::${export_name} ALIAS ${target})
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
endfunction()

add_library(spacemit_camera_core SHARED src/camera.cpp src/embedded_metadata.cpp
  src/imu_boundary_deduplication.cpp src/seeker_protocol.cpp src/seeker_stream.cpp
  src/quad_camera.cpp)
spacemit_camera_public_target(spacemit_camera_core camera spacemit_camera)
target_link_libraries(spacemit_camera_core PRIVATE Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(src/seeker_stream.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
endif()
if(CAMERA_SDK_WITH_V4L2)
  target_sources(spacemit_camera_core PRIVATE src/quad_capture_v4l2.cpp)
  target_compile_definitions(spacemit_camera_core PRIVATE CAMERA_SDK_WITH_V4L2=1)
endif()
if(CAMERA_SDK_WITH_MPP)
  if(NOT MPP_LIBRARY)
    find_library(_camera_mpp_library NAMES mpp PATHS
      "${MPP_BUILD_DIR}/${CMAKE_INSTALL_LIBDIR}" "${MPP_BUILD_DIR}/lib"
      "${MPP_BUILD_DIR}/lib64" "${CMAKE_INSTALL_FULL_LIBDIR}"
      "${CMAKE_INSTALL_PREFIX}/lib" "${CMAKE_INSTALL_PREFIX}/lib64"
      "${_camera_sdk_root}/output/staging/lib" "${_camera_sdk_root}/output/staging/lib64"
      "${MPP_ROOT}/build-k3/lib" NO_DEFAULT_PATH REQUIRED)
    set(MPP_LIBRARY "${_camera_mpp_library}" CACHE FILEPATH "Matching MPP library" FORCE)
  endif()
  target_sources(spacemit_camera_core PRIVATE src/quad_capture_mpp.cpp)
  target_compile_definitions(spacemit_camera_core PRIVATE CAMERA_SDK_WITH_MPP=1)
  target_include_directories(spacemit_camera_core PRIVATE "${MPP_ROOT}/include"
    "${MPP_ROOT}/include/sys" "${MPP_ROOT}/include/uvc"
    "${MPP_ROOT}/utils/include" "${MPP_ROOT}/al/include")
  target_link_libraries(spacemit_camera_core PRIVATE "${MPP_LIBRARY}")
  include(CheckCXXSourceCompiles)
  include(CMakePushCheckState)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_INCLUDES "${MPP_ROOT}/include" "${MPP_ROOT}/include/sys")
  set(CMAKE_REQUIRED_LIBRARIES "${MPP_LIBRARY}" Threads::Threads)
  get_filename_component(_camera_mpp_library_dir "${MPP_LIBRARY}" DIRECTORY)
  set(CMAKE_REQUIRED_LINK_OPTIONS "-Wl,-rpath-link,${_camera_mpp_library_dir}")
  unset(CAMERA_SDK_MPP_HAS_INPUT_POOL CACHE)
  check_cxx_source_compiles("
    #include <sys/sys_api.h>
    #include <vdec/vdec_api.h>
    int main() {
      VdecChnAttr attr{};
      attr.bEnableInputDmaBuf = MPP_TRUE;
      MppNode source{}, sink{};
      return SYS_ConfigStreamDmaBufPool(&source, &sink, 4096, 1);
    }" CAMERA_SDK_MPP_HAS_INPUT_POOL)
  cmake_pop_check_state()
  if(CAMERA_SDK_MPP_HAS_INPUT_POOL)
    target_compile_definitions(spacemit_camera_core PRIVATE CAMERA_SDK_MPP_HAS_INPUT_POOL=1)
  endif()
  set_target_properties(spacemit_camera_core PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

add_library(spacemit_camera_ahrs STATIC src/mahony_ahrs.cpp)
spacemit_camera_public_target(spacemit_camera_ahrs ahrs spacemit_camera_ahrs)
add_library(spacemit_camera_calibration STATIC src/kalibr_calibration.cpp)
spacemit_camera_public_target(spacemit_camera_calibration calibration spacemit_camera_calibration)
target_link_libraries(spacemit_camera_calibration PUBLIC yaml-cpp)
add_library(spacemit_camera_clock STATIC src/camera_clock.cpp src/gpio_exposure.cpp
  src/imu_frequency.cpp src/startup_alignment.cpp)
spacemit_camera_public_target(spacemit_camera_clock clock spacemit_camera_clock)
target_link_libraries(spacemit_camera_clock PUBLIC Threads::Threads
  PRIVATE $<BUILD_INTERFACE:Boost::boost>)

install(TARGETS spacemit_camera_core spacemit_camera_ahrs spacemit_camera_calibration
  spacemit_camera_clock EXPORT spacemit_cameraTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT spacemit_cameraTargets FILE spacemit_cameraTargets.cmake
  NAMESPACE spacemit_camera:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spacemit_camera)
configure_package_config_file(cmake/spacemit_cameraConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/spacemit_cameraConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spacemit_camera)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/spacemit_cameraConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spacemit_camera)
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/spacemit_camera)
