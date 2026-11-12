# Adapt the existing SDK IMU package at the consumer boundary. Its original
# shared library is reused; no DDS implementation is built or copied here.
if(NOT TARGET camera_io::imu_dds)
  find_package(peripherals_imu_node REQUIRED)

  set(_camera_io_imu_target "")
  foreach(_candidate peripherals_imu_node::peripherals_imu_node_lib
      peripherals_imu_node_lib peripherals_imu_node::dds)
    if(TARGET "${_candidate}")
      set(_camera_io_imu_target "${_candidate}")
      break()
    endif()
  endforeach()
  if(NOT _camera_io_imu_target)
    message(FATAL_ERROR "peripherals_imu_node does not export its IMU DDS library")
  endif()

  set(_camera_io_imu_header_hints ${peripherals_imu_node_INCLUDE_DIRS})
  if(IMU_DDS_ROOT)
    list(APPEND _camera_io_imu_header_hints "${IMU_DDS_ROOT}/src/impl")
  endif()
  if(DEFINED ENV{IMU_DDS_ROOT})
    list(APPEND _camera_io_imu_header_hints "$ENV{IMU_DDS_ROOT}/src/impl")
  endif()
  foreach(_sdk_root "${SROBOTIS_ROOT}" "$ENV{SROBOTIS_ROOT}")
    if(_sdk_root)
      list(APPEND _camera_io_imu_header_hints
        "${_sdk_root}/middleware/ros2/peripherals/imu/src/impl")
    endif()
  endforeach()

  # Works both in the source checkout and beneath its output/staging prefix.
  set(_camera_io_sdk_search "${CMAKE_CURRENT_LIST_DIR}")
  while(NOT _camera_io_sdk_search STREQUAL "/")
    if(EXISTS "${_camera_io_sdk_search}/build/envsetup.sh")
      list(APPEND _camera_io_imu_header_hints
        "${_camera_io_sdk_search}/middleware/ros2/peripherals/imu/src/impl")
      break()
    endif()
    get_filename_component(_camera_io_sdk_search "${_camera_io_sdk_search}" DIRECTORY)
  endwhile()
  foreach(_include_dir IN LISTS peripherals_imu_node_INCLUDE_DIRS)
    list(APPEND _camera_io_imu_header_hints "${_include_dir}/peripherals_imu_node/dds")
  endforeach()
  # Original SDK releases do not install these generated public interfaces.
  # A source checkout is therefore required when compiling their consumers.
  find_path(CAMERA_IO_IMU_DDS_INCLUDE_DIR ImuPublisher.h
    HINTS ${_camera_io_imu_header_hints} NO_DEFAULT_PATH)
  if(NOT CAMERA_IO_IMU_DDS_INCLUDE_DIR OR
      NOT EXISTS "${CAMERA_IO_IMU_DDS_INCLUDE_DIR}/Imu.h")
    message(FATAL_ERROR
      "SDK IMU DDS headers are missing; set IMU_DDS_ROOT to the unchanged "
      "middleware/ros2/peripherals/imu source directory")
  endif()
  add_library(camera_io::imu_dds INTERFACE IMPORTED)
  set_target_properties(camera_io::imu_dds PROPERTIES
    INTERFACE_LINK_LIBRARIES "${_camera_io_imu_target}"
    INTERFACE_INCLUDE_DIRECTORIES "${CAMERA_IO_IMU_DDS_INCLUDE_DIR}")
  unset(_camera_io_imu_target)
  unset(_camera_io_imu_header_hints)
  unset(_camera_io_sdk_search)
  unset(_candidate)
  unset(_sdk_root)
  unset(_include_dir)
endif()
