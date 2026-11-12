cmake_minimum_required(VERSION 3.16)
project(esdf_drm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mpp_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)
pkg_check_modules(GLES REQUIRED IMPORTED_TARGET glesv2)
pkg_check_modules(GBM REQUIRED IMPORTED_TARGET gbm)
pkg_check_modules(LIBDRM REQUIRED IMPORTED_TARGET libdrm)

include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/RobotSDK.cmake")
spacemit_find_mpp()
find_package(spacemit_mapping CONFIG REQUIRED COMPONENTS functional)

add_library(esdf_drm_component SHARED
  src/drm_recorder.cpp
  src/esdf_update_pipeline.cpp
  src/feature_overlay_panel.cpp
  src/gles_render_pipeline.cpp
  src/mouse_input.cpp
  src/performance_hud.cpp
  src/esdf_drm_node.cpp)
target_include_directories(esdf_drm_component PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${MPP_INCLUDE_DIR}"
  "${MPP_INCLUDE_DIR}/sys")
target_link_libraries(esdf_drm_component
  spacemit_mapping::spacemit_mapping
  "${MPP_LIBRARY}"
  PkgConfig::EGL
  PkgConfig::GLES
  PkgConfig::GBM
  PkgConfig::LIBDRM
  Threads::Threads
  ${CMAKE_DL_LIBS})
ament_target_dependencies(esdf_drm_component
  rclcpp rclcpp_components mpp_msgs nav_msgs sensor_msgs std_msgs common)
rclcpp_components_register_node(
  esdf_drm_component
  PLUGIN "esdf_drm::RobotEsdfDrmNode"
  EXECUTABLE esdf_drm_node
  EXECUTOR MultiThreadedExecutor)

install(TARGETS esdf_drm_component LIBRARY DESTINATION lib)

option(SPACEMIT_ROBOT_ESDF_BUILD_DESKTOP_VIEWER "Build the X11/Xwayland DMA-BUF viewer" ON)
if(SPACEMIT_ROBOT_ESDF_BUILD_DESKTOP_VIEWER)
  # x11.pc can depend on protocol-only .pc files absent from deployment
  # sysroots. The viewer needs only Xlib, not those build-time protocol packages.
  find_path(SPACEMIT_ROBOT_X11_INCLUDE_DIR X11/Xlib.h REQUIRED)
  find_library(SPACEMIT_ROBOT_X11_LIBRARY NAMES X11 REQUIRED)
  add_executable(esdf_desktop_viewer src/desktop_viewer.cpp)
  target_include_directories(esdf_desktop_viewer PRIVATE
    "${SPACEMIT_ROBOT_X11_INCLUDE_DIR}" "${MPP_INCLUDE_DIR}" "${MPP_INCLUDE_DIR}/sys")
  target_link_libraries(esdf_desktop_viewer
    "${MPP_LIBRARY}" "${SPACEMIT_ROBOT_X11_LIBRARY}" PkgConfig::EGL PkgConfig::GLES
    PkgConfig::LIBDRM Threads::Threads)
  ament_target_dependencies(esdf_desktop_viewer rclcpp mpp_msgs)
  install(TARGETS esdf_desktop_viewer DESTINATION lib/${PROJECT_NAME})
endif()


ament_package()
