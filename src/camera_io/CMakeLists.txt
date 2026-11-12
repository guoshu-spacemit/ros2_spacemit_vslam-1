cmake_minimum_required(VERSION 3.16)
project(camera_io VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(spacemit_camera CONFIG REQUIRED)
find_package(Boost REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mpp_msgs REQUIRED)
find_package(common REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/RobotSDK.cmake")
include(cmake/imu_dds_compatibility.cmake)
spacemit_find_mpp()

include(CheckCXXSourceCompiles)
include(CMakePushCheckState)
find_package(Threads REQUIRED)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_INCLUDES ${MPP_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES "${MPP_LIBRARY}" Threads::Threads)
get_filename_component(_mpp_library_dir "${MPP_LIBRARY}" DIRECTORY)
set(CMAKE_REQUIRED_LINK_OPTIONS "-Wl,-rpath-link,${_mpp_library_dir}")
unset(CAMERA_MPP_HAS_INPUT_POOL CACHE)
check_cxx_source_compiles("
  #include <sys/sys_api.h>
  #include <vdec/vdec_api.h>
  int main() {
    VdecChnAttr attr{};
    attr.bEnableInputDmaBuf = MPP_TRUE;
    MppNode source{}, sink{};
    return SYS_ConfigStreamDmaBufPool(&source, &sink, 4096, 1);
  }" CAMERA_MPP_HAS_INPUT_POOL)
cmake_pop_check_state()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SESSION_AV REQUIRED IMPORTED_TARGET libavformat libavcodec libavutil)
add_library(camera_interfaces SHARED src/ros/camera_info.cpp)
set_target_properties(camera_interfaces PROPERTIES EXPORT_NAME interfaces)
add_library(camera_io::interfaces ALIAS camera_interfaces)
target_compile_features(camera_interfaces PUBLIC cxx_std_17)
target_compile_options(camera_interfaces PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(camera_interfaces PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_interfaces PUBLIC spacemit_camera::calibration)
ament_target_dependencies(camera_interfaces PUBLIC sensor_msgs std_msgs)

add_library(camera_imu_transport STATIC src/camera/imu_transport.cpp)
set_target_properties(camera_imu_transport PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(camera_imu_transport PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_imu_transport PUBLIC camera_io::imu_dds)
ament_target_dependencies(camera_imu_transport PUBLIC sensor_msgs)

add_library(camera_mpp_component SHARED
  src/camera/camera_mpp_publisher.cpp
  src/camera/camera_mpp_config.cpp
  src/camera/camera_mpp_capture.cpp
  src/camera/camera_mpp_frames.cpp)
target_compile_options(camera_mpp_component PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(camera_mpp_component PRIVATE ${MPP_INCLUDE_DIRS})
target_link_libraries(camera_mpp_component PRIVATE common::lifecycle camera_imu_transport
  spacemit_camera::camera spacemit_camera::ahrs spacemit_camera::calibration spacemit_camera::clock
  "${MPP_LIBRARY}" Threads::Threads Boost::boost PkgConfig::SESSION_AV)
ament_target_dependencies(camera_mpp_component PUBLIC rclcpp rclcpp_components
  rclcpp_lifecycle sensor_msgs std_msgs geometry_msgs tf2_ros mpp_msgs common)
rclcpp_components_register_node(camera_mpp_component PLUGIN "camera_io::CameraMppPublisher"
  EXECUTABLE camera_mpp_publisher EXECUTOR MultiThreadedExecutor)

add_executable(session_replay_publisher src/camera/session_replay_publisher.cpp)
target_compile_options(session_replay_publisher PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(session_replay_publisher PRIVATE ${MPP_INCLUDE_DIRS})
target_link_libraries(session_replay_publisher PRIVATE camera_mpp_component common::lifecycle
  spacemit_camera::camera spacemit_camera::ahrs spacemit_camera::calibration spacemit_camera::clock
  "${MPP_LIBRARY}" Threads::Threads Boost::boost PkgConfig::SESSION_AV)
ament_target_dependencies(session_replay_publisher PUBLIC rclcpp rclcpp_components
  rclcpp_lifecycle sensor_msgs std_msgs geometry_msgs nav_msgs tf2_ros mpp_msgs common)
if(CAMERA_MPP_HAS_INPUT_POOL)
  target_compile_definitions(camera_mpp_component PRIVATE CAMERA_MPP_HAS_INPUT_POOL=1)
endif()

add_executable(camera_agc_node src/camera/camera_agc_node.cpp)
target_include_directories(camera_agc_node PRIVATE ${MPP_INCLUDE_DIRS})
target_link_libraries(camera_agc_node PRIVATE spacemit_camera::camera "${MPP_LIBRARY}")
ament_target_dependencies(camera_agc_node PUBLIC rclcpp mpp_msgs)

add_executable(recorder_node src/recorder/recorder_node.cpp)
target_compile_options(recorder_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(recorder_node PRIVATE ${MPP_INCLUDE_DIRS})
target_link_libraries(recorder_node PRIVATE "${MPP_LIBRARY}" PkgConfig::SESSION_AV
  Threads::Threads Boost::boost)
ament_target_dependencies(recorder_node PUBLIC rclcpp sensor_msgs mpp_msgs)

install(TARGETS camera_interfaces
  EXPORT camera_io_targets LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS camera_mpp_component LIBRARY DESTINATION lib)
install(TARGETS camera_agc_node session_replay_publisher recorder_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
ament_export_targets(camera_io_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(spacemit_camera sensor_msgs std_msgs)
ament_package()
