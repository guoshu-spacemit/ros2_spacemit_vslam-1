cmake_minimum_required(VERSION 3.16)
project(vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(mpp_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/RobotSDK.cmake")
spacemit_find_mpp()
find_package(spacemit_camera CONFIG REQUIRED)
find_package(spacemit_vslam CONFIG REQUIRED COMPONENTS functional)

add_library(vio_component SHARED
  src/frame_recorder.cpp
  src/imu_propagator.cpp
  src/vio_node.cpp)
target_include_directories(vio_component PRIVATE
  include
  "${MPP_INCLUDE_DIR}")
target_link_libraries(vio_component
  spacemit_vslam::spacemit_vslam
  "${MPP_LIBRARY}"
  yaml-cpp
  Eigen3::Eigen spacemit_camera::clock)
ament_target_dependencies(vio_component
  rclcpp rclcpp_components sensor_msgs std_msgs nav_msgs geometry_msgs message_filters
  mpp_msgs tf2_ros common visualization_msgs std_srvs)
rclcpp_components_register_node(
  vio_component
  PLUGIN "vio::RobotVioNode"
  EXECUTABLE vio_node
  EXECUTOR MultiThreadedExecutor)

install(TARGETS vio_component
  LIBRARY DESTINATION lib)


ament_package()
