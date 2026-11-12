cmake_minimum_required(VERSION 3.16)
project(common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DmaDepthFrameDesc.msg"
  "msg/FeaturePoint.msg"
  "msg/FeaturePointArray.msg"
  DEPENDENCIES std_msgs)

add_library(common_lifecycle SHARED src/async_lifecycle_node.cpp)
add_library(common::lifecycle ALIAS common_lifecycle)
set_target_properties(common_lifecycle PROPERTIES EXPORT_NAME lifecycle)
target_compile_features(common_lifecycle PUBLIC cxx_std_17)
target_include_directories(common_lifecycle PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(common_lifecycle PUBLIC rclcpp rclcpp_lifecycle)
target_link_libraries(common_lifecycle PRIVATE
  lifecycle_msgs::lifecycle_msgs__rosidl_generator_cpp std_msgs::std_msgs__rosidl_typesupport_cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS common_lifecycle EXPORT export_common_lifecycle LIBRARY DESTINATION lib)
ament_export_targets(export_common_lifecycle HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rosidl_default_runtime rclcpp rclcpp_lifecycle std_msgs)


ament_package()
