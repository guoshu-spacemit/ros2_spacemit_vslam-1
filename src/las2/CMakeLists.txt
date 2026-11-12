cmake_minimum_required(VERSION 3.16)
project(las2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mpp_msgs REQUIRED)
find_package(common REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(OpenCL REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/RobotSDK.cmake")
spacemit_find_mpp()
find_package(stereo_matching CONFIG REQUIRED COMPONENTS external_input)
get_target_property(LAS2_INCLUDE_DIR stereo_matching::external_input INTERFACE_INCLUDE_DIRECTORIES)
set(LAS2_LIBRARY stereo_matching::external_input)

add_library(las2_component SHARED src/las2_node.cpp src/nv12_center_hud.cpp)
target_include_directories(las2_component PRIVATE
  include
  "${LAS2_INCLUDE_DIR}"
  "${MPP_INCLUDE_DIR}")
target_link_libraries(las2_component
  "${LAS2_LIBRARY}"
  "${MPP_LIBRARY}"
  OpenCL::OpenCL
  pthread)
ament_target_dependencies(las2_component
  builtin_interfaces rclcpp rclcpp_components std_msgs nav_msgs mpp_msgs common)
rclcpp_components_register_node(
  las2_component
  PLUGIN "las2::RobotLas2Node"
  EXECUTABLE las2_node
  EXECUTOR MultiThreadedExecutor)

install(TARGETS las2_component LIBRARY DESTINATION lib)

add_executable(las2_pipeline_node src/las2_pipeline_node.cpp)
target_include_directories(las2_pipeline_node PRIVATE "${LAS2_INCLUDE_DIR}" "${MPP_INCLUDE_DIR}")
target_link_libraries(las2_pipeline_node "${LAS2_LIBRARY}" "${MPP_LIBRARY}" pthread)
ament_target_dependencies(las2_pipeline_node rclcpp std_msgs mpp_msgs common)
install(TARGETS las2_pipeline_node DESTINATION lib/${PROJECT_NAME})

add_executable(las2_readiness_barrier src/readiness_barrier.cpp)
ament_target_dependencies(las2_readiness_barrier rclcpp std_msgs)
install(TARGETS las2_readiness_barrier DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
add_executable(quad_preview_node src/quad_preview_node.cpp)
target_include_directories(quad_preview_node PRIVATE "${MPP_INCLUDE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(quad_preview_node "${MPP_LIBRARY}" ${OpenCV_LIBS})
ament_target_dependencies(quad_preview_node rclcpp common)
install(TARGETS quad_preview_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/quad_preview_web.py DESTINATION lib/${PROJECT_NAME})
ament_package()
