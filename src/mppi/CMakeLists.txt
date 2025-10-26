cmake_minimum_required(VERSION 3.5)

set(CMAKE_CUDA_ARCHITECTURES 86)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/MPPI-Generic/cmake")

project(mppi LANGUAGES CXX CUDA)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/MPPI-Generic)
include(MPPIGenericToolsConfig)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

include_directories(include/mppi src)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mppi_node src/mppi_node.cu)

ament_target_dependencies(mppi_node
  rclcpp
  ackermann_msgs
  nav_msgs
  interfaces
  tf2_ros
  tf2_geometry_msgs
)


target_include_directories(mppi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(mppi_node MPPI::MPPI)

install(TARGETS mppi_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
