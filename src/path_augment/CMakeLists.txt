cmake_minimum_required(VERSION 3.8)
project(path_augment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)

add_executable(path_augment src/path_augment.cpp)

ament_target_dependencies(path_augment
  rclcpp
  nav_msgs
  vehicle_msgs
)

install(TARGETS
  path_augment
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
