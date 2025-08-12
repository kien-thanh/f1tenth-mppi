cmake_minimum_required(VERSION 3.5)
project(wall_follow)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  control_toolbox
  std_msgs
  rosidl_default_generators
  builtin_interfaces
  )

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Distance.msg"
  "msg/PID.msg"
  "msg/Steering.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Create Cpp executable
add_executable(wall_follow_node src/wall_follow_node.cpp)
ament_target_dependencies(wall_follow_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs control_toolbox std_msgs
)

add_dependencies(wall_follow_node ${PROJECT_NAME}__rosidl_generator_cpp)

rosidl_target_interfaces(wall_follow_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install Cpp executables
install(TARGETS
  wall_follow_node
  DESTINATION lib/${PROJECT_NAME})

# Install config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
