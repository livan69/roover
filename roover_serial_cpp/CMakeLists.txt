cmake_minimum_required(VERSION 3.8)
project(roover_serial_cpp)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# zet de include-dir, zodat #include "roover_serial_cpp/serial_command.hpp" werkt
include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
)

# --- Dependencies ----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(roover_f710_input REQUIRED)
find_package(geometry_msgs REQUIRED)


# --- Message interface -----------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RooverFeedback.msg"
)

# --- Feedback-node ---------------------------------------------------------
add_executable(serial_feedback_node
  src/serial_feedback_node.cpp
)
ament_target_dependencies(serial_feedback_node
  rclcpp
)
rosidl_target_interfaces(serial_feedback_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --- Command-node ----------------------------------------------------------
add_executable(serial_command_node
  src/serial_command_node.cpp
)
ament_target_dependencies(serial_command_node
  rclcpp
  geometry_msgs
)

# --- Install ---------------------------------------------------------------
install(
  TARGETS
    serial_feedback_node
    serial_command_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#Afsluiten
ament_package()
