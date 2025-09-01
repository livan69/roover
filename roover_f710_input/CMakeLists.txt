cmake_minimum_required(VERSION 3.8)
project(roover_f710_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ====================================================
# SDL2 instellen
# ====================================================
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/F710Input.msg"
  DEPENDENCIES std_msgs
)

add_executable(f710_node src/f710_node.cpp)
ament_target_dependencies(f710_node
  rclcpp
  std_msgs
)

# nodig voor de typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(f710_node
  ${SDL2_LIBRARIES}        # voeg dit toe
  ${cpp_typesupport_target} # je had dit al voor de message-typesupport
)

install(TARGETS f710_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
