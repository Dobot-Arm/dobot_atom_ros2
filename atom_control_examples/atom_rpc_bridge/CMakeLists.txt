cmake_minimum_required(VERSION 3.8)
project(atom_rpc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 使用独立的RPC客户端实现，无需外部依赖
message(STATUS "Using standalone RPC client implementation")

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)

# 独立RPC客户端无需额外包含目录

# Create library
add_library(${PROJECT_NAME}
  src/atom_dds_client.cpp
  src/standalone_rpc_client.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

# 独立RPC客户端无需外部库依赖
message(STATUS "Using standalone RPC implementation - no external libraries required")

# Create executables
add_executable(atom_dds_test
  src/atom_dds_test.cpp
)

add_executable(ros2_control_example
  examples/ros2_control_example.cpp
)

ament_target_dependencies(atom_dds_test
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

ament_target_dependencies(ros2_control_example
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(atom_dds_test
  ${PROJECT_NAME}
)

target_link_libraries(ros2_control_example
  ${PROJECT_NAME}
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  atom_dds_test
  ros2_control_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install examples
install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

ament_package()