cmake_minimum_required(VERSION 3.8)
project(atom_control_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dobot_atom REQUIRED)
find_package(std_msgs REQUIRED)

# Create src directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add executables
add_executable(lower_state_reader src/lower_state_reader.cpp)
ament_target_dependencies(lower_state_reader rclcpp dobot_atom std_msgs)

add_executable(upper_state_reader src/upper_state_reader.cpp)
ament_target_dependencies(upper_state_reader rclcpp dobot_atom std_msgs)

add_executable(main_nodes_state_reader src/main_nodes_state_reader.cpp)
ament_target_dependencies(main_nodes_state_reader rclcpp dobot_atom std_msgs)

add_executable(hands_state_reader src/hands_state_reader.cpp)
ament_target_dependencies(hands_state_reader rclcpp dobot_atom std_msgs)

add_executable(atom_head_controller src/atom_head_controller.cpp)
ament_target_dependencies(atom_head_controller rclcpp dobot_atom std_msgs)

# Add new SDK-ported executables
add_executable(atom_full_body_trajectory_controller src/atom_full_body_trajectory_controller.cpp)
ament_target_dependencies(atom_full_body_trajectory_controller rclcpp dobot_atom std_msgs)

add_executable(atom_velocity_controller src/atom_velocity_controller.cpp)
ament_target_dependencies(atom_velocity_controller rclcpp dobot_atom std_msgs geometry_msgs)

add_executable(atom_joint_recorder src/atom_joint_recorder.cpp)
ament_target_dependencies(atom_joint_recorder rclcpp dobot_atom std_msgs)

# Install executables
install(TARGETS
  lower_state_reader
  upper_state_reader
  main_nodes_state_reader
  hands_state_reader
  atom_head_controller
  atom_full_body_trajectory_controller
  atom_velocity_controller
  atom_joint_recorder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()