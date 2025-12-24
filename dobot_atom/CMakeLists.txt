cmake_minimum_required(VERSION 3.5)
project(dobot_atom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/BmsState.msg"
        "msg/MotorState.msg"
        "msg/MotorCmd.msg"
        "msg/IMUState.msg"
        "msg/SetFsmId.msg"
        "msg/SwitchUpperControl.msg"
        "msg/UpperState.msg"
        "msg/UpperCmd.msg"
        "msg/LowerState.msg"
        "msg/LowerCmd.msg"
        "msg/AxisStateInfo.msg"
        "msg/EcatSlaveInfo.msg"
        "msg/MainNodesState.msg"
        "msg/ClearErrors.msg"
        "msg/HandsState.msg"
        "msg/HandsCmd.msg"
        "msg/AMRState.msg"
        "msg/AMRCommand.msg"
        "msg/AMREventStatus.msg"
        "msg/AMRBasicStatus.msg"
        "msg/JoystickValue.msg"
        "msg/RemoteControl.msg"
        "msg/EmergencyState.msg"
  DEPENDENCIES geometry_msgs
)

rosidl_generate_dds_interfaces(
  ${rosidl_generate_interfaces_TARGET}__dds_connext_idl
  IDL_TUPLES ${rosidl_generate_interfaces_IDL_TUPLES}
  OUTPUT_SUBFOLDERS "dds_connext"
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}__dds_connext_idl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()