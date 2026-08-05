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
        "msg/BmsLog.msg"
        "msg/BmsCmd.msg"
        "msg/BmsInfo.msg"
        "msg/MotorState.msg"
        "msg/MotorCmd.msg"
        "msg/IMUState.msg"
        "msg/SetFsmId.msg"
        "msg/SwitchUpperControl.msg"
        "msg/LowerCmd.msg"
        "msg/LowerState.msg"
        "msg/UpperCmd.msg"
        "msg/UpperState.msg"
        "msg/Velocity.msg"
        "msg/AxisStateInfo.msg"
        "msg/EcatSlaveInfo.msg"
        "msg/MainNodesState.msg"
        "msg/ClearErrors.msg"
        "msg/HandsCmd.msg"
        "msg/HandsState.msg"
        "msg/AMRBasicStatus.msg"
        "msg/AMREventStatus.msg"
        "msg/AMRCommand.msg"
        "msg/AMRState.msg"
        "msg/JoystickValue.msg"
        "msg/RemoteControl.msg"
        "msg/EmergencyState.msg"
        "msg/CalibrateImu.msg"
        "msg/CerebrumCmd.msg"
        "msg/CerebrumState.msg"
        "msg/ChangeMode.msg"
        "msg/DHGripperSingleCmd.msg"
        "msg/DHGripperCmd.msg"
        "msg/DHGripperSingleState.msg"
        "msg/DHGripperState.msg"
        "msg/EnableMotors.msg"
        "msg/GoHome.msg"
        "msg/InspireCmd.msg"
        "msg/InspireState.msg"
        "msg/LaserScan.msg"
        "msg/LowCmd.msg"
        "msg/LowState.msg"
        "msg/ModifyJointOffset.msg"
        "msg/Odom.msg"
        "msg/PlayTTS.msg"
        "msg/CartTargetPose.msg"
        "msg/PosControlJointQ.msg"
        "msg/PosControlCmd.msg"
        "msg/PosControlState.msg"
        "msg/PowerCmd.msg"
        "msg/PowerState.msg"
        "msg/SetCalibrationFlag.msg"
        "msg/TeleopCmd.msg"
        "msg/TeleopState.msg"
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
