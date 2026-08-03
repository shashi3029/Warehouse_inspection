cmake_minimum_required(VERSION 3.8)
project(warehouse_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/BatteryStatus.msg"
  "msg/TaskStatus.msg"
  "msg/FleetStatus.msg"
  "msg/Detection.msg"
  "msg/ChargingStatus.msg"
  "msg/MissionStatus.msg"
  "msg/RobotInfo.msg"
  "srv/AssignTask.srv"
  "srv/GetRobotState.srv"
  "srv/EmergencyStop.srv"
  "srv/GetNearestCharger.srv"
  "action/ExecuteMission.action"
  "action/NavigateRobot.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
