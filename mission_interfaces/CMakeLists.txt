cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # flight_controller / bench_flight_controller
  "action/Takeoff.action"
  "action/GoTo.action"
  "action/Land.action"
  "action/Survey.action"
  "msg/VehicleState.msg"
  "msg/MotorStatus.msg"
  "msg/PositionSetpoint.msg"
  # perception
  "msg/BannerDetection.msg"
  "msg/QrDetection.msg"
  "srv/SetBannerColor.srv"
  "msg/RedPatch.msg"
  "msg/RedZoneResult.msg"
  "msg/RedMap.msg"
  "action/CaptureReference.action"
  # behaviours
  "action/SearchBanner.action"
  "action/AlignBanner.action"
  "action/TraverseCorridor.action"
  "action/SearchTarget.action"
  "action/SafeGoto.action"
  "action/LawnmowerSearch.action"
  "action/CenterOnQr.action"
  "action/NavigateToTarget.action"
  "action/PayloadDrop.action"
  # mission tree -> HUD
  "msg/MissionStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
