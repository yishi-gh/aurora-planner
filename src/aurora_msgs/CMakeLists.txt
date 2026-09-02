cmake_minimum_required(VERSION 3.16)
project(aurora_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/DynamicObstacleTrack.msg"
  "msg/DynamicObstacleTrackArray.msg"
  "msg/DynamicObstaclePrediction.msg"
  "msg/EmergencyStopState.msg"
  "msg/GlobalReferencePoint.msg"
  "msg/PlannerStatus.msg"
  "msg/PlanningRequest.msg"
  "msg/PlanningResult.msg"
  "msg/PredictedState.msg"
  "msg/RiskReport.msg"
  "msg/SafetyReport.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryExecutionStatus.msg"
  "msg/TrajectoryConstraints.msg"
  "msg/TrajectorySegment.msg"
  "msg/UnassociatedObstacleDetection.msg"
  "msg/UnassociatedObstacleDetectionArray.msg"
  "msg/VehicleState.msg"
)

set(srv_files
  "srv/SetEmergencyStop.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
