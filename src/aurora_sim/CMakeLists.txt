cmake_minimum_required(VERSION 3.16)
project(aurora_sim)

find_package(ament_cmake REQUIRED)
find_package(aurora_flight_adapter REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aurora_math REQUIRED)
find_package(aurora_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(aurora_trajectory_executor STATIC
  src/trajectory_executor.cpp
)
target_compile_features(aurora_trajectory_executor PUBLIC cxx_std_17)
target_include_directories(aurora_trajectory_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aurora_trajectory_executor
  Eigen3
  aurora_math
)

add_executable(aurora_sim_node src/aurora_sim_node.cpp)
target_compile_features(aurora_sim_node PRIVATE cxx_std_17)
target_link_libraries(aurora_sim_node aurora_trajectory_executor)
ament_target_dependencies(aurora_sim_node
  aurora_flight_adapter
  Eigen3
  aurora_math
  aurora_msgs
  geometry_msgs
  rclcpp
)

install(
  TARGETS aurora_trajectory_executor aurora_sim_node
  EXPORT export_aurora_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_aurora_sim HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_flight_adapter aurora_math eigen3_cmake_module Eigen3 aurora_msgs geometry_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_trajectory_executor test/test_trajectory_executor.cpp)
  target_link_libraries(test_trajectory_executor aurora_trajectory_executor)
  target_compile_features(test_trajectory_executor PRIVATE cxx_std_17)
  ament_target_dependencies(test_trajectory_executor aurora_math)

  add_launch_test(test/test_simulation_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 90
    ENV ROS_LOG_DIR=/tmp/aurora_simulation_ros2_launch_test
  )
endif()

ament_package()
