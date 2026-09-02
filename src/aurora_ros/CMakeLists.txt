cmake_minimum_required(VERSION 3.16)
project(aurora_ros LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(aurora_flight_adapter REQUIRED)
find_package(aurora_map REQUIRED)
find_package(aurora_math REQUIRED)
find_package(aurora_msgs REQUIRED)
find_package(aurora_planner_core REQUIRED)
find_package(aurora_prediction REQUIRED)
find_package(aurora_risk REQUIRED)
find_package(aurora_tracking REQUIRED)
find_package(aurora_trajectory REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(aurora_ros_adapters
  src/depth_image_adapter.cpp
  src/dynamic_obstacle_adapter.cpp
  src/unassociated_obstacle_adapter.cpp
)

target_compile_features(aurora_ros_adapters PUBLIC cxx_std_17)
target_compile_options(aurora_ros_adapters PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_ros_adapters
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_ros_adapters Eigen3::Eigen)
ament_target_dependencies(aurora_ros_adapters
  aurora_msgs
  aurora_prediction
  aurora_tracking
  sensor_msgs
)

add_executable(aurora_planner_node
  src/aurora_planner_node.cpp
)

target_compile_features(aurora_planner_node PRIVATE cxx_std_17)
target_compile_options(aurora_planner_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(aurora_planner_node Eigen3::Eigen)
target_link_libraries(aurora_planner_node aurora_ros_adapters)
ament_target_dependencies(aurora_planner_node
  aurora_flight_adapter
  aurora_map
  aurora_math
  aurora_msgs
  aurora_planner_core
  aurora_prediction
  aurora_risk
  aurora_tracking
  aurora_trajectory
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

install(TARGETS aurora_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS aurora_ros_adapters
  EXPORT export_aurora_ros_adapters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_ros_adapters HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_flight_adapter aurora_msgs aurora_prediction aurora_tracking sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_dynamic_obstacle_adapter
    test/test_dynamic_obstacle_adapter.cpp
  )
  target_link_libraries(test_dynamic_obstacle_adapter aurora_ros_adapters)
  target_compile_features(test_dynamic_obstacle_adapter PRIVATE cxx_std_17)
  ament_target_dependencies(test_dynamic_obstacle_adapter
    aurora_msgs
    aurora_prediction
  )

  ament_add_gtest(test_unassociated_obstacle_adapter
    test/test_unassociated_obstacle_adapter.cpp
  )
  target_link_libraries(test_unassociated_obstacle_adapter aurora_ros_adapters)
  target_compile_features(test_unassociated_obstacle_adapter PRIVATE cxx_std_17)
  ament_target_dependencies(test_unassociated_obstacle_adapter
    aurora_msgs
    aurora_prediction
    aurora_tracking
  )

  ament_add_gtest(test_depth_image_adapter
    test/test_depth_image_adapter.cpp
  )
  target_link_libraries(test_depth_image_adapter aurora_ros_adapters)
  target_compile_features(test_depth_image_adapter PRIVATE cxx_std_17)
  ament_target_dependencies(test_depth_image_adapter
    sensor_msgs
  )

  add_launch_test(test/test_planner_node_integration.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_planner_launch_test
  )
  add_launch_test(test/test_static_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_static_rosbag2_launch_test
  )
  add_launch_test(test/test_dynamic_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_dynamic_rosbag2_launch_test
  )
  add_launch_test(test/test_acceleration_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_acceleration_rosbag2_launch_test
  )
  add_launch_test(test/test_stopping_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_stopping_rosbag2_launch_test
  )
  add_launch_test(test/test_information_stale_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 90
    ENV ROS_LOG_DIR=/tmp/aurora_information_stale_rosbag2_launch_test
  )
  add_launch_test(test/test_dynamic_occlusion_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_dynamic_occlusion_rosbag2_launch_test
  )
  add_launch_test(test/test_internal_detection_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_internal_detection_ros2_launch_test
  )
  add_launch_test(test/test_dynamic_input_conflict_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_dynamic_input_conflict_ros2_launch_test
  )
  add_launch_test(test/test_internal_detection_lost_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_internal_detection_lost_ros2_launch_test
  )
  add_launch_test(test/test_map_quality_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 90
    ENV ROS_LOG_DIR=/tmp/aurora_map_quality_ros2_launch_test
  )
  add_launch_test(test/test_soft_risk_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 90
    ENV ROS_LOG_DIR=/tmp/aurora_soft_risk_ros2_launch_test
  )
  add_launch_test(test/test_internal_detection_rosbag2_replay.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
    ENV ROS_LOG_DIR=/tmp/aurora_internal_detection_rosbag2_launch_test
  )
  add_launch_test(test/test_planner_faults_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 90
    ENV ROS_LOG_DIR=/tmp/aurora_planner_faults_ros2_launch_test
  )
  add_launch_test(test/test_depth_image_ros2.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 90
    ENV ROS_LOG_DIR=/tmp/aurora_depth_image_ros2_launch_test
  )
endif()

ament_package()
