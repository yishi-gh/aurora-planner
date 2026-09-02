cmake_minimum_required(VERSION 3.16)
project(aurora_tracking LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aurora_prediction REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aurora_tracking
  src/obstacle_tracker.cpp
)

target_compile_features(aurora_tracking PUBLIC cxx_std_17)
target_compile_options(aurora_tracking PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_tracking
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_tracking Eigen3::Eigen)
ament_target_dependencies(aurora_tracking aurora_prediction)

install(
  TARGETS aurora_tracking
  EXPORT export_aurora_tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_tracking HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_prediction eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_obstacle_tracker
    test/test_obstacle_tracker.cpp
  )
  target_link_libraries(test_obstacle_tracker aurora_tracking)
  target_compile_features(test_obstacle_tracker PRIVATE cxx_std_17)
endif()

ament_package()
