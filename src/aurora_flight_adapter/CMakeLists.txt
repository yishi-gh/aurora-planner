cmake_minimum_required(VERSION 3.16)
project(aurora_flight_adapter LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aurora_math REQUIRED)
find_package(aurora_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(aurora_flight_adapter
  src/trajectory_admission.cpp
)

target_compile_features(aurora_flight_adapter PUBLIC cxx_std_17)
target_compile_options(aurora_flight_adapter PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_flight_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_flight_adapter PUBLIC Eigen3::Eigen)
ament_target_dependencies(aurora_flight_adapter PUBLIC aurora_math aurora_msgs geometry_msgs)

install(
  TARGETS aurora_flight_adapter
  EXPORT export_aurora_flight_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_aurora_flight_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_math aurora_msgs geometry_msgs eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_admission
    test/test_trajectory_admission.cpp
  )
  target_link_libraries(test_trajectory_admission aurora_flight_adapter)
  target_compile_features(test_trajectory_admission PRIVATE cxx_std_17)
  ament_target_dependencies(test_trajectory_admission aurora_msgs geometry_msgs)
endif()

ament_package()
