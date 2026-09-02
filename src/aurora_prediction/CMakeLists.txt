cmake_minimum_required(VERSION 3.16)
project(aurora_prediction LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aurora_prediction
  src/kinematic_predictor.cpp
)

target_compile_features(aurora_prediction PUBLIC cxx_std_17)
target_compile_options(aurora_prediction PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_prediction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_prediction PUBLIC Eigen3::Eigen)

install(
  TARGETS aurora_prediction
  EXPORT export_aurora_prediction
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_prediction HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematic_predictor
    test/test_kinematic_predictor.cpp
  )
  target_link_libraries(test_kinematic_predictor aurora_prediction)
  target_compile_features(test_kinematic_predictor PRIVATE cxx_std_17)
endif()

ament_package()
