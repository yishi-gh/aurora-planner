cmake_minimum_required(VERSION 3.16)
project(aurora_risk LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aurora_prediction REQUIRED)
find_package(aurora_trajectory REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aurora_risk
  src/dynamic_risk_evaluator.cpp
  src/risk_context.cpp
  src/risk_cost_field.cpp
)

target_compile_features(aurora_risk PUBLIC cxx_std_17)
target_compile_options(aurora_risk PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_risk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_risk PUBLIC Eigen3::Eigen)
ament_target_dependencies(aurora_risk PUBLIC aurora_prediction aurora_trajectory)

install(
  TARGETS aurora_risk
  EXPORT export_aurora_risk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_risk HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_prediction aurora_trajectory eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamic_risk_evaluator
    test/test_dynamic_risk_evaluator.cpp
  )
  target_link_libraries(test_dynamic_risk_evaluator aurora_risk)
  target_compile_features(test_dynamic_risk_evaluator PRIVATE cxx_std_17)

  ament_add_gtest(test_risk_context
    test/test_risk_context.cpp
  )
  target_link_libraries(test_risk_context aurora_risk)
  target_compile_features(test_risk_context PRIVATE cxx_std_17)

  ament_add_gtest(test_risk_cost_field
    test/test_risk_cost_field.cpp
  )
  target_link_libraries(test_risk_cost_field aurora_risk)
  target_compile_features(test_risk_cost_field PRIVATE cxx_std_17)
endif()

ament_package()
