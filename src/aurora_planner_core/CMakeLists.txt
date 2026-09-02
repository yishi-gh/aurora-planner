cmake_minimum_required(VERSION 3.16)
project(aurora_planner_core LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aurora_map REQUIRED)
find_package(aurora_math REQUIRED)
find_package(aurora_search REQUIRED)
find_package(aurora_trajectory REQUIRED)

add_library(aurora_planner_core
  src/static_local_planner.cpp
  src/static_safety_gate.cpp
)

target_compile_features(aurora_planner_core PUBLIC cxx_std_17)
target_compile_options(aurora_planner_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aurora_planner_core PUBLIC
  aurora_map aurora_math aurora_search aurora_trajectory
)

install(
  TARGETS aurora_planner_core
  EXPORT export_aurora_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

option(AURORA_BUILD_BENCHMARKS "Build AURORA core performance benchmarks" OFF)
if(AURORA_BUILD_BENCHMARKS)
  add_executable(aurora_static_planner_benchmark
    benchmark/static_planner_benchmark.cpp
  )
  target_link_libraries(aurora_static_planner_benchmark aurora_planner_core)
  target_compile_features(aurora_static_planner_benchmark PRIVATE cxx_std_17)
  target_compile_options(aurora_static_planner_benchmark PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS aurora_static_planner_benchmark
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_targets(export_aurora_planner_core HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_map aurora_math aurora_search aurora_trajectory)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_static_local_planner
    test/test_static_local_planner.cpp
  )
  target_link_libraries(test_static_local_planner aurora_planner_core)
  target_compile_features(test_static_local_planner PRIVATE cxx_std_17)

  ament_add_gtest(test_static_safety_gate
    test/test_static_safety_gate.cpp
  )
  target_link_libraries(test_static_safety_gate aurora_planner_core)
  target_compile_features(test_static_safety_gate PRIVATE cxx_std_17)
endif()

ament_package()
