cmake_minimum_required(VERSION 3.16)
project(aurora_trajectory LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aurora_map REQUIRED)
find_package(aurora_math REQUIRED)

add_library(aurora_trajectory
  src/static_bspline_optimizer.cpp
)

target_compile_features(aurora_trajectory PUBLIC cxx_std_17)
target_compile_options(aurora_trajectory PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_trajectory
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aurora_trajectory PUBLIC aurora_map aurora_math)

install(
  TARGETS aurora_trajectory
  EXPORT export_aurora_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_trajectory HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_map aurora_math)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_static_bspline_optimizer
    test/test_static_bspline_optimizer.cpp
  )
  target_link_libraries(test_static_bspline_optimizer aurora_trajectory)
  target_compile_features(test_static_bspline_optimizer PRIVATE cxx_std_17)

  find_package(aurora_search REQUIRED)
  ament_add_gtest(test_static_planner_pipeline
    test/test_static_planner_pipeline.cpp
  )
  target_link_libraries(test_static_planner_pipeline aurora_trajectory)
  ament_target_dependencies(test_static_planner_pipeline aurora_search)
  target_compile_features(test_static_planner_pipeline PRIVATE cxx_std_17)
endif()

ament_package()
