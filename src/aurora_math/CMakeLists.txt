cmake_minimum_required(VERSION 3.16)
project(aurora_math LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aurora_math
  src/minimum_snap.cpp
  src/strict_minimum_snap.cpp
  src/time_allocation.cpp
  src/path_resampler.cpp
  src/uniform_bspline.cpp
)

target_compile_features(aurora_math PUBLIC cxx_std_17)
target_compile_options(aurora_math PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_math PUBLIC Eigen3::Eigen)

install(
  TARGETS aurora_math
  EXPORT export_aurora_math
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_math HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uniform_bspline
    test/test_uniform_bspline.cpp
  )
  target_link_libraries(test_uniform_bspline aurora_math)
  target_compile_features(test_uniform_bspline PRIVATE cxx_std_17)

  ament_add_gtest(test_minimum_snap
    test/test_minimum_snap.cpp
  )
  target_link_libraries(test_minimum_snap aurora_math)
  target_compile_features(test_minimum_snap PRIVATE cxx_std_17)

  ament_add_gtest(test_strict_minimum_snap
    test/test_strict_minimum_snap.cpp
  )
  target_link_libraries(test_strict_minimum_snap aurora_math)
  target_compile_features(test_strict_minimum_snap PRIVATE cxx_std_17)

  ament_add_gtest(test_time_allocation
    test/test_time_allocation.cpp
  )
  target_link_libraries(test_time_allocation aurora_math)
  target_compile_features(test_time_allocation PRIVATE cxx_std_17)

  ament_add_gtest(test_path_resampler
    test/test_path_resampler.cpp
  )
  target_link_libraries(test_path_resampler aurora_math)
  target_compile_features(test_path_resampler PRIVATE cxx_std_17)
endif()

ament_package()
