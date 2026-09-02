cmake_minimum_required(VERSION 3.16)
project(aurora_map LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aurora_map
  src/voxel_map.cpp
)

target_compile_features(aurora_map PUBLIC cxx_std_17)
target_compile_options(aurora_map PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_map PUBLIC Eigen3::Eigen)

install(
  TARGETS aurora_map
  EXPORT export_aurora_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_map HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_map
    test/test_voxel_map.cpp
  )
  target_link_libraries(test_voxel_map aurora_map)
  target_compile_features(test_voxel_map PRIVATE cxx_std_17)
endif()

ament_package()
