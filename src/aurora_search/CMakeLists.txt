cmake_minimum_required(VERSION 3.16)
project(aurora_search LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(aurora_map REQUIRED)

add_library(aurora_search
  src/astar_3d.cpp
)

target_compile_features(aurora_search PUBLIC cxx_std_17)
target_compile_options(aurora_search PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(aurora_search
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aurora_search PUBLIC aurora_map)

install(
  TARGETS aurora_search
  EXPORT export_aurora_search
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_aurora_search HAS_LIBRARY_TARGET)
ament_export_dependencies(aurora_map)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar_3d
    test/test_astar_3d.cpp
  )
  target_link_libraries(test_astar_3d aurora_search)
  target_compile_features(test_astar_3d PRIVATE cxx_std_17)
endif()

ament_package()
