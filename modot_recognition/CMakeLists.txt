cmake_minimum_required(VERSION 3.8)
project(modot_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake_auto REQUIRED)

find_package(PCL REQUIRED QUIET)

ament_auto_find_build_dependencies()

ament_auto_add_executable(obstacle_detector
  src/obstacle_detector_node.cpp
  src/obstacle_detector.cpp
)
target_link_libraries(obstacle_detector ${PCL_LIBRARIES})

ament_auto_package()
