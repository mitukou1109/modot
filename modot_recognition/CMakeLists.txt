cmake_minimum_required(VERSION 3.8)
project(modot_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(PCL REQUIRED QUIET)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/obstacle_detector.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "modot_recognition::ObstacleDetector"
  EXECUTABLE obstacle_detector_node
)

ament_auto_package()
