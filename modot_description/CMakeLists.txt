cmake_minimum_required(VERSION 3.8)
project(modot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/camera_tf_publisher.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "modot_description::CameraTFPublisher"
  EXECUTABLE camera_tf_publisher_node
)

ament_auto_package()
