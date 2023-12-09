cmake_minimum_required(VERSION 3.8)
project(modot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(camera_tf_publisher
  src/camera_tf_publisher_node.cpp
  src/camera_tf_publisher.cpp
)

ament_auto_package()
