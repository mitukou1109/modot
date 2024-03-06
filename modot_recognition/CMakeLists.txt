cmake_minimum_required(VERSION 3.8)
project(modot_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/obstacle_detector.cpp
  src/face_detection_visualizer.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "modot_recognition::ObstacleDetector"
  EXECUTABLE obstacle_detector_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "modot_recognition::FaceDetectionVisualizer"
  EXECUTABLE face_detection_visualizer_node
)

ament_auto_package()
