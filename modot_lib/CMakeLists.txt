cmake_minimum_required(VERSION 3.8)
project(modot_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(parameter_updater SHARED
  include/modot_lib/parameter_updater.hpp
)
set_target_properties(parameter_updater PROPERTIES LINKER_LANGUAGE CXX)

ament_auto_package()
