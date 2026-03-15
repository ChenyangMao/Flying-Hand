cmake_minimum_required(VERSION 3.8)
project(base_main_class)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Library used by pose_controller and wrench_controller nodes
add_library(base_main_class SHARED
  src/Base.cpp
  src/BaseNode.cpp
)
target_include_directories(base_main_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(base_main_class rclcpp)

install(TARGETS base_main_class
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_include_directories(include)
ament_export_libraries(base_main_class)
ament_export_dependencies(rclcpp)

ament_package()
