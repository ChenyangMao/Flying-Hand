cmake_minimum_required(VERSION 3.8)
project(core_px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(core_drone_interface REQUIRED)
find_package(mavros_msgs REQUIRED)

add_library(px4_interface SHARED
  src/px4_interface.cpp
)
target_include_directories(px4_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core_drone_interface/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_interface
  rclcpp geometry_msgs mav_msgs pluginlib core_drone_interface mavros_msgs
)

pluginlib_export_plugin_description_file(core_drone_interface plugins.xml)

install(TARGETS px4_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(px4_interface)
ament_export_dependencies(
  rclcpp geometry_msgs mav_msgs pluginlib core_drone_interface mavros_msgs
)
ament_package()
