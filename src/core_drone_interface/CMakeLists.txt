cmake_minimum_required(VERSION 3.8)
project(core_drone_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(base_main_class REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DroneCommand.srv"
  DEPENDENCIES std_msgs
)

add_library(drone_interface_lib SHARED
  src/drone_interface.cpp
)
target_include_directories(drone_interface_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(drone_interface_lib
  rclcpp geometry_msgs std_msgs mav_msgs
)

add_executable(drone_interface_node
  src/drone_interface_node.cpp
)
target_include_directories(drone_interface_node PRIVATE include)
ament_target_dependencies(drone_interface_node
  rclcpp geometry_msgs std_msgs mav_msgs pluginlib base_main_class
)
target_link_libraries(drone_interface_node drone_interface_lib)
ament_target_dependencies(drone_interface_node ${PROJECT_NAME})

install(TARGETS drone_interface_node drone_interface_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
  FILES_MATCHING PATTERN "*.srv"
)

ament_export_include_directories(include)
ament_export_libraries(drone_interface_lib)
ament_export_dependencies(
  rclcpp geometry_msgs std_msgs mav_msgs pluginlib
)
ament_package()
