cmake_minimum_required(VERSION 3.8)
project(core_wrench_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(base_main_class REQUIRED)
find_package(core_pid_controller REQUIRED)
find_package(core_pose_controller REQUIRED)

# WrenchController library (uses core_pid_controller)
add_library(wrench_controller_lib SHARED
  src/wrench_controller.cpp
)
target_include_directories(wrench_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wrench_controller_lib
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(wrench_controller_lib core_pid_controller)

# WrenchControlNode executable
add_executable(wrench_controller
  src/wrench_controller_node.cpp
  src/wrench_controller_main.cpp
)
target_include_directories(wrench_controller PRIVATE include)
ament_target_dependencies(wrench_controller
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs mav_msgs
)
target_link_libraries(wrench_controller
  wrench_controller_lib
  base_main_class
  core_pid_controller
  pose_controller_lib
)

install(TARGETS wrench_controller_lib wrench_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_export_include_directories(include)
ament_export_libraries(wrench_controller_lib)
ament_export_dependencies(
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs mav_msgs
  base_main_class core_pid_controller core_pose_controller
)

ament_package()
