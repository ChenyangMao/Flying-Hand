cmake_minimum_required(VERSION 3.8)
project(core_pose_controller)

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
find_package(Eigen3 REQUIRED)
find_package(base_main_class REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

# PoseController + GeoFence library
add_library(pose_controller_lib SHARED
  src/pose_controller.cpp
)
target_include_directories(pose_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pose_controller_lib
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs base_main_class
)

# PoseControlNode executable
add_executable(pose_controller
  src/pose_controller_node.cpp
  src/pose_controller_main.cpp
)
target_include_directories(pose_controller PRIVATE include ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(pose_controller
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs mav_msgs base_main_class
)
target_link_libraries(pose_controller pose_controller_lib)

install(TARGETS pose_controller_lib pose_controller
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
ament_export_libraries(pose_controller_lib)
ament_export_dependencies(
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs mav_msgs base_main_class
)

ament_package()
