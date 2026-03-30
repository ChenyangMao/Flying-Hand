cmake_minimum_required(VERSION 3.8)
project(core_visual_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/color_circle_detection.py
  src/Hough_circle_detection.py
  src/IBVS.py
  src/tracking_point_sender.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
