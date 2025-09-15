cmake_minimum_required(VERSION 3.10)
project(rbq_description)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Python 스크립트 설치
install(
  PROGRAMS 
    scripts/robot_odometry_to_tf.py
    scripts/slam_odometry_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
