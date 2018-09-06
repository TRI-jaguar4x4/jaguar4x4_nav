cmake_minimum_required(VERSION 3.5)
project(jaguar4x4_nav)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jaguar4x4_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(jaguar4x4_nav_node src/jaguar4x4_nav.cpp)

ament_target_dependencies(jaguar4x4_nav_node
  geometry_msgs
  jaguar4x4_nav_msgs
  nav_msgs
  rclcpp
  std_srvs
  tf2
  tf2_ros
)

target_include_directories(jaguar4x4_nav_node
  PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  jaguar4x4_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
