cmake_minimum_required(VERSION 3.8)
project(ur5e_surface_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometric_shapes REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(trace_cube src/trace_cube.cpp)

set(dependencies
  rclcpp
  std_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  geometric_shapes
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2
)
ament_target_dependencies(hello_moveit ${dependencies})
ament_target_dependencies(trace_cube ${dependencies})


install(TARGETS 
  hello_moveit
  trace_cube
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config meshes worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()