cmake_minimum_required(VERSION 3.8)
project(piper_lightweight_sim)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(lightweight_grasp_executor src/lightweight_grasp_executor.cpp)
target_compile_features(lightweight_grasp_executor PUBLIC cxx_std_17)
target_link_libraries(lightweight_grasp_executor Eigen3::Eigen)
ament_target_dependencies(lightweight_grasp_executor
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
)

install(TARGETS lightweight_grasp_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
