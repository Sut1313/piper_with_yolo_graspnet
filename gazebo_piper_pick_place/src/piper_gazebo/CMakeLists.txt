cmake_minimum_required(VERSION 3.8)
project(piper_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

# ── 自定义服务接口 ────────────────────────────────────────────
rosidl_generate_interfaces(piper_gazebo
  "srv/AttachDetach.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  piper_gazebo rosidl_typesupport_cpp
)

# ── Gazebo WorldPlugin: link_attacher ────────────────────────
add_library(piper_link_attacher SHARED src/link_attacher_plugin.cpp)
target_include_directories(piper_link_attacher PRIVATE
  ${gazebo_ros_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
)
target_link_libraries(piper_link_attacher
  ${gazebo_ros_LIBRARIES}
  ${gazebo_dev_LIBRARIES}
  ${cpp_typesupport_target}
)
ament_target_dependencies(piper_link_attacher
  gazebo_dev
  gazebo_ros
  rclcpp
)

# ── pick-and-place 可执行文件 ─────────────────────────────────
add_executable(pick_and_place
  src/pick_and_place.cpp
  src/grasp_frame_transform.cpp
  src/graspnet_6d_grasp_planner.cpp
  src/graspnet_perception_loader.cpp
  src/piper_analytic_ik.cpp
  src/pick_state_writer.cpp
  src/position_grasp_planner.cpp
)
target_include_directories(pick_and_place PRIVATE include)
ament_target_dependencies(pick_and_place
  control_msgs
  gazebo_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  trajectory_msgs
  urdf
)
target_link_libraries(pick_and_place ${cpp_typesupport_target})

# ── 安装 ─────────────────────────────────────────────────────
install(
  TARGETS piper_link_attacher pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本（相机桥接等）
install(
  PROGRAMS
    scripts/camera_bridge_node.py
    scripts/piper_ssik_ik.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config worlds srv
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
