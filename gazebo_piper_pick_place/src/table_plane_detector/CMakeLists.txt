cmake_minimum_required(VERSION 3.8)
project(table_plane_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters sample_consensus segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TablePlane.msg"
  DEPENDENCIES std_msgs
)

add_library(table_plane_core
  src/plane_estimator.cpp
  src/temporal_filter.cpp
)
target_compile_features(table_plane_core PUBLIC cxx_std_17)
target_include_directories(table_plane_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(table_plane_core Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(table_plane_core PUBLIC ${PCL_DEFINITIONS})

add_executable(table_plane_detector_node src/table_plane_detector_node.cpp)
target_compile_features(table_plane_detector_node PUBLIC cxx_std_17)
target_include_directories(table_plane_detector_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(table_plane_detector_node table_plane_core Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(table_plane_detector_node
  diagnostic_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(table_plane_detector_node "${cpp_typesupport_target}")

add_executable(depth_image_to_cloud_node src/depth_image_to_cloud_node.cpp)
target_compile_features(depth_image_to_cloud_node PUBLIC cxx_std_17)
target_include_directories(depth_image_to_cloud_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(depth_image_to_cloud_node ${PCL_LIBRARIES})
ament_target_dependencies(depth_image_to_cloud_node
  pcl_conversions
  rclcpp
  sensor_msgs
)

install(TARGETS table_plane_core table_plane_detector_node depth_image_to_cloud_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz worlds DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE CHANGELOG.rst DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/evaluate_table_plane.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_plane_math test/test_plane_math.cpp)
  target_link_libraries(test_plane_math table_plane_core Eigen3::Eigen ${PCL_LIBRARIES})

  ament_add_gtest(test_synthetic_cloud test/test_synthetic_cloud.cpp)
  target_link_libraries(test_synthetic_cloud table_plane_core Eigen3::Eigen ${PCL_LIBRARIES})

  ament_add_gtest(test_temporal_filter test/test_temporal_filter.cpp)
  target_link_libraries(test_temporal_filter table_plane_core Eigen3::Eigen ${PCL_LIBRARIES})
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
