cmake_minimum_required(VERSION 3.16)
project(robot_body_filter LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(dependencies
  rclcpp
  filters
  pluginlib
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  urdf
  geometric_shapes
)
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/OrientedBoundingBox.msg
  msg/OrientedBoundingBoxStamped.msg
  msg/Sphere.msg
  msg/SphereStamped.msg
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(${PROJECT_NAME}_core SHARED src/ros2/robot_body_filter.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

add_executable(${PROJECT_NAME}_node src/ros2/robot_body_filter_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs filters)

pluginlib_export_plugin_description_file(filters laser_filters.xml)
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
# The ROS 1 API is retained in the source tree, but is not part of the ROS 2 API.
install(DIRECTORY include/${PROJECT_NAME}/ros2 DESTINATION include/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} rosidl_default_runtime)
ament_package()
