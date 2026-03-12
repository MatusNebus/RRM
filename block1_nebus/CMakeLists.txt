cmake_minimum_required(VERSION 3.8)
project(block1_nebus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rrm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePoint.srv"
  "srv/PlayTrajectory.srv"
)

add_executable(teleop_node src/teleop_node.cpp)
target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teleop_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  teleop_node
  "rclcpp"
  "rrm_msgs"
  "sensor_msgs"
)

add_executable(logger_node src/logger_node.cpp)
target_include_directories(logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(logger_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  logger_node
  "rclcpp"
  "sensor_msgs"
)

add_executable(teach_point_server_node src/teach_point_server_node.cpp)
target_include_directories(teach_point_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teach_point_server_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  teach_point_server_node
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "ament_index_cpp"
)

add_executable(teach_point_client_node src/teach_point_client_node.cpp)
target_include_directories(teach_point_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teach_point_client_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  teach_point_client_node
  "rclcpp"
)

add_executable(play_trajectory_server_node src/play_trajectory_server_node.cpp)
target_include_directories(play_trajectory_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(play_trajectory_server_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  play_trajectory_server_node
  "rclcpp"
  "rrm_msgs"
  "sensor_msgs"
  "ament_index_cpp"
)

add_executable(play_trajectory_client_node src/play_trajectory_client_node.cpp)
target_include_directories(play_trajectory_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(play_trajectory_client_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  play_trajectory_client_node
  "rclcpp"
)

add_executable(control_node src/control_node.cpp)
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(control_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  control_node
  "rclcpp"
  "rrm_msgs"
  "sensor_msgs"
  "std_srvs"
)

rosidl_target_interfaces(teach_point_server_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(teach_point_client_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(play_trajectory_server_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(play_trajectory_client_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(control_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  teleop_node
  logger_node
  teach_point_server_node
  teach_point_client_node
  play_trajectory_server_node
  play_trajectory_client_node
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()