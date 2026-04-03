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
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(urdf REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePoint.srv"
  "srv/PlayTrajectory.srv"
  "srv/GetIkSolutions.srv"
  "srv/GetBestIkSolution.srv"
  "srv/MoveToPoint.srv"
  "srv/GetIkSolutions6Dof.srv"
  "srv/GetBestIkSolution6Dof.srv"
  "srv/MoveToPose.srv"
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
  "geometry_msgs"
  "tf2_ros"
)
target_link_libraries(logger_node Eigen3::Eigen)

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
)
target_compile_definitions(teach_point_server_node PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

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
)
target_compile_definitions(play_trajectory_server_node PRIVATE PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

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

add_executable(ik_solver_node src/ik_solver_node.cpp)
target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ik_solver_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  ik_solver_node
  "rclcpp"
  "sensor_msgs"
  "urdf"
)
target_link_libraries(ik_solver_node Eigen3::Eigen)

add_executable(motion_manager_node src/motion_manager_node.cpp)
target_include_directories(motion_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(motion_manager_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  motion_manager_node
  "rclcpp"
  "sensor_msgs"
  "rrm_msgs"
)

add_executable(ik_solver_6dof_node src/ik_solver_6dof_node.cpp)
target_include_directories(ik_solver_6dof_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ik_solver_6dof_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  ik_solver_6dof_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "urdf"
)
target_link_libraries(ik_solver_6dof_node Eigen3::Eigen)

add_executable(motion_manager_6dof_node src/motion_manager_6dof_node.cpp)
target_include_directories(motion_manager_6dof_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(motion_manager_6dof_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  motion_manager_6dof_node
  "rclcpp"
  "sensor_msgs"
  "rrm_msgs"
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

rosidl_target_interfaces(ik_solver_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(motion_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(ik_solver_6dof_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(motion_manager_6dof_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  teleop_node
  logger_node
  teach_point_server_node
  teach_point_client_node
  play_trajectory_server_node
  play_trajectory_client_node
  control_node
  ik_solver_node
  ik_solver_6dof_node
  motion_manager_node
  motion_manager_6dof_node
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