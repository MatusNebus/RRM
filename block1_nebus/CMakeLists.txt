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

install(TARGETS
  teleop_node
  logger_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()