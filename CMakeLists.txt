cmake_minimum_required(VERSION 3.8)
project(f610_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorTarget.msg"
  "msg/MotorState.msg"
)
add_executable(controller src/F610_controller_node.cpp)
target_compile_features(controller PRIVATE cxx_std_17)
ament_target_dependencies(controller rclcpp can_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(controller "${cpp_typesupport_target}")
install(TARGETS controller DESTINATION lib/${PROJECT_NAME})
add_executable(motor_target_publisher src/motor_target_publisher.cpp)
target_compile_features(motor_target_publisher PRIVATE cxx_std_17)
ament_target_dependencies(motor_target_publisher rclcpp)
target_link_libraries(motor_target_publisher "${cpp_typesupport_target}")
install(TARGETS motor_target_publisher DESTINATION lib/${PROJECT_NAME})

# Expose the requested ros2 run alias without renaming the message package.
ament_index_register_resource("packages" PACKAGE_NAME "F610Controller")
install(PROGRAMS "$<TARGET_FILE:controller>"
  DESTINATION lib/F610Controller
  RENAME controller)

ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(controller_integration
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_controller.py"
      "$<TARGET_FILE:controller>"
    TIMEOUT 90
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs"
      "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
