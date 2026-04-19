cmake_minimum_required(VERSION 3.8)
project(gazebo_ackermann_steering_vehicle)

# Đảm bảo sử dụng C++17 cho std::clamp và các tính năng hiện đại
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include nodes/image_processing src)

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller rclcpp std_msgs sensor_msgs)

add_executable(keyboard_controller
  src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller rclcpp std_msgs)

add_executable(drive_model
  src/drive_model.cpp)
ament_target_dependencies(drive_model rclcpp std_msgs nav_msgs)


add_executable(odom_filter nodes/odom_filter/odom_filter_node.cpp)
ament_target_dependencies(odom_filter rclcpp nav_msgs)

# add_executable(nmpc_controller
#   nodes/nmpc/nmpc_controller.cpp)
# ament_target_dependencies(nmpc_controller rclcpp std_msgs)
# target_link_libraries(nmpc_controller Eigen3::Eigen)

install(
  DIRECTORY launch model config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  vehicle_controller
  joystick_controller
  keyboard_controller
  drive_model
  odom_filter
  # nmpc_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
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