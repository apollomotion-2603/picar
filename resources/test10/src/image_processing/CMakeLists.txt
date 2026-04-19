cmake_minimum_required(VERSION 3.5)
project(image_processing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(image_processing_lib
  src/image_functions.cpp
  src/image_fitting.cpp
)

target_link_libraries(image_processing_lib
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(image_processing_lib rclcpp sensor_msgs OpenCV Eigen3)

add_executable(image_processor src/image_processor.cpp src/image_fitting.cpp src/image_functions.cpp)
ament_target_dependencies(image_processor rclcpp sensor_msgs std_msgs OpenCV Eigen3)
target_link_libraries(image_processor ${OpenCV_LIBRARIES})

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs std_msgs OpenCV cv_bridge)
target_link_libraries(camera_publisher ${OpenCV_LIBRARIES})

install(TARGETS image_processor camera_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS image_processing_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(image_processing_lib)
ament_export_dependencies(rclcpp sensor_msgs OpenCV cv_bridge Eigen3)

ament_package()
