cmake_minimum_required(VERSION 3.8)
project(dual_nero_camera_relay)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(color_jpeg_relay src/color_jpeg_relay.cpp)
target_include_directories(color_jpeg_relay PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(color_jpeg_relay ${OpenCV_LIBS})
ament_target_dependencies(color_jpeg_relay rclcpp sensor_msgs)

install(
  TARGETS color_jpeg_relay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
