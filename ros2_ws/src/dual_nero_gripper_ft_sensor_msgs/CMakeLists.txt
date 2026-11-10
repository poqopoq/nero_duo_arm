cmake_minimum_required(VERSION 3.5)

project(dual_nero_gripper_ft_sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SixAxisForce.msg"
  "srv/SetBaudRate.srv"
  "srv/SetSampleRate.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
