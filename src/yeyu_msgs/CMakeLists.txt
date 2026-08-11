cmake_minimum_required(VERSION 3.8)
project(yeyu_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemState.msg"
  "msg/DrivingStatus.msg"
  "msg/FunctionStatus.msg"
  "msg/AudioCommand.msg"
  "msg/IRSensor.msg"
  "srv/StartRetry.srv" 
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()