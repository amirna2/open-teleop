cmake_minimum_required(VERSION 3.8)
project(open_teleop_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/EncodedFrame.msg"
  "msg/StreamStatus.msg"
)

set(SRV_FILES
  "srv/ManageStream.srv"
  "srv/GetStatus.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs # Add other dependencies here if needed, like builtin_interfaces
)

ament_package() 