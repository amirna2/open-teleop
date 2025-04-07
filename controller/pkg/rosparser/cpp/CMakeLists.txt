cmake_minimum_required(VERSION 3.8)
project(rosparser)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
# find_package(rcl REQUIRED) # No longer directly needed?
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rcutils REQUIRED) # Added for shared_library utils
find_package(ament_index_cpp REQUIRED) # Added for get_package_prefix
find_package(rosbag2_cpp REQUIRED) # Added for typesupport helpers

# We're using our own copy of nlohmann/json (header-only library)
# No need to find_package anymore

# Create shared library
add_library(ros_parser SHARED
  ros_parser.cpp
)

# Use ament_target_dependencies for linking ROS packages
ament_target_dependencies(ros_parser PUBLIC
  rclcpp
  rcpputils
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_c
  # rcl # No longer directly needed?
  rosidl_typesupport_fastrtps_cpp
  rcutils # Added
  ament_index_cpp # Added
  rosbag2_cpp # Added
)

# Explicitly add all required include directories
target_include_directories(ros_parser PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
    ${rclcpp_INCLUDE_DIRS}
    ${rcpputils_INCLUDE_DIRS}
    ${rosidl_typesupport_introspection_cpp_INCLUDE_DIRS}
    ${rosidl_runtime_c_INCLUDE_DIRS}
    # ${rcl_INCLUDE_DIRS} # Include from find_package(rcl)
    # Add potentially missing direct paths if the above fails
    /opt/ros/jazzy/include/rcl 
    /opt/ros/jazzy/include/rosidl_runtime_c
    /opt/ros/jazzy/include/rosidl_typesupport_introspection_cpp
    /opt/ros/jazzy/include/rcpputils
    /opt/ros/jazzy/include/rclcpp
    /opt/ros/jazzy/include/rosidl_typesupport_fastrtps_cpp # Add include for RMW
)

# Set library output name and install destination
set_target_properties(ros_parser PROPERTIES
  OUTPUT_NAME "ros_parser"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# Install the library and header
install(
  TARGETS ros_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES ros_parser.h
  DESTINATION include/rosparser
)

# Export package information
ament_export_include_directories(include)
ament_export_libraries(ros_parser)
ament_export_dependencies(
  rclcpp
  rcpputils
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_c
  # rcl # No longer directly needed?
  rosidl_typesupport_fastrtps_cpp
  rcutils # Added
  ament_index_cpp # Added
  rosbag2_cpp # Added
)

ament_package() 