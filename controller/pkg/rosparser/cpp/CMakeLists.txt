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

# We're using our own copy of nlohmann/json (header-only library)
# No need to find_package anymore

# Create shared library
add_library(ros_parser SHARED
  ros_parser.cpp
)

# Set library output name and install destination
set_target_properties(ros_parser PROPERTIES
  OUTPUT_NAME "ros_parser"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# Include headers
target_include_directories(ros_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>  # Add third_party directory
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rcpputils_INCLUDE_DIRS}
  ${rosidl_typesupport_introspection_cpp_INCLUDE_DIRS}
  ${rosidl_runtime_c_INCLUDE_DIRS}
)

# Link against required libraries
target_link_libraries(ros_parser
  ${rclcpp_LIBRARIES}
  ${rcpputils_LIBRARIES}
  ${rosidl_typesupport_introspection_cpp_LIBRARIES}
  ${rosidl_runtime_c_LIBRARIES}
  # Remove nlohmann_json::nlohmann_json as we're using our bundled version
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
)

ament_package() 