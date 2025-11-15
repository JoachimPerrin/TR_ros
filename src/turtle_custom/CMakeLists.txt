cmake_minimum_required(VERSION 3.8)
project(turtle_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtle_action_interfaces REQUIRED)

# Build the pkg as a component (.so lib)
add_library(custom_component SHARED
  src/custom_sub.cpp
)

# Add my headers
target_include_directories(custom_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# set visibility (see visibility_control.h)
target_compile_definitions(custom_component PRIVATE 
  "TURTLE_CUSTOM_CPP_BUILDING_DLL"
)

# Add dependencies
ament_target_dependencies(custom_component
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "turtlesim"
  "geometry_msgs"
  "turtle_action_interfaces"
)

# Declare node as component
rclcpp_components_register_node(custom_component 
  PLUGIN "custom::TurtlePlugin" 
  EXECUTABLE custom_node
)

# Install target
install(TARGETS custom_component
  EXPORT export_custom_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()