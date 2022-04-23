cmake_minimum_required(VERSION 3.5)
project(metacontrol_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# find ROS 1 packages
set(cmake_extras_files cmake/find_ros1_package.cmake)
include(cmake/find_ros1_package.cmake)

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package(
    CONFIG_EXTRAS ${cmake_extras_files}
  )
  return()
endif()

find_ros1_package(std_msgs REQUIRED)
find_ros1_package(lifecycle_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} src/metacontrol_bridge/MetaControlNode.cpp)
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp lifecycle_msgs 
  ros1_roscpp ros1_lifecycle_msgs
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node 
  rclcpp rclcpp_lifecycle lifecycle_msgs 
  ros1_roscpp ros1_lifecycle_msgs
)
target_link_libraries(demo_node ${PROJECT_NAME})
install(TARGETS demo_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
