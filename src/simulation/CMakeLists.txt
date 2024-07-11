cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch
  src
  src/worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(diff_drive_publisher src/diff_drive_publisher.cpp)
ament_target_dependencies(diff_drive_publisher
  rclcpp
  geometry_msgs
  )

install(TARGETS
  diff_drive_publisher
  DESTINATION lib${PROJECT_NAME}
  )

ament_package()