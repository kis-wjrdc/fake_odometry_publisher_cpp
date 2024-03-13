cmake_minimum_required(VERSION 3.5)
project(fake_odometry_publisher_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(fake_odometry_publisher_cpp src/fake_odometry_publisher.cpp)

# Specify include directories
target_include_directories(fake_odometry_publisher_cpp
  PRIVATE
  include
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Specify libraries to link against
target_link_libraries(fake_odometry_publisher_cpp
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

# Install executable
install(TARGETS
  fake_odometry_publisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}
#)

# Install package.xml
ament_package()

