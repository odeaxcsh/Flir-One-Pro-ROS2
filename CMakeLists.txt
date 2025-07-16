cmake_minimum_required(VERSION 3.5)
project(flir_one_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find ament and ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# find libusb via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 REQUIRED IMPORTED_TARGET libusb-1.0)
# you can also do:
# pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

include_directories(
  include
  $<TARGET_PROPERTY:PkgConfig::LIBUSB1,INTERFACE_INCLUDE_DIRECTORIES>
)

# build your node
add_executable(flir_one_node
  src/flir_one_node.cpp
  src/driver_flir.cpp
)

# link to ROS 2 libraries
ament_target_dependencies(flir_one_node
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

# link to libusb
target_link_libraries(flir_one_node
  PkgConfig::LIBUSB1
)

# install
install(
  TARGETS flir_one_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament package macro
ament_package()
