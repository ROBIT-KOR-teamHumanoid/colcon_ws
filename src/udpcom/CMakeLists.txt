cmake_minimum_required(VERSION 3.8)
project(udpcom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
)

add_executable(udpcom_receiver_node src/udpcom_receiver_node.cpp src/udpcom_multicast_receiver.cpp)
ament_target_dependencies(udpcom_receiver_node rclcpp rclcpp_lifecycle Boost humanoid_interfaces std_msgs)

add_executable(udpcom_sender_node src/udpcom_sender_node.cpp src/udpcom_multicast_sender.cpp)
ament_target_dependencies(udpcom_sender_node rclcpp rclcpp_lifecycle Boost humanoid_interfaces std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  udpcom_receiver_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  udpcom_sender_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
