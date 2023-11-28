cmake_minimum_required(VERSION 3.8)
project(rover_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS
  rclcpp
  actionlib
  amcl
  control_msgs
  controller_manager
  gmapping
  move_base
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  tf2
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install configuration, meshes, urdf and launch files.
install(DIRECTORY config description include launch meshes params worlds
  DESTINATION share/${PROJECT_NAME}/
)

#ament_target_dependencies(mobile_robot_hardware_interface rclcpp)

#add_library(i2c_ros src/i2c_ros.cpp)

#add_executable(mobile_robot_hardware_interface src/robot_hardware_interface_node.cpp)

#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} rosidl_typesupport_cpp)

#target_link_libraries(mobile_robot_hardware_interface "${cpp_typesupport_target}")

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

ament_package()
