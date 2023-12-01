cmake_minimum_required(VERSION 3.8)
project(ur5_kdr_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

#add_executable(joint_pos_ctrl src/joint_pos_ctrl.cpp)
#ament_target_dependencies(joint_pos_ctrl rclcpp trajectory_msgs)

# add_executable(joint_vel_ctrl src/joint_vel_ctrl.cpp)
# ament_target_dependencies(joint_vel_ctrl rclcpp std_msgs controller_manager_msgs std_srvs)

add_executable(frame_publisher src/frame_publisher.cpp)
ament_target_dependencies(frame_publisher rclcpp geometry_msgs tf2_ros)

install(TARGETS
  #joint_pos_ctrl
  # joint_vel_ctrl
  frame_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
