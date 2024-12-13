cmake_minimum_required(VERSION 3.8)
project(allocation_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Add this for message generation

# Uncomment the following section if you need additional dependencies
# find_package(<dependency> REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/allocation_robot_info.msg"  
  "msg/allocation_task_info.msg"
  "msg/allocation2terminal_info.msg"
  "msg/gazebo_robot_info.msg"
  "msg/gazebo_task_info.msg"
  "msg/gazebo2world_info.msg"
  "msg/Observation.msg"
  "msg/Point2d.msg"
  "msg/Pose.msg"
  "msg/robot2gazebo_info.msg"
  "msg/Taskinfo.msg"
  "msg/Teammateinfo.msg"
  "msg/terminal2DQN_info.msg"
  "msg/terminal2gazebo_info.msg"
  "msg/terminal2robot_info.msg"
  "msg/Twist.msg"
  "srv/GetAction.srv"
  "srv/ReturnReward.srv"
)

# Export dependencies for messages
ament_export_dependencies(rosidl_default_runtime)

# Add testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
