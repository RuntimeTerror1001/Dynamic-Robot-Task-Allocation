cmake_minimum_required(VERSION 3.8)
project(allocation_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(allocation_common REQUIRED) # Assuming allocation_common is another package
find_package(ignition-gazebo6 REQUIRED)  # Ignition Gazebo dependency
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Source files for the package
set(SOURCE_FILES
  src/world_gazebo.cpp
  src/task_gazebo.cpp
  src/robot_gazebo.cpp
  # Add other .cpp files in your package's `src` directory
)

# Create a library for this package
add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_FILES})

# Link dependencies to the library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  gazebo_msgs
  std_msgs
  allocation_common
  ignition-gazebo6
)

# Create an executable if needed
# Example:
# add_executable(${PROJECT_NAME}_node src/main.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Install the library
install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts (if applicable)
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Add testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
