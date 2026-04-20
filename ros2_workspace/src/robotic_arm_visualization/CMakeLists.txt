cmake_minimum_required(VERSION 3.8)
project(robotic_arm_visualization)

# Build settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install RViz configuration files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
