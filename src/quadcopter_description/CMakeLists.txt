cmake_minimum_required(VERSION 3.8)
project(quadcopter_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
# Include all launch files which are located in launch directory in the package root.
urdf
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
