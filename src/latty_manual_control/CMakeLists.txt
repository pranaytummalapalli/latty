cmake_minimum_required(VERSION 3.8)
project(latty_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build executable
add_executable(manual_control
  src/manual_ctrl.cpp
  src/main.cpp
)

ament_target_dependencies(manual_control
  rclcpp
  geometry_msgs
  std_msgs
)

# Include headers
target_include_directories(manual_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  manual_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
