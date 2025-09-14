cmake_minimum_required(VERSION 3.8)
project(latty_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros2_control REQUIRED)


# Build executable
add_executable(move_latty 
  src/move_latty.cpp
  src/main.cpp
  )

ament_target_dependencies(move_latty 
  rclcpp
  gazebo_ros2_control
  gazebo_msgs 
  geometry_msgs
  )

# Include headers
target_include_directories(move_latty PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

# Install targets
install(TARGETS
  move_latty
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
  )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
