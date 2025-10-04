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
find_package(std_msgs REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)


# Build executable
add_executable(move_latty 
  src/move_latty.cpp
  src/main.cpp
  )

add_executable(latty_odom src/odom.cpp)

add_executable(imu_euler src/experimentation/imu_euler_publisher.cpp)

add_executable(imu_expm src/experimentation/imu_expm_publisher.cpp)

ament_target_dependencies(move_latty 
  rclcpp
  gazebo_ros2_control
  std_msgs
  gazebo_msgs 
  geometry_msgs
  )

ament_target_dependencies(latty_odom 
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

ament_target_dependencies(imu_euler
  rclcpp
  sensor_msgs
  nav_msgs
  Eigen3
  )

ament_target_dependencies(imu_expm
  rclcpp
  sensor_msgs
  nav_msgs
  Eigen3
  )

# Include headers
target_include_directories(move_latty PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_include_directories(latty_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_include_directories(imu_euler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_include_directories(imu_expm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

# Install targets
install(TARGETS
  move_latty
  latty_odom
  imu_euler
  imu_expm
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

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
