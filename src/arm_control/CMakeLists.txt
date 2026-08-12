cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(arm_control src/moveit.cpp)

ament_target_dependencies(
  arm_control
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)
install(TARGETS arm_control 
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
