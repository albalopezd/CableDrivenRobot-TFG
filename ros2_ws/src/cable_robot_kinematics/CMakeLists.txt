cmake_minimum_required(VERSION 3.8)
project(cable_robot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(continuum_kinematics src/continuum_kinematics.cpp)
target_include_directories(continuum_kinematics PUBLIC include)
ament_target_dependencies(continuum_kinematics)


add_executable(joint_state_combiner_cpp src/joint_state_combiner.cpp)
add_executable(cable_bend_bridge_cpp src/cable_bend_bridge.cpp)
add_executable(ik_bridge_cpp src/ik_bridge.cpp)
add_executable(fk_bridge_cpp src/fk_bridge.cpp)

ament_target_dependencies(joint_state_combiner_cpp
  rclcpp
  sensor_msgs
)
ament_target_dependencies(cable_bend_bridge_cpp
  rclcpp
  sensor_msgs
)
ament_target_dependencies(ik_bridge_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
)
ament_target_dependencies(fk_bridge_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(ik_bridge_cpp continuum_kinematics)
target_link_libraries(fk_bridge_cpp continuum_kinematics)


install(TARGETS
  joint_state_combiner_cpp
  cable_bend_bridge_cpp
  fk_bridge_cpp
  ik_bridge_cpp
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
