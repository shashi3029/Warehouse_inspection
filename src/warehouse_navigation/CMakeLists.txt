cmake_minimum_required(VERSION 3.8)
project(warehouse_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(warehouse_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  warehouse_msgs
)

include_directories(include)

# Navigation Controller executable
add_executable(navigation_controller
  src/navigation_controller.cpp
)
ament_target_dependencies(navigation_controller ${DEPENDENCIES})
target_include_directories(navigation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Formation Controller executable
add_executable(formation_controller
  src/formation_controller.cpp
)
ament_target_dependencies(formation_controller ${DEPENDENCIES})
target_include_directories(formation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  navigation_controller
  formation_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
