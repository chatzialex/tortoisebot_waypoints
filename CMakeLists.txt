cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(tortoisebot_action_server_lib src/tortoisebot_action_server.cpp)
target_include_directories(tortoisebot_action_server_lib 
	PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tortoisebot_action_server_lib
  rclcpp
  rclcpp_action
  tf2
  nav_msgs
  geometry_msgs)
rosidl_target_interfaces(tortoisebot_action_server_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_targets(tortoisebot_action_server_lib HAS_LIBRARY_TARGET)

add_executable(tortoisebot_action_server_standalone src/tortoisebot_action_server_standalone.cpp)
target_link_libraries(tortoisebot_action_server_standalone tortoisebot_action_server_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  tortoisebot_action_server_lib
  tortoisebot_action_server_standalone
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS tortoisebot_action_server_lib
  EXPORT tortoisebot_action_server_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
