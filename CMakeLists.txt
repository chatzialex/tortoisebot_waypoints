cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

option(ENABLE_STANDALONE_GTEST "Enable standalone GTest" OFF)

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
find_package(launch_testing_ament_cmake REQUIRED)

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
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_SOURCES
    test/main.cpp
    test/tortoisebot_action_client.hpp
    test/tortoisebot_action_client.cpp
    test/tortoisebot_action_server_test.cpp
	test/odom_listener.hpp
	test/odom_listener.cpp
  )
  set(TEST_DEPS
    tortoisebot_action_server_lib
  )
  set(TEST_TARGET_NAME
    ${PROJECT_NAME}_test
  )
  set(TEST_TIMEOUT 120)
  if(ENABLE_STANDALONE_GTEST)
    ament_add_gtest(${TEST_TARGET_NAME} ${TEST_SOURCES} TIMEOUT ${TEST_TIMEOUT})
  else()
    add_executable(${TEST_TARGET_NAME} ${TEST_SOURCES})
    target_link_libraries(${TEST_TARGET_NAME} gtest)
    add_launch_test(test/waypoints_launch_test.py TIMEOUT ${TEST_TIMEOUT})
  endif()

  target_link_libraries(${TEST_TARGET_NAME} ${TEST_DEPS})
  install(
    TARGETS ${TEST_TARGET_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
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
