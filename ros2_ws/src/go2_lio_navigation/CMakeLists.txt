cmake_minimum_required(VERSION 3.8)
project(go2_lio_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lio_base_bridge src/lio_base_bridge.cpp)
ament_target_dependencies(lio_base_bridge
  geometry_msgs nav2_msgs nav_msgs rclcpp rclcpp_action std_msgs tf2_ros)
target_link_libraries(lio_base_bridge Eigen3::Eigen)

add_library(loop_registration STATIC src/loop_registration.cpp)
target_include_directories(loop_registration PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(loop_registration PUBLIC Eigen3::Eigen gtsam ${PCL_LIBRARIES})

add_executable(scan_context_loop_closure src/scan_context_loop_closure.cpp)
ament_target_dependencies(scan_context_loop_closure
  geometry_msgs message_filters nav_msgs pcl_conversions rclcpp sensor_msgs
  std_msgs tf2_ros)
target_include_directories(scan_context_loop_closure PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(scan_context_loop_closure
  loop_registration)

add_executable(traversability_mapper src/traversability_mapper.cpp)
ament_target_dependencies(traversability_mapper
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs tf2_ros)
target_include_directories(traversability_mapper PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(traversability_mapper Eigen3::Eigen ${PCL_LIBRARIES})

install(TARGETS
  lio_base_bridge
  scan_context_loop_closure
  traversability_mapper
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/environment_mapper.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/environment_terrain_display.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scan_safety_gate.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/environment_grid.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/turn_clearance.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/observation_window.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/raw_lidar_guard.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/unitree_near_guard.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/background_budget.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/reactive_safety.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/idle_stand_guard.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/idle_stand.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/obstacle_memory.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/navigation_fault_guard.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/obstacle_memory_model.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/dynamic_obstacles.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_loop_registration test/test_loop_registration.cpp)
  target_link_libraries(test_loop_registration loop_registration)
  ament_add_gtest(test_terrain_geometry test/test_terrain_geometry.cpp)
  target_include_directories(test_terrain_geometry PRIVATE include)
endif()

ament_package()
