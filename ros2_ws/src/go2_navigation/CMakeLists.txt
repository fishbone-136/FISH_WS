cmake_minimum_required(VERSION 3.8)
project(go2_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_rotation_shim_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
add_library(go2_gait_safety_critic SHARED src/gait_safety_critic.cpp src/collision_footprint_critic.cpp)
target_include_directories(go2_gait_safety_critic PRIVATE include)
ament_target_dependencies(go2_gait_safety_critic
  dwb_core dwb_critics nav2_costmap_2d nav2_util pluginlib std_msgs sensor_msgs)
pluginlib_export_plugin_description_file(dwb_core gait_safety_critic_plugin.xml)
install(TARGETS go2_gait_safety_critic LIBRARY DESTINATION lib)

add_library(go2_current_obstacle_layer SHARED src/current_obstacle_layer.cpp)
ament_target_dependencies(go2_current_obstacle_layer nav2_costmap_2d pluginlib)
pluginlib_export_plugin_description_file(nav2_costmap_2d current_obstacle_plugin.xml)
install(TARGETS go2_current_obstacle_layer LIBRARY DESTINATION lib)

add_library(go2_gait_trajectory_generator SHARED src/gait_trajectory_generator.cpp)
ament_target_dependencies(go2_gait_trajectory_generator dwb_core dwb_plugins nav2_util pluginlib)
pluginlib_export_plugin_description_file(dwb_core gait_trajectory_plugin.xml)
install(TARGETS go2_gait_trajectory_generator LIBRARY DESTINATION lib)

add_library(go2_exact_goal_smac SHARED src/exact_goal_smac.cpp)
ament_target_dependencies(go2_exact_goal_smac nav2_smac_planner pluginlib)
pluginlib_export_plugin_description_file(nav2_core exact_goal_planner_plugin.xml)
install(TARGETS go2_exact_goal_smac LIBRARY DESTINATION lib)

add_library(go2_latest_goal_bt_node SHARED src/latest_goal_bt_node.cpp)
ament_target_dependencies(go2_latest_goal_bt_node
  behaviortree_cpp_v3 geometry_msgs tf2_ros tf2_geometry_msgs rclcpp)
target_compile_definitions(go2_latest_goal_bt_node PRIVATE BT_PLUGIN_EXPORT)
install(TARGETS go2_latest_goal_bt_node LIBRARY DESTINATION lib)

set(CMAKE_CXX_STANDARD 17)
add_library(go2_stable_rotation_shim SHARED
  src/stable_rotation_shim.cpp src/stable_goal_checker.cpp)
target_include_directories(go2_stable_rotation_shim PRIVATE include)
ament_target_dependencies(go2_stable_rotation_shim
  nav2_rotation_shim_controller nav2_core rclcpp_lifecycle pluginlib tf2_geometry_msgs)
ament_target_dependencies(go2_stable_rotation_shim std_msgs sensor_msgs)
pluginlib_export_plugin_description_file(nav2_core stable_rotation_plugin.xml)
install(TARGETS go2_stable_rotation_shim LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_global_detour test/test_global_detour.cpp)
  ament_target_dependencies(test_global_detour nav2_core nav2_costmap_2d nav2_util pluginlib tf2_ros)
  add_dependencies(test_global_detour go2_exact_goal_smac)
  ament_add_gtest(test_motion_geometry test/test_motion_geometry.cpp)
  target_include_directories(test_motion_geometry PRIVATE include)
  ament_target_dependencies(test_motion_geometry nav2_costmap_2d)
  ament_add_gtest(test_gait_safety_critic test/test_gait_safety_critic.cpp)
  target_include_directories(test_gait_safety_critic PRIVATE include)
  ament_target_dependencies(test_gait_safety_critic dwb_core nav2_costmap_2d nav2_util pluginlib sensor_msgs)
  add_dependencies(test_gait_safety_critic go2_gait_safety_critic)
  ament_add_gtest(test_gait_trajectory_generator test/test_gait_trajectory_generator.cpp)
  ament_target_dependencies(test_gait_trajectory_generator dwb_core dwb_plugins nav2_util pluginlib)
  add_dependencies(test_gait_trajectory_generator go2_gait_trajectory_generator)
  ament_add_gtest(test_rotation_helpers test/test_rotation_helpers.cpp)
  target_include_directories(test_rotation_helpers PRIVATE include)
  ament_target_dependencies(test_rotation_helpers rclcpp tf2_ros tf2_geometry_msgs)
  ament_add_gtest(test_stable_goal_checker test/test_stable_goal_checker.cpp)
  ament_target_dependencies(test_stable_goal_checker nav2_core pluginlib rclcpp_lifecycle)
  add_dependencies(test_stable_goal_checker go2_stable_rotation_shim)
  ament_add_gtest(test_latest_goal_bt test/test_latest_goal_bt.cpp)
  ament_target_dependencies(test_latest_goal_bt
    behaviortree_cpp_v3 rclcpp tf2_ros tf2_geometry_msgs)
  target_compile_definitions(test_latest_goal_bt PRIVATE
    LATEST_GOAL_PLUGIN="$<TARGET_FILE:go2_latest_goal_bt_node>"
    OBSTACLE_WAIT_BT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  add_dependencies(test_latest_goal_bt go2_latest_goal_bt_node)
endif()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  go2_navigation/navigation_command_tcpbridge.py
  go2_navigation/multi_waypoint_nav.py
  go2_navigation/path_recorder.py
  go2_navigation/patrol_navigator.py
  go2_navigation/tcp_client_standalone.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
