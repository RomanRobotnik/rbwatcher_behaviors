cmake_minimum_required(VERSION 3.8)
project(rbwatcher_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ptz_tracker_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
  std_msgs
  std_srvs
  ament_index_cpp
  ptz_tracker_interfaces
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteBehaviorTree.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(rbwatcher_behavior_nodes SHARED
  src/actions/follow_waypoints_action.cpp
  src/actions/navigate_to_pose_action.cpp
  src/actions/trigger_service_action.cpp
  src/actions/ptz_tracker_action.cpp
  src/conditions/bool_topic_condition.cpp
)
ament_target_dependencies(rbwatcher_behavior_nodes ${dependencies})

add_executable(behavior_tree_action_server
  src/behavior_tree_action_server.cpp
  src/main.cpp
)
ament_target_dependencies(behavior_tree_action_server ${dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rbwatcher_behavior_nodes ${cpp_typesupport_target})
target_link_libraries(behavior_tree_action_server rbwatcher_behavior_nodes ${cpp_typesupport_target})

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  rbwatcher_behavior_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS behavior_tree_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(rbwatcher_behavior_nodes)
ament_export_dependencies(${dependencies})

ament_package()
