cmake_minimum_required(VERSION 3.0.2)
project(ship_navigation_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  gazebo_ros
  robot_state_publisher
  controller_manager
  diff_drive_controller
  joint_state_controller
  robot_localization
  costmap_2d
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/global_planner_node.py
                scripts/dwa_planner_node.py
                scripts/pure_pursuit_node.py
                scripts/experiment_runner.py
                scripts/metrics_collector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)