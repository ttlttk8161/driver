cmake_minimum_required(VERSION 3.0.2)
project(autonomous_driving)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Make Python scripts executable
catkin_install_python(PROGRAMS
  autonomous_driving_node.py
  lane_detector.py
  obstacle_detector.py
  path_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
