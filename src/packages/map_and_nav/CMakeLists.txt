
cmake_minimum_required(VERSION 3.0.2)
project(map_and_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package()

install(PROGRAMS
  src/cross_intersection_node.py
  src/detect_and_control_lane_node.py
  src/detect_tags_node.py
  src/mapping_and_relocalization_node.py
  src/mapping_topological_node.py
  src/navigator_node.py
  src/switch_control_node.py
  src/topological_graph.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
