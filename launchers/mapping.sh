#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun map_and_nav detect_tags_node.py &

sleep 3

rosrun map_and_nav switch_control_node.py &
rosrun map_and_nav cross_intersection_node.py &
python3 src/packages/map_and_nav/src/mapping_topological_node.py &
# rosrun follow_lane dashboard_node.py &
python3 src/packages/map_and_nav/src/debug_mapping_node.py &
sleep 4

rosrun map_and_nav detect_and_control_lane_node.py

wait
