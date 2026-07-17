#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun slam_and_service detect_tags_node.py _publish_intersection_decisions:=false &

sleep 3

rosrun slam_and_service switch_control_node.py &
rosrun slam_and_service cross_intersection_node.py &
python3 src/packages/slam_and_service/src/mapping_topological_node.py &
rosrun slam_and_service navigator_node.py &
rosrun follow_lane dashboard_node.py &

sleep 4

rosrun slam_and_service detect_and_control_lane_node.py

wait
