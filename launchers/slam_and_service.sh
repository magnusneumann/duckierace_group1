#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun slam_and_service detect_tags_node.py &
rosrun slam_and_service switch_control_node.py &
rosrun slam_and_service cross_intersection_node.py &
rosrun slam_and_service mapping_and_relocalization_node.py &

sleep 3

rosrun slam_and_service detect_and_control_lane_node.py

wait
