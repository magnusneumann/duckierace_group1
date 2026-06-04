#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun follow_lane detect_lane_node.py &
rosrun follow_lane detect_sign_node.py &
#rosrun follow_lane configuration_node.py &
rosrun follow_lane switch_control_node.py &
rosrun follow_lane cross_intersection_node.py &
#rosrun follow_lane detect_ducks_node.py &
rosrun follow_lane dashboard_node.py &
#rosrun follow_lane mapping_node.py &
sleep 5

rosrun follow_lane control_lane_node.py

wait
