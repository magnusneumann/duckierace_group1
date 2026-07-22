#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun map_and_nav detect_tags_node.py _publish_intersection_decisions:=false &

sleep 3

rosrun map_and_nav switch_control_node.py _start_in_standby:=true &
rosrun map_and_nav cross_intersection_node.py &
rosrun map_and_nav navigator_node.py &

rosrun map_and_nav debug_navigation_node.py &

sleep 4

rosrun map_and_nav detect_and_control_lane_node.py

wait
