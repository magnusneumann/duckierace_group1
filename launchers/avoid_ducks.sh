#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun avoid_ducks detect_ducks_node.py &

sleep 5

rosrun avoid_ducks duck_avoidance_node.py 

wait
