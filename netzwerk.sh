#!/bin/bash
source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://tick.local:11311

export ROS_IP=192.168.90.111

export VEHICLE_NAME=tick

cd ~/DuckieRace_2026-main/

source devel/setup.bash

echo "ROS-Netzwerk geladen! Master: $ROS_MASTER_URI | VM-IP $ROS_IP"
