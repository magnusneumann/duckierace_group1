#!/bin/bash

# 1. ROS-Umgebung laden (Befehle verfügbar machen)
source /opt/ros/noetic/setup.bash

# 2. Netzwerk-Variablen (Master auf dem Bot, IP deiner VM)
export ROS_MASTER_URI=http://trick.local:11311
export ROS_IP=192.168.90.111
export VEHICLE_NAME=trick

# 3. Workspace laden (Pfade zu deinen Paketen wie follow_lane)
source devel/setup.bash

# 4. Bestätigung ausgeben
echo "✅ Netzwerk bereit: Master=tick.local | IP=192.168.90.111"
