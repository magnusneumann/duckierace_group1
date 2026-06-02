#!/bin/bash
# Pfad: ~/duckierace_groupe1/netzwerk.sh

# 1. Namen aus dem Argument nehmen (falls leer, nimm 'tick')
BOT_NAME=$1
if [ -z "$BOT_NAME" ]; then
    BOT_NAME="gustav"
fi

# 2. ROS-Umgebung laden
source /opt/ros/noetic/setup.bash

# 3. Netzwerk-Variablen dynamisch setzen
export VEHICLE_NAME=$BOT_NAME
export ROS_MASTER_URI=http://$BOT_NAME.local:11311
export ROS_IP=192.168.90.111

# 4. Workspace laden
cd ~/duckierace_groupe1
source devel/setup.bash

echo "✅ Netzwerk bereit: $VEHICLE_NAME | Master: $ROS_MASTER_URI"