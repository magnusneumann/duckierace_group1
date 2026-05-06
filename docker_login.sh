#!/bin/bash

# 1. Namen abfragen
echo -n "🤖 Welchen Duckiebot willst du steuern? (z.B. trick oder tick): "
read BOT_NAME

if [ -z "$BOT_NAME" ]; then
    BOT_NAME="trick"
fi

# 2. Pfade festlegen
BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 3. Das Setup-Kommando, das in jedem neuen Fenster laufen soll
# Wir nutzen 'tail -f /dev/null' am Ende nicht, sondern lassen die Shell offen
SETUP_CMD="source /opt/ros/noetic/setup.bash; source $BASE_DIR/devel/setup.bash; export VEHICLE_NAME=$BOT_NAME; export ROS_MASTER_URI=http://$BOT_NAME.local:11311; export ROS_IP=$(hostname -I | awk '{print $1}'); exec bash"

# 4. Tmux Logik
# Prüfen, ob wir bereits IN einem tmux sind, um Endlosschleifen zu vermeiden
if [ -z "$TMUX" ]; then
    # Alte Sitzung killen
    tmux kill-session -t duckie_docker 2>/dev/null
    
    # Neue Sitzung im Hintergrund starten (-d)
    tmux new-session -d -s duckie_docker "bash -c '$SETUP_CMD'"
    
    # Horizontaler Split (rechts/links)
    tmux split-window -h "bash -c '$SETUP_CMD'"
    
    # In die Sitzung springen
    tmux attach-session -t duckie_docker
else
    # Falls man das Skript innerhalb von tmux aufruft, nur Setup ausführen
    source /opt/ros/noetic/setup.bash
    source "$BASE_DIR/devel/setup.bash"
    export VEHICLE_NAME=$BOT_NAME
    export ROS_MASTER_URI=http://$BOT_NAME.local:11311
    export ROS_IP=$(hostname -I | awk '{print $1}')
    echo "✅ Umgebung in bestehender Sitzung aktualisiert."
fi