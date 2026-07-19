# DuckieRace
## Duckie Challange 2026 - Fach Robogistics

### Nach dem Klonen: 
Erweiterung dev Containers installieren, STRG SHIFT P um reopen in container auszuführen,
das dockerfile wurde erweitert, alle Pakete müssten automatisch installiert werden. 

zuvor musst nachträglich installiert werden:
apt-get update && apt-get install -y tmux python3-pip && python3 -m pip install --upgrade pip && python3 -m pip install --upgrade numpy ultralytics shapely pupil-apriltags && python3 -m pip install onnx onnxruntime

## Auf Bot einloggen im Terminal (Container läuft && im DuckieNetz):
./docker_login.sh dann Botnamen eingeben

### In diesem Terminal scrollen:
STRG + B, :, set -g mouse on



## Bot auf Parkour fahren lassen:
launchers/follow_lane.sh

## Bot auf Wendehammer mit Enten fahren lassen:
launchers/avoid_ducks.sh

## Bot den Parkour mappen lassen und als Service auf Kanten gemappte Punkte abfahren:
launchers/mapping.sh
### Mapping Starten: (löscht den aktuellen Graphen und fängt an aufzuzeichnen)
rosservice call /duckie_bot_NAME/mapping/start
### Karte Speichern (Export):
rosservice call /duckie_bot_NAME/mapping/export





### ggf interessant:

Terminal wechseln:
STRG + B, Pfeiltaste

Docker darf GUI's öffnen:
xhost +local:root im gobalen Terminal

ChArUco kalibrierung während der charuco_calibration_node läuft:
rosservice call /VEHICLENAME/calibration/save_homography

wenn ein Knoten rumheult, dass er kein Display findet, obwohl es ein Display gibt
export DISPLAY=:0

## Challenge 4: Topologisches Mapping und Gate-Navigation

Konfiguration:
`src/packages/map_and_nav/config/graph.json`

Starten:
`bash launchers/mapping.sh` und `bash launchers/navigation.sh`

Topologisches Mapping starten:
`rosservice call /duckie_bot_NAME/mapping4/start`

Mapping mit Gate-Zuordnungen und Fahrzeiten speichern:
`rosservice call /duckie_bot_NAME/mapping4/export`

Optional: Automatischer Export und Navigationstart nach vollständigem Mapping
- `rosparam set /mapping_topological_node/auto_export_mapping true`
- `rosparam set /mapping_topological_node/auto_start_navigation true`
- `rosparam set /mapping_topological_node/navigation_start_delay 2.0`

Mapping fuer Navigation laden:
`rosservice call /duckie_bot_NAME/navigation/load_mapping`

Navigation mit der konfigurierten Gate-Reihenfolge starten:
`rosservice call /duckie_bot_NAME/navigation/start`

Gate-Reihenfolge zur Laufzeit setzen:
`rostopic pub -1 /duckie_bot_NAME/navigation/route std_msgs/String "data: '[5, 9, 6]'"`

Separate Launch Scripts:
- Mapping only: `bash launchers/mapping.sh`
- Navigation only: `bash launchers/navigation.sh`

