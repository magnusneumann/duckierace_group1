# DuckieRace
Duckie Challange 2026 - Fach Robogistics

Nach dem Klonen: Erweiterung dev Containers installieren, STRG SHIFT P um reopen in container auszuführen, Pakete installieren: tmux, pupil-apriltags, numpy updaten

Auf Bot einloggen im Terminal (Container läuft && im DuckieNetz):
./docker_login.sh dann Botnamen eingeben

Dann:
Bot auf Parkour fahren lassen:
launchers/follow_lane.sh
launchers/avoid_ducks.sh

Im Terminal scrollen:
STRG + B, :, set -g mouse on

Terminal wechseln:
STRG + B, Pfeiltaste

Docker darf GUI's öffnen:
xhost +local:root im gobalen Terminal

ChArUco kalibrierung während der charuco_calibration_node läuft:
rosservice call /VEHICLENAME/calibration/save_homography



