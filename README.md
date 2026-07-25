# DuckieRace 🦆🏁
**Duckie Challenge 2026 – Fach Robogistics**

Willkommen im Haupt-Repository für die Duckietown-Challenges! Dieses Projekt umfasst autonomes Fahren, Kollisionsvermeidung, topologisches Mapping und zielgesteuerte Navigation.

---

##  Detaillierte Paket-Dokumentationen
Für ein tiefes technisches Verständnis der implementierten Module (Perzeption, Regelung, State Machines, YOLO & Homographie) besuche bitte die detaillierten READMEs in den jeweiligen Source-Verzeichnissen:

1. **[Challenge 1 & 2: Follow Lane & Intersections](src/packages/follow_lane/README.md)**
2. **[Challenge 3: Avoid Ducks (YOLO & Zonen)](src/packages/avoid_ducks/README.md)**
3. **[Challenge 4: Map & Navigation (Dijkstra)](src/packages/map_and_nav/README.md)**

---

##  Installation & Setup (Nach dem Klonen)

1. **Dev Container:** 
   Installiere die Erweiterung "Dev Containers" in VS Code. Drücke `STRG + SHIFT + P` und wähle **Reopen in Container**. Das Dockerfile wurde so erweitert, dass alle Standard-Pakete automatisch installiert werden sollten.

2. **Fehlende Abhängigkeiten:**
   Falls Pakete fehlen, führe folgenden Befehl im Container-Terminal aus:
   ```bash
   apt-get update && apt-get install -y tmux python3-pip
   python3 -m pip install --upgrade pip
   python3 -m pip install --upgrade numpy ultralytics shapely pupil-apriltags onnx onnxruntime
   ```

3. **Rechte vergeben:**
   Im globalen Terminal (Projekthauptverzeichnis) ausführen, um alle Python-Skripte ausführbar zu machen:
   ```bash
   chmod +x -R .
   ```

4. **Kompilieren:**
   Führe in deinem VS Code Terminal den Build aus:
   ```bash
   catkin_make
   ```

---

##  Auf dem Bot einloggen

Sobald der Container läuft und du dich im DuckieNetz befindest, verbinde dich mit dem Roboter:
```bash
./docker_login.sh
```
*(Anschließend den Namen deines Roboters eingeben)*

### Wichtige Terminal & Tmux Shortcuts:
- **Terminal wechseln:** `STRG + B`, dann eine `Pfeiltaste`.
- **Maus / Scrollen aktivieren:** `STRG + B`, dann `:` drücken und `set -g mouse on` eingeben.

---

##  Die Challenges Starten

Ersetze in den folgenden Befehlen `VEHICLE_NAME` immer durch den echten Namen deines Roboters (z.B. `duckiebot` oder `gundel`)!

### Challenge 1 & 2: Follow Lane (Parkour)
Startet das Basis-Spurfolgen inklusive Kreuzungserkennung:
```bash
bash launchers/follow_lane.sh
```

### Challenge 3: Avoid Ducks (Wendehammer)
Startet das Fahren mit integrierter YOLO-Hinderniserkennung (Enten ausweichen):
```bash
bash launchers/avoid_ducks.sh
```

### Challenge 4: Mapping (Topologische Karte erstellen)
Startet die Exploration, bei der der Roboter den Graphen abfährt und Gate-Schilder verortet:
```bash
bash launchers/mapping.sh
```
- **Mapping manuell starten:** (Löscht alte Karten und beginnt frisch)
  ```bash
  rosservice call /VEHICLE_NAME/mapping4/start
  ```
- **Karte speichern (Export):**
  ```bash
  rosservice call /VEHICLE_NAME/mapping4/export
  ```

*(Hinweis: Automatisches Speichern beim Schließen per `Strg+C` ist ebenfalls eingebaut!)*

### Challenge 4: Navigation (Ziele abfahren)
Sobald das Mapping abgeschlossen ist, nutzt du den Navigationsmodus, um Schilder in einer bestimmten Reihenfolge abzufahren:
```bash
# Terminal 1: Startet die Navigation (Roboter ist im Standby)
bash launchers/navigation.sh

# Terminal 2: Route beauftragen (z.B. Gate 7, dann 9, dann 8)
rostopic pub /VEHICLE_NAME/navigation/route std_msgs/String "data: '[7, 9, 8]'"
```

---

##  Nützliche Tipps & Trouble-Shooting

- **GUI-Fenster öffnen sich nicht (Docker Fehler):**
  Führe im *globalen* Terminal (außerhalb des Containers) folgenden Befehl aus, um dem Docker Zugriff auf den Bildschirm zu gewähren:
  ```bash
  xhost +local:root
  ```
  Zusätzlich, falls ein Node meldet, er finde kein Display:
  ```bash
  export DISPLAY=:0
  ```

- **ChArUco Kamera-Kalibrierung:**
  Während der `charuco_calibration_node` läuft, kannst du die Homographie mit folgendem Service speichern:
  ```bash
  rosservice call /VEHICLE_NAME/calibration/save_homography
  ```

- **Optionale Mapping-Parameter (Challenge 4):**
  Wenn du möchtest, dass der Roboter die Karte automatisch exportiert und sofort danach ohne Terminal-Eingriff in den Navigations-Modus wechselt, setze diese Parameter:
  ```bash
  rosparam set /mapping_topological_node/auto_export_mapping true
  rosparam set /mapping_topological_node/auto_start_navigation true
  rosparam set /mapping_topological_node/navigation_start_delay 2.0
  ```
