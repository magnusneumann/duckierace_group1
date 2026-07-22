# Follow Lane & Kreuzungen (Challenge 1 & 2)

Dieses Paket implementiert die grundlegende Fähigkeit des Duckiebots, Fahrbahnmarkierungen (weiße und gelbe Linien) zu erkennen, autonom innerhalb seiner Spur zu bleiben (Challenge 1) und Kreuzungen sicher zu überqueren (Challenge 2). Es bildet das Fundament für alle weiteren Navigationsaufgaben in Duckietown.

## Starten (Launcher)

Das autonome Fahren in der Fahrspur inklusive Kreuzungserkennung kann über das Shell-Skript gestartet werden:
```bash
./launchers/follow_lane.sh
```
Dieses Skript startet die ROS-Umgebung, die Perzeptions- und Regelungs-Nodes sowie die State-Machine (FSM) für das Überqueren von Kreuzungen. Zudem wird das visuelle Dashboard (`dashboard_node.py`) gestartet, welches ein Live-Feedback über die Kamera und die Bildverarbeitung liefert.

## Architektur & Technische Funktionsweise

Das Paket ist in drei Hauptkomponenten unterteilt: Perzeption (Linien und Stopplinien), Regelung (Steuerung) und State-Machine (Kreuzungen).

### 1. Perzeption (`detect_lane_node.py`)
Dieser Node ist das "Auge" des Roboters. Er abonniert den Kamera-Stream und extrahiert die relevanten Spur-Informationen:
- **Cropping (ROI):** Das Kamerabild wird im oberen Bereich abgeschnitten (Region of Interest), da der Himmel oder Objekte außerhalb der Strecke für die Spurführung irrelevant sind. Das spart enorm Rechenleistung.
- **Farbfilterung (HSV):** Das Bild wird vom RGB- in den HSV-Farbraum konvertiert. Über definierte Farbmasken werden gezielt weiße Linien (rechter Rand), gelbe Linien (Mittelstreifen) und rote Stopplinien (Kreuzungen) extrahiert.
- **Kantenerkennung (Canny Edge Detection):** Auf die gefilterten Bereiche wird der Canny-Algorithmus angewandt, um harte Konturen herauszuarbeiten.
- **Linienkoordinaten (Center of Mass):** Aus den detektierten weißen und gelben Pixeln wird ein gewichteter Mittelpunkt berechnet. Das Node publiziert diese Zielkoordinaten auf dem Topic `/{vehicle}/detect/lane_center`. Erkennt das Modul eine rote horizontale Linie, publiziert es `stop_line = True`.

### 2. Regelung (`control_lane_node.py`)
Dieser Node übernimmt das "Gehirn" der Bewegung im Standardfall. Er verarbeitet die Zielkoordinaten:
- **Fehlerberechnung:** Er misst, wie weit der Linienmittelpunkt von der idealen Bildmitte abweicht (der sogenannte *Error*).
- **PID-Regler:** Die Steuerungslogik verwendet einen Proportional-Regler (P-Regler).
  - Je größer die Abweichung vom Zentrum (Error), desto stärker wird die Winkelgeschwindigkeit (`omega`) entgegengesteuert, um den Roboter zurück in die Spurmitte zu drücken.
  - Die Vorwärtsgeschwindigkeit (`v`) ist dynamisch gekoppelt: In engen Kurven (großer Error) bremst der Roboter ab, auf geraden Strecken (kleiner Error) gibt er Gas.

### 3. State Machine & Kreuzungen (`switch_control_node.py` & `cross_intersection_node.py`)
Da das System nicht nur stur Linien folgen, sondern auch an Kreuzungen agieren soll, wird die Steuerung über eine FSM orchestriert:
- **`switch_control_node.py` (Der Dirigent):**
  - Standardmäßig im `LANE_FOLLOWING`-State: Die Motorbefehle des `control_lane_node` gehen ungehindert an die Räder.
  - Sobald eine rote Stopplinie detektiert wird, wechselt die Maschine auf `STOPPED_AT_LINE`. Der Roboter bremst hart (`v=0, omega=0`) und wartet 2 Sekunden (Stoppschild-Logik).
  - Danach wechselt sie auf `CROSSING_INTERSECTION`.
- **`cross_intersection_node.py` (Der Kreuzungs-Manager):**
  - Während der Roboter die Kreuzung überquert, lenkt dieser Node den Roboter in einem berechneten Blindflug (Open-Loop), basierend auf der Entscheidung (links, rechts, geradeaus). 
  - Die Entscheidung kann über Schilder (AprilTags via `detect_sign_node.py`) vorgegeben werden oder fällt im Zweifel auf "geradeaus" zurück.
  - Sobald die Kurve vollendet ist und wieder durchgehend eine gelbe Mittelstreifen-Linie gesichtet wird, übergibt das System die Kontrolle zurück an das klassische `LANE_FOLLOWING`.
