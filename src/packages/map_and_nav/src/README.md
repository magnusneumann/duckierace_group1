# Map and Navigate (Challenge 4)

Dieses Paket ist das komplexeste der Duckietown-Challenges. Es baut auf der Spurfolge auf und versetzt den Roboter in die Lage, selbstständig durch ein Netzwerk von Kreuzungen zu navigieren. Das System besteht aus zwei unabhängigen, aber stark gekoppelten Phasen: **Topologisches Mapping** (Kartierung) und **Globale Navigation**.

## Starten (Launcher)

**1. Mapping (Erkunden der Welt):**
```bash
./launchers/mapping.sh
```
Der Roboter fährt autonom durch das Graphen-Netzwerk, klappert alle Kreuzungen ab, erkennt Edge-Marker (AprilTags) und speichert am Ende eine Map-Datei (`challenge4_mapping.json`) mit allen gefundenen Schildern. Der Vorgang kann jederzeit sicher mit `Strg+C` beendet und gespeichert werden.

**2. Navigation (Gezieltes Abfahren):**
```bash
./launchers/navigation.sh
```
Der Roboter lädt die zuvor erkundete Map, verweilt im `STANDBY`-Modus und wartet auf eine Liste von Zielen. Du schickst ihm eine Routen-Anfrage über ein ROS-Topic:
```bash
rostopic pub /dein_vehicle_name/navigation/route std_msgs/String "data: '[7, 10, 8]'"
```

## Architektur & Technische Funktionsweise

Das Herzstück bildet die Transformation der physischen Straßen in eine **Topologische Karte** (`topological_graph.py`). Die Welt besteht nicht aus GPS-Koordinaten, sondern aus **Knoten** (Nodes = Kreuzungen) und **Kanten** (Edges = Straßen). Jeder Knoten hat **Ports** (1, 2, 3, 4) als physikalische Ein- und Ausfahrten.

### Perzeption (Das Erkennen der Welt)
- **Rote Linien:** Eine dedizierte OpenCV-Maske (HSV) sucht in Bodennähe nach horizontalen, dicken roten Linien. Erkennt der Bot eine rote Linie, weiß er: *Hier ist eine Stopplinie vor einer Kreuzung.* (`detect_and_control_lane_node.py` publiziert `stop_line=True`).
- **Kreuzungs-Verhalten (`cross_intersection_node.py`):** Anstatt Linien zu folgen, führt dieser Node einen "Blindflug"-Kurven-Algorithmus aus. Bekommt er den Befehl "right", fährt er stur für eine bestimmte Dauer im harten Winkel nach rechts, bis die Kamera wieder eine gelbe Mittelstreifen-Linie einfängt.
- **AprilTags (`detect_tags_node.py`):** Mithilfe der AprilTag-Bibliothek werden die IDs der Schilder aus dem Bildraum extrahiert.

### State Machine (Der Dirigent)
`switch_control_node.py` steuert den Lebenszyklus des Roboters komplett hierarchisch:
1. `STANDBY`: Räder blockiert, wartet auf Ziele (Navigation-Modus).
2. `LANE_FOLLOWING`: Gibt das Steuer an den PID-Regler der Kamera frei.
3. `STOPPED_AT_LINE`: Wenn eine rote Linie erkannt wurde, friert er ein, hält 2 Sekunden an und triggert dann das Kreuzungs-Manöver.
4. `CROSSING_INTERSECTION`: Übergibt das Steuer an den Intersection-Node, der den Bot stumpf über die Kreuzung drückt. Ignoriert in dieser Phase rote Linien.

### Logik-Ebene 1: Mapping (`mapping_topological_node.py`)
Während der Mapping-Phase führt der Roboter einen Zufalls-Spaziergang (Random Walk) aus. Jedes Mal, wenn er an eine Stopplinie kommt, würfelt er zufällig, wo er abbiegt. Während er auf den Straßen (Kanten) fährt und ein AprilTag sieht, trägt er in sein internes Wörterbuch ein: *"Auf der Kante A.1--B.1 hängt das Schild 7"*. Am Ende des Mappings speichert er diesen Graphen ab.

### Logik-Ebene 2: Navigation (`navigator_node.py`)
Wenn der Nutzer die Route `[7, 10, 8]` anfordert:
1. **Zustands-Lokalisierung:** Der Roboter liest aus dem beim Mapping erstellten JSON aus, an welcher Kante oder vor welchem Knoten er gerade steht. *(Wichtig: Der Roboter darf nach dem Mapping nicht willkürlich woanders auf die Strecke gestellt werden, sondern exakt vor die im JSON definierte Stopplinie).*
2. **Dijkstra's Algorithmus:** Der Graphen-Rechner nutzt Dijkstra, um die kürzeste Route von Schild 7 zu Schild 10 zu Schild 8 zu berechnen. 
   - *Besonderheit (Stateful Dijkstra):* Da Duckiebots auf einer zweispurigen Straße fahren, sind **U-Turns physisch unmöglich**. Der Algorithmus speichert deshalb als Status nicht nur "Knoten B", sondern "Knoten B erreicht über Port 3". Dadurch erzwingt die Logik, dass eine Strecke, die ihn geradeaus zu einer Sackgasse schicken würde, sofort im Dijkstra bestraft wird.
3. **Execution:** Der Navigator legt eine Liste aller Kanten (`route_steps`) an. Jedes Mal, wenn der `switch_control_node` an einer roten Linie ankommt, ruft er beim Navigator an. Der Navigator löscht den obersten Eintrag aus seiner Liste und antwortet mit dem nächsten Abbiege-Kommando (`straight`, `left`, `right`).

### Debugging & Visualisierung
Die komplexe Topologie wird live mit `networkx` und `matplotlib` im `debug_navigation_node.py` gerendert. 
- Das Netzwerk der Kanten wird abstrakt aufgezeichnet. 
- Die zukünftig **geplante** Route (laut Dijkstra) leuchtet Blau (`PLANNED`). 
- Die Kante, auf der er sich **aktuell** befindet, leuchtet Rot (`ACTIVE`). 
- Die textlichen Kanten-Labels (z.B. `A.1--B.1`) sind die Source-of-Truth und binden die abstrakte Matrix-Logik an die physische Welt.
