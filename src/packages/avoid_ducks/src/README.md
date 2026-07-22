# Avoid Ducks (Challenge 3)

Dieses Paket implementiert das autonome Ausweichen von Hindernissen – speziell von Enten (Rubber Ducks) auf der Fahrbahn. Im Gegensatz zu einer hierarchischen State-Machine vereint dieses Paket die komplette Logik von Spurfolge (Lane Following) und Hinderniserkennung (YOLO) direkt in einer zonenbasierten Wahrnehmung.

## Starten (Launcher)

Das autonome Fahren inklusive Kollisionsvermeidung wird gestartet über:
```bash
./launchers/avoid_ducks.sh
```
Dieser Launcher startet die Objekterkennungs-Inferenz (`detect_ducks_node.py`), die Kamera-Kalibrierung sowie den hochintegrierten `duck_avoidance_node.py`, der sowohl lenkt als auch ausweicht.

## Architektur & Technische Funktionsweise

Die Herausforderung besteht hier darin, klassische Bildverarbeitung (Linien-Tracking) mit neuronalen Netzen in Echtzeit zu kombinieren, ohne dass sich die Steuerungssignale widersprechen. Statt einer simplen P-Regelung nutzt dieses System ein projiziertes Vogelperspektiven-Modell (Homographie).

### 1. Objekterkennung (YOLO) - `detect_ducks_node.py`
Dieser Node integriert ein trainiertes YOLO-Modell (You Only Look Once), um Objekte im Kamerabild zu klassifizieren und zu lokalisieren.
- **Inferenz:** Das Kamerabild wird durch das neuronale Netz geschickt. Das Netz liefert Bounding Boxen (`Polygon`-Nachrichten) und Konfidenz-Werte (Probability) für gefundene Enten zurück.
- **Publisher:** Die Boxen der Enten werden auf dem Topic `/{vehicle}/detect/duck_obstacles` veröffentlicht, wo sie vom Haupt-Node (Avoidance Node) verarbeitet werden.

### 2. Zonen-Logik & Regelung - `duck_avoidance_node.py`
Dieses Node ersetzt den klassischen Lane-Follower und Switch-Control komplett. Es projiziert das 2D-Kamerabild mithilfe von Intrinsics und Homographie in den 3D-Raum (bzw. auf die 2D-Bodenebene), um echte Metriken (Zentimeter) zu erhalten.

Die Fahrbahn direkt vor dem Roboter wird in **drei physische Zonen** aufgeteilt (z.B. Zone 1: ganz nah, Zone 2: mittel, Zone 3: fern).
- **Perzeption (Segmentierung):** 
  - Die Kamera extrahiert mithilfe von HSV-Masken weiße (rechter Rand) und gelbe (Mittelstreifen) Pixel.
  - Das System überprüft mathematisch (via `shapely.geometry`), wie viele weiße/gelbe Pixel oder detektierte Enten-Bounding-Boxen in die jeweiligen Zonen fallen.
- **Lane Following (Spurfolge):** 
  - Das Lenken (`omega`) passiert hierbei rein basierend darauf, welche Zonen blockiert sind!
  - Blockiert der rechte Bildrand (weiße Linie) die Zone, lenkt er nach links. Blockiert der gelbe Mittelstreifen die Zone, lenkt er nach rechts.
- **Enten-Ausweichen (Duck Avoidance):** 
  - Sobald eine Ente in einer der Zonen vor dem Roboter registriert wird, greift sofort eine Ausweich- oder Bremslogik ein.
  - Befindet sich eine Ente in Zone 1 (kritisch nah), stoppt der Roboter sofort (`v=0`).
  - Je nachdem, in welcher Zone (links/rechts/mitte) eine Ente in der Ferne erkannt wird, lenkt der Roboter geschmeidig in die gegenüberliegende freie Zone aus, um das Objekt rechtzeitig zu umfahren (Wiggle-Bewegung / Ausweichmanöver).

### 3. Effizienz und Hardware
Das Ausführen neuronaler Netze auf Edge-Devices ist rechenintensiv. Das Enten-Netz (YOLO) wertet die Bilder daher oft asynchron oder in gedrosselter Auflösung aus, während die Zonen-Segmentierung (Homographie & HSV) für die reine Spurführung in hoher Taktfrequenz (`buffer_size`) weiterläuft, um die dynamische Stabilität in Kurven und beim Umfahren der Hindernisse zu garantieren.
