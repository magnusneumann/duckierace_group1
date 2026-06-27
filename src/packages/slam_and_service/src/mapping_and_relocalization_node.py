#!/usr/bin/env python3
import os
import math
import json
import rospy
import cv2
import numpy as np
import networkx as nx
from networkx.readwrite import json_graph
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import WheelEncoderStamped
from std_srvs.srv import Trigger, TriggerResponse

class MapRelocNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._v = os.environ.get('VEHICLE_NAME', 'duckiebot')
        
        # --- ODOMETRIE PARAMETER (Duckiebot Standard) ---
        self.R = 0.215  # Radradius in Metern
        self.N = 135    # Encoder-Ticks pro Umdrehung
        self.L = 0.10   # Radabstand (Baseline) in Metern -> TUNING WERT!

        self.ticks_left = None
        self.ticks_right = None

        # --- ZUSTAND & POSE ---
        # START-MODUS auf IDLE geändert! Der Knoten wartet nun auf Service Calls.
        self.mode = "IDLE" 
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        # --- GRAPH & MAP ---
        self.map_file_path = "/root/DuckieRace/src/packages/avoid_ducks/config/parkour_map.json"
        self.graph = nx.Graph()
        self.last_node_id = None
        self.last_seen_tag = None
        
        # Temporärer Speicher für den exakten, kurvigen Weg ZWISCHEN zwei Kreuzungen
        self.first_intersection_reached = False
        self.current_path_segment = []

        # --- ROS Services ---
        rospy.Service(f"/{self._v}/mapping/start", Trigger, self.srv_start_mapping)
        rospy.Service(f"/{self._v}/mapping/export", Trigger, self.srv_export_map)
        rospy.Service(f"/{self._v}/mapping/relocalize", Trigger, self.srv_start_relocalization)
        
        # --- ROS Subscriber ---
        rospy.Subscriber(f"/{self._v}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_left)
        rospy.Subscriber(f"/{self._v}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_right)
        
        rospy.Subscriber(f"/{self._v}/detect/stop_line", Bool, self.cb_stopline)
        rospy.Subscriber(f"/{self._v}/detect/tag_id", String, self.cb_tag)

        # Timer für die Visualisierung (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self.render_map)
        rospy.loginfo("Mapping Node im IDLE Modus gestartet. Warte auf Service Calls (/mapping/start, /export, /relocalize)!")

    # ==========================================
    # ROS SERVICE CALLBACKS
    # ==========================================
    def srv_start_mapping(self, req):
        self.mode = "MAPPING"
        self.graph.clear()
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.first_intersection_reached = False
        self.current_path_segment = []
        self.last_node_id = None
        self.last_seen_tag = None
        return TriggerResponse(success=True, message="Mapping gestartet. Warte auf erste Kreuzung...")

    def srv_export_map(self, req):
        self.mode = "IDLE"
        try:
            data = json_graph.node_link_data(self.graph)
            with open(self.map_file_path, 'w') as f:
                json.dump(data, f, indent=4)
            return TriggerResponse(success=True, message=f"Map mit {len(self.graph.nodes)} Knoten gespeichert.")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Fehler beim Speichern: {e}")

    def srv_start_relocalization(self, req):
        try:
            with open(self.map_file_path, 'r') as f:
                data = json.load(f)
            self.graph = json_graph.node_link_graph(data)
            self.mode = "RELOCALIZING"
            return TriggerResponse(success=True, message="Map geladen. Relokalisation aktiv.")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Keine Map gefunden: {e}")

    # ==========================================
    # ODOMETRIE CALLBACKS
    # ==========================================
    def cb_left(self, msg):
        if self.ticks_left is None:
            self.ticks_left = msg.data
            return
        delta_ticks = msg.data - self.ticks_left
        self.ticks_left = msg.data
        self.update_odometry(delta_ticks, 0)

    def cb_right(self, msg):
        if self.ticks_right is None:
            self.ticks_right = msg.data
            return
        delta_ticks = msg.data - self.ticks_right
        self.ticks_right = msg.data
        self.update_odometry(0, delta_ticks)

    def update_odometry(self, d_left_ticks, d_right_ticks):
        if self.mode != "MAPPING":
            return

        d_left = (d_left_ticks / self.N) * (2 * math.pi * self.R)
        d_right = (d_right_ticks / self.N) * (2 * math.pi * self.R)

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.L

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Wenn wir uns mehr als 2cm bewegt haben, Punkt zur Kurve hinzufügen
        if self.first_intersection_reached and len(self.current_path_segment) > 0:
            if math.dist((self.x, self.y), self.current_path_segment[-1]) > 0.02:
                self.current_path_segment.append((self.x, self.y))

    # ==========================================
    # KREUZUNGS- UND SCHILDER-LOGIK
    # ==========================================
    def cb_tag(self, msg):
        tag_id = msg.data
        if tag_id == "None" or not tag_id: return
        self.last_seen_tag = tag_id
        
        if self.mode == "RELOCALIZING":
            for u, v, data in self.graph.edges(data=True):
                if data.get('tag_id') == tag_id:
                    node_data = self.graph.nodes[u]
                    self.x = node_data['x']
                    self.y = node_data['y']
                    rospy.loginfo(f"RELOKALISIERT! Tag {tag_id} gefunden.")
                    self.mode = "MAPPING"
                    self.last_node_id = u
                    self.current_path_segment = [(self.x, self.y)]
                    break

    def _average_paths(self, path1, path2, num_points=50):
        """Mittelt zwei Pfade (Arrays aus X/Y Koordinaten), interpoliert sie auf gleiche Länge"""
        if len(path1) < 2 or len(path2) < 2:
            return path1
            
        p1 = np.array(path1)
        p2 = np.array(path2)
        
        # Überprüfen, in welche Richtung der Bot gefahren ist
        # Abstand zwischen Startpunkten vs. Abstand Start1 zu Ende2
        dist_straight = math.hypot(p1[0][0] - p2[0][0], p1[0][1] - p2[0][1])
        dist_reversed = math.hypot(p1[0][0] - p2[-1][0], p1[0][1] - p2[-1][1])
        
        # Wenn entgegengesetzt gefahren, Array 2 umdrehen
        if dist_reversed < dist_straight:
            p2 = p2[::-1] 
            
        def resample(path):
            # Berechnet die kummulative Distanz und interpoliert N gleichmäßige Punkte
            diffs = np.diff(path, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            cum_dists = np.concatenate(([0], np.cumsum(dists)))
            total_dist = cum_dists[-1]
            if total_dist == 0: return path
            target_dists = np.linspace(0, total_dist, num_points)
            rx = np.interp(target_dists, cum_dists, path[:, 0])
            ry = np.interp(target_dists, cum_dists, path[:, 1])
            return np.column_stack((rx, ry))
            
        res1 = resample(p1)
        res2 = resample(p2)
        
        # Durchschnitt berechnen
        avg = (res1 + res2) / 2.0
        return [(float(x), float(y)) for x, y in avg]

    def cb_stopline(self, msg):
        if self.mode != "MAPPING": return
        if not msg.data: return
        
        # Relativer Vektor zur Kreuzung: 20cm vor (x), 20cm links (y)
        dx = 0.225
        dy = 0.13
        
        # NEU: Startpunkt an der allerersten Kreuzung festlegen
        if not self.first_intersection_reached:
            rospy.loginfo("Erste Kreuzung erreicht! Setze Startpunkt (Node 0) auf (0,0).")
            self.first_intersection_reached = True
            
            # Die Kreuzung (Node) wird in den Ursprung (0,0) gelegt.
            # Da der Roboter auf der rechten Spur steht, setzen wir ihn relativ nach hinten/rechts.
            self.theta = 0.0
            self.x = -dx
            self.y = -dy
            
            # Erste Node (Node 0) exakt bei (0,0) erstellen
            self.last_node_id = self._add_or_get_node(0.0, 0.0, snap_radius=0.1)
            self.current_path_segment = [(self.x, self.y)]
            return
        
        # Debouncing: Bot muss mind. etwas fahren, bevor eine neue Stopplinie triggert
        if len(self.current_path_segment) < 10: 
            return 

        center_x = self.x + (dx * math.cos(self.theta) - dy * math.sin(self.theta))
        center_y = self.y + (dx * math.sin(self.theta) + dy * math.cos(self.theta))

        # Prüfe ob wir eine bekannte Kreuzung im (erhöhten) Radius erreichen
        snap_radius = 0.55 # Radius auf 35 cm erhöht
        snapped_node_id, found_existing = self._find_node_in_radius(center_x, center_y, radius=snap_radius)

        if found_existing:
            rospy.loginfo(f"Bekannte Kreuzung {snapped_node_id} erreicht! Snapping...")
            node = self.graph.nodes[snapped_node_id]
            # Loop Closure Drift-Korrektur
            self.x = node['x'] - (dx * math.cos(self.theta) - dy * math.sin(self.theta))
            self.y = node['y'] - (dx * math.sin(self.theta) + dy * math.cos(self.theta))
        else:
            snapped_node_id = self._add_or_get_node(center_x, center_y, snap_radius=0.1)
            rospy.loginfo(f"Neue Kreuzung {snapped_node_id} registriert.")

        # KANTE ERSTELLEN ODER MITTELN
        if self.last_node_id is not None and self.last_node_id != snapped_node_id:
            if not self.graph.has_edge(self.last_node_id, snapped_node_id):
                # Neue Kante
                edge_attrs = {"path": list(self.current_path_segment)}
                if self.last_seen_tag is not None:
                    edge_attrs["tag_id"] = self.last_seen_tag
                    self.last_seen_tag = None
                self.graph.add_edge(self.last_node_id, snapped_node_id, **edge_attrs)
            else:
                # Kante existiert bereits -> Mitteln!
                existing_path = self.graph[self.last_node_id][snapped_node_id]['path']
                averaged_path = self._average_paths(existing_path, self.current_path_segment)
                self.graph[self.last_node_id][snapped_node_id]['path'] = averaged_path
                
                # Tag ggf. nachtragen (Falls im 1. Durchlauf Schild übersehen wurde)
                if self.last_seen_tag is not None:
                    self.graph[self.last_node_id][snapped_node_id]['tag_id'] = self.last_seen_tag
                    self.last_seen_tag = None
                    
                rospy.loginfo(f"Kante {self.last_node_id}-{snapped_node_id} befahren. Odometrie-Pfade gemittelt!")

        # Reset für den nächsten Streckenabschnitt
        self.last_node_id = snapped_node_id
        self.current_path_segment = [(self.x, self.y)]

    def _find_node_in_radius(self, x, y, radius):
        for n_id, data in self.graph.nodes(data=True):
            if math.dist((x, y), (data['x'], data['y'])) < radius:
                return n_id, True
        return None, False

    def _add_or_get_node(self, x, y, snap_radius):
        n_id, found = self._find_node_in_radius(x, y, snap_radius)
        if found: return n_id
        new_id = len(self.graph.nodes)
        self.graph.add_node(new_id, x=x, y=y)
        return new_id

    # ==========================================
    # VISUALISIERUNG & MAIN LOOP
    # ==========================================
    def render_map(self, event=None):
        img = np.zeros((600, 600, 3), dtype=np.uint8)
        
        # --- DYNAMISCHE KAMERA ---
        all_x = [self.x]
        all_y = [self.y]
        for nx, ny in self.current_path_segment:
            all_x.append(nx)
            all_y.append(ny)
        for n, data in self.graph.nodes(data=True):
            all_x.append(data['x'])
            all_y.append(data['y'])
        for u, v, data in self.graph.edges(data=True):
            if 'path' in data:
                for px, py in data['path']:
                    all_x.append(px)
                    all_y.append(py)
            
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        
        width_m = max(max_x - min_x, 1.0)
        height_m = max(max_y - min_y, 1.0)
        
        # 500px nutzbarer Raum (50px Rand pro Seite)
        scale = min(500 / width_m, 500 / height_m, 300) # Maximales reinzoomen begrenzen
        
        def to_pixel(wx, wy):
            px = int(300 + (wx - center_x) * scale)
            py = int(300 - (wy - center_y) * scale)
            return px, py

        # 1. Kanten zeichnen (Gespeicherte Fahrwege)
        for u, v, data in self.graph.edges(data=True):
            if 'path' in data:
                # Array von (x,y) in Array von Pixelkoordinaten umwandeln
                pts = np.array([to_pixel(px, py) for px, py in data['path']], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(img, [pts], isClosed=False, color=(255, 255, 255), thickness=2)
                
                if 'tag_id' in data:
                    mid_idx = len(data['path']) // 2
                    mid_pt = to_pixel(*data['path'][mid_idx])
                    cv2.putText(img, f"T:{data['tag_id']}", (mid_pt[0]+5, mid_pt[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # 2. Aktuellen Streckenabschnitt zeichnen (Gelb)
        if len(self.current_path_segment) > 1:
            pts = np.array([to_pixel(px, py) for px, py in self.current_path_segment], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(img, [pts], isClosed=False, color=(0, 255, 255), thickness=2)

        # 3. Knoten zeichnen (Blaue Kreise für Kreuzungen)
        for n_id, data in self.graph.nodes(data=True):
            pt = to_pixel(data['x'], data['y'])
            # 35cm Radius entspricht 0.35 * scale Pixeln
            r_px = max(int(0.35 * scale), 5) 
            cv2.circle(img, pt, r_px, (255, 0, 0), 2)
            cv2.putText(img, f"N{n_id}", (pt[0]-10, pt[1]-r_px-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 150, 0), 1)

        # 4. Aktuelle Roboterposition (Roter Punkt & Linie)
        bot_pt = to_pixel(self.x, self.y)
        cv2.circle(img, bot_pt, 6, (0, 0, 255), -1)
        dir_pt = to_pixel(self.x + 0.15 * math.cos(self.theta), self.y + 0.15 * math.sin(self.theta))
        cv2.line(img, bot_pt, dir_pt, (0, 0, 255), 2)

        # Overlay Info
        cv2.putText(img, f"Modus: {self.mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Zoom: {scale:.1f} px/m", (10, 580), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        cv2.imshow("Map", img)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = MapRelocNode('mapping_relocalization_node')
    node.run()