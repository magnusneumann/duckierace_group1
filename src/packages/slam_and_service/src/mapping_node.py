#!/usr/bin/env python3
import os
import cv2
import math
import numpy as np
import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import CompressedImage

class MappingNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._v = os.environ.get('VEHICLE_NAME', 'gundel')

        # --- ODOMETRIE PARAMETER (Duckiebot Standard) ---
        self.R = 0.033  # Radradius in Metern
        self.N = 135     # Encoder-Ticks pro Umdrehung
        self.L = 0.10    # Radabstand (Baseline) in Metern -> TUNING WERT!

        self.ticks_left = None
        self.ticks_right = None

        # --- ROBOTER POSE (Weltkoordinaten) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- KARTEN DATEN ---
        self.path = [(0.0, 0.0)] # Array für die weiße Linie
        self.nodes = []          # Gespeicherte Kreuzungen
        self.current_tag = "Unknown"

        # Subscriber
        rospy.Subscriber(f"/{self._v}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_left)
        rospy.Subscriber(f"/{self._v}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_right)
        rospy.Subscriber(f"/{self._v}/detect/stopline", String, self.cb_stopline)
        
        # Abhören, was der Sign-Node als letztes gesehen hat
        rospy.Subscriber(f"/{self._v}/intersection/turn_decision", String, self.cb_tag) 

        # Publisher für das Map-Bild
        self.pub_map = rospy.Publisher(f"/{self._v}/debug/map", CompressedImage, queue_size=1)

        # 10Hz Timer zeichnet die Karte
        rospy.Timer(rospy.Duration(0.1), self.draw_map)
        rospy.loginfo("Mapping Node (SLAM) gestartet!")

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
        # Zurückgelegte Strecke der einzelnen Räder in Metern
        d_left = (d_left_ticks / self.N) * (2 * math.pi * self.R)
        d_right = (d_right_ticks / self.N) * (2 * math.pi * self.R)

        # Kinematik-Modell des Differentialantriebs
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.L

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Wir speichern nicht jeden Mikrometer, sondern nur alle 2 cm einen Punkt für den Plot
        if math.dist((self.x, self.y), self.path[-1]) > 0.02:
            self.path.append((self.x, self.y))

    def cb_tag(self, msg):
        self.current_tag = msg.data

    def cb_stopline(self, msg):
        # Relativer Vektor zur Kreuzung: 20cm vor (x), 20cm links (y)
        dx = 0.20
        dy = 0.20
        
        # 2D-Rotationsmatrix, um das auf die globale Map umzulegen
        center_x = self.x + (dx * math.cos(self.theta) - dy * math.sin(self.theta))
        center_y = self.y + (dx * math.sin(self.theta) + dy * math.cos(self.theta))

        radius = 0.25 # Toleranz-Radius (25 cm)
        found_existing = False

        # Loop Closure Check: Sind wir nah an einer bekannten Kreuzung?
        for node in self.nodes:
            if math.dist((center_x, center_y), (node['x'], node['y'])) < radius:
                rospy.loginfo("Bekannte Kreuzung erreicht! Snapping...")
                
                # LOOP CLOSURE: Drift killen! Wir zwingen den Roboter rechnerisch 
                # exakt auf die Position, an der er beim letzten Besuch dieser Kreuzung stand.
                self.x = node['x'] - (dx * math.cos(self.theta) - dy * math.sin(self.theta))
                self.y = node['y'] - (dx * math.sin(self.theta) + dy * math.cos(self.theta))
                
                found_existing = True
                break

        if not found_existing:
            rospy.loginfo(f"Neue Kreuzung registriert: {center_x:.2f}, {center_y:.2f}")
            self.nodes.append({'x': center_x, 'y': center_y, 'tag': self.current_tag})

    def draw_map(self, event):
        if self.pub_map.get_num_connections() == 0:
            return

        # Schwarze Leinwand (600x600 Pixel)
        img = np.zeros((600, 600, 3), dtype=np.uint8)
        
        # Skalierung: Startpunkt ist genau in der Mitte (300, 300)
        # 1 Meter entspricht 100 Pixeln
        cx, cy = 300, 300
        scale = 100 

        # Hilfsfunktion: Wandelt Welt-Meter in Bild-Pixel um (Y invertiert)
        def to_px(x, y):
            return int(cx + x * scale), int(cy - y * scale)

        # 1. Gefahrene Strecke zeichnen (Weiß)
        if len(self.path) > 1:
            for i in range(1, len(self.path)):
                pt1 = to_px(self.path[i-1][0], self.path[i-1][1])
                pt2 = to_px(self.path[i][0], self.path[i][1])
                cv2.line(img, pt1, pt2, (255, 255, 255), 2)

        # 2. Kreuzungen als Kreise zeichnen (Blau)
        for i, node in enumerate(self.nodes):
            nx, ny = to_px(node['x'], node['y'])
            # Zeichnet einen Kreis mit exakt 25cm Radius (25 Pixel)
            cv2.circle(img, (nx, ny), int(0.25 * scale), (255, 0, 0), 2)
            cv2.putText(img, f"Node {i}", (nx - 20, ny - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 150, 0), 1)

        # 3. Roboter Position (Roter Punkt) + Blickrichtung (Rote Linie)
        rx, ry = to_px(self.x, self.y)
        cv2.circle(img, (rx, ry), 5, (0, 0, 255), -1)
        
        hx, hy = to_px(self.x + 0.15 * math.cos(self.theta), self.y + 0.15 * math.sin(self.theta))
        cv2.line(img, (rx, ry), (hx, hy), (0, 0, 255), 2)

        # Bild komprimieren und an Dashboard funken
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        self.pub_map.publish(msg_out)

if __name__ == "__main__":
    node = MappingNode("mapping_node")
    rospy.spin()