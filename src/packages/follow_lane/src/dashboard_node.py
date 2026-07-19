#!/usr/bin/env python3
import os
import json
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class DashboardNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "gundel")

        # Platzhalter für die Bilder (schwarz)
        self.imgs = {
            "duck": np.zeros((300, 300, 3), dtype=np.uint8),
            "tags": np.zeros((300, 300, 3), dtype=np.uint8),
            "lane": np.zeros((300, 300, 3), dtype=np.uint8),
            "map": np.zeros((300, 300, 3), dtype=np.uint8),
            "graph": np.zeros((300, 300, 3), dtype=np.uint8),
            "info": np.zeros((300, 300, 3), dtype=np.uint8),
            "blank": np.zeros((300, 300, 3), dtype=np.uint8),
        }
        self.mapping_status = {}

        # Subscriber für alle Debug-Topics
        rospy.Subscriber(f"/{self._vehicle_name}/debug/duck_detection", CompressedImage, self.cb_img, callback_args="duck", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/tags", CompressedImage, self.cb_img, callback_args="tags", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/lane_croped", CompressedImage, self.cb_img, callback_args="lane", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/map", CompressedImage, self.cb_img, callback_args="map", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/info", CompressedImage, self.cb_img, callback_args="info", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/mapping/status", String, self.cb_mapping_status, queue_size=1)

        # 10 Hz Timer, der das Dashboard zeichnet
        rospy.Timer(rospy.Duration(0.1), self.draw_dashboard)
        rospy.loginfo("Debug Dashboard gestartet!")

    def cb_img(self, msg, key):
        # Bild decodieren
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Manche Masken sind Graustufen, wir brauchen aber RGB zum Zusammenkleben
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
        # Auf Einheitspreis skalieren (z.B. 300x300), damit das Grid nicht zerbricht
        self.imgs[key] = cv2.resize(img, (300, 300))

    def cb_mapping_status(self, msg):
        try:
            self.mapping_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.mapping_status = {"raw": msg.data}
        self.imgs["graph"] = self._render_mapping_panel(self.mapping_status)

    def _render_mapping_panel(self, status):
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        y = 25
        cv2.putText(img, "Mapping-Vergleich", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y += 30

        def put(line, color=(255, 255, 255)):
            nonlocal y
            cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y += 22

        if not status:
            put("Keine Mapping-Daten empfangen.", (100, 100, 100))
            return img

        if "status" in status:
            put(f"Status: {status.get('status')}")
        if "current_edge" in status:
            put(f"Aktuelle Kante: {status.get('current_edge')}")
        if "edge" in status and status.get("edge") != status.get("current_edge"):
            put(f"Zuletzt gemappt: {status.get('edge')}")
        if "mapped_gate" in status:
            put(f"Gate erkannt: {status.get('mapped_gate')}")
        if "mapped_edges" in status and status.get("mapped_edges"):
            put("Bekannte Tore:")
            mapped = status.get("mapped_edges")
            for gate_id in sorted(mapped, key=lambda x: int(x))[:5]:
                put(f" {gate_id} => {mapped[gate_id]}")
            if len(mapped) > 5:
                put(f" ...{len(mapped)-5} weitere", (200, 200, 200))
        elif "raw" in status:
            put(f"Raw: {status['raw']}", (200, 200, 200))
        else:
            put("Noch keine Tore gemappt.", (200, 200, 200))

        return img

    def draw_dashboard(self, event):
        # Beschriftungen hinzufügen
        labeled_imgs = {}
        for key, img in self.imgs.items():
            labeled = img.copy()
            cv2.putText(labeled, key.upper(), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            labeled_imgs[key] = labeled

        # Reihe 1 zusammenkleben (H-Stack)
        row1 = cv2.hconcat([labeled_imgs["duck"], labeled_imgs["tags"], labeled_imgs["lane"]])
        
        # Reihe 2 zusammenkleben
        row2 = cv2.hconcat([labeled_imgs["graph"], labeled_imgs["map"], labeled_imgs["info"]])

        # Reihe 3 leer (nur zur Vereinheitlichung)
        row3 = cv2.hconcat([labeled_imgs["blank"], labeled_imgs["blank"], labeled_imgs["blank"]])
        
        # Alles vertikal zusammenkleben (V-Stack)
        dashboard = cv2.vconcat([row1, row2, row3])

        # Ein EINZIGES Fenster öffnen
        cv2.imshow("DuckieRace Control Center", dashboard)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = DashboardNode("dashboard_node")
    node.run()