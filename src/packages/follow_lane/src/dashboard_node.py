#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

class DashboardNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "gundel")

        # Platzhalter für die Bilder (schwarz)
        self.imgs = {
            "duck": np.zeros((300, 300, 3), dtype=np.uint8),
            "sign": np.zeros((300, 300, 3), dtype=np.uint8),
            "lane": np.zeros((300, 300, 3), dtype=np.uint8),
            "white": np.zeros((300, 300, 3), dtype=np.uint8),
            "yellow": np.zeros((300, 300, 3), dtype=np.uint8),
            "map": np.zeros((300, 300, 3), dtype=np.uint8),
            "red": np.zeros((300, 300, 3), dtype=np.uint8)
        }

        # Subscriber für alle Debug-Topics
        rospy.Subscriber(f"/{self._vehicle_name}/debug/duck_detection", CompressedImage, self.cb_img, callback_args="duck", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/sign", CompressedImage, self.cb_img, callback_args="sign", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/lane_croped", CompressedImage, self.cb_img, callback_args="lane", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/lane_white", CompressedImage, self.cb_img, callback_args="white", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/lane_yellow", CompressedImage, self.cb_img, callback_args="yellow", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/map", CompressedImage, self.cb_img, callback_args="map", queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/lane_red", CompressedImage, self.cb_img, callback_args="red", queue_size=1)

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

    def draw_dashboard(self, event):
        # Beschriftungen hinzufügen
        labeled_imgs = {}
        for key, img in self.imgs.items():
            labeled = img.copy()
            cv2.putText(labeled, key.upper(), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            labeled_imgs[key] = labeled

        # Reihe 1 zusammenkleben (H-Stack)
        row1 = cv2.hconcat([labeled_imgs["duck"], labeled_imgs["sign"], labeled_imgs["lane"]])
        
        # Reihe 2 zusammenkleben
        row2 = cv2.hconcat([labeled_imgs["white"], labeled_imgs["yellow"], labeled_imgs["map"]])
        
        # Alles vertikal zusammenkleben (V-Stack)
        dashboard = cv2.vconcat([row1, row2])

        # Ein EINZIGES Fenster öffnen
        cv2.imshow("DuckieRace Control Center", dashboard)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = DashboardNode("dashboard_node")
    node.run()