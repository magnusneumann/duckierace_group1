#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Polygon, Point32
from ultralytics import YOLO
import util

class DetectDucksNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # YOLO Modell laden
        current_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(current_dir, "best.onnx")

        self.model = YOLO(weights_path)

        # Wir klauen uns die Parameter-Datei vom Lane-Node!
        util.init_parameters("detect_lane_node", self.cbUpdateParameters)
        self._crop_im_size = 400

        self.duck_class_id = 0
        self.safety_margin = 30 
        
        # --- NEU: Zähler für das Frame-Skipping ---
        self.frame_counter = 0

        # Publisher für die Hindernisse
        self.pub_duck_obstacles = rospy.Publisher(f"/{self._vehicle_name}/detect/duck_obstacles", Polygon, queue_size=1)
        
        # Publisher für unser Debug-Dashboard
        self.pub_debug_duck = rospy.Publisher(f"/{self._vehicle_name}/debug/duck_detection", CompressedImage, queue_size=1)

        image_topic = f"/{self._vehicle_name}/camera_node/image/compressed" #für Variante B
        
        rospy.Subscriber(image_topic, CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)
        rospy.loginfo("Duck Detection Node läuft (Wartet auf Bilder...)")

    def cbUpdateParameters(self, parameters):
        # Update perspective transform points für Variante B
        self.top_left_x = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

    def cb_image(self, msg):
        # --- FIX 1: Warten, bis die Parameter geladen wurden ---
        if not hasattr(self, 'top_left_x'):
            return  # Überspringen, bis util.init_parameters fertig ist

        # --- FIX 2: Frame-Skipping (Nur jedes 3. Bild verarbeiten) ---
        self.frame_counter += 1
        if self.frame_counter % 3 != 0:
            return  # CPU schonen, Bild sofort ignorieren
            
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Inferenz durchführen
        results = self.model.predict(source=img, task="detect", conf=0.5, verbose=False, imgsz=320)
        obstacle_msg = Polygon()

        # ACHTUNG: Diese Punkte müssen mit den util-Parametern aus lane_node übereinstimmen!
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        
        # 1. Filtern: Die Ente finden, die am weitesten links ist
        # 1. Filtern: Die Ente finden, die am weitesten links ist 
        # ABER: Nur Enten beachten, die nah am Roboter sind!
        leftmost_duck = None
        min_x = float('inf')
        
        # TUNING WERT: Ab welchem Y-Pixel (von oben gezählt) ist die Ente nah genug?
        # y=0 ist oben am Himmel, y=480 (oft Kamera-Max) ist direkt am Stoßfänger.
        # Da dein Crop bei ca. 218 anfängt, nehmen wir hier 200 als Grenze.
        y_threshold = 240 

        for box in results[0].boxes:
            if int(box.cls[0]) == self.duck_class_id:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # NEU: Tiefen-Filter! Wir prüfen die untere Kante (y2) der Bounding Box.
                # Ist die Ente weiter oben am Horizont als unser Threshold? Dann ignorieren!
                if y2 < y_threshold:
                    continue
                
                # Von den übrig gebliebenen Enten (nah dran) die linkeste suchen
                if x1 < min_x:
                    min_x = x1
                    leftmost_duck = (x1, y1, x2, y2)
        # 2. Dreieck berechnen (Spitze bei 400, 400)
        # 2. Bounding Box in die Vogelperspektive transformieren
        if leftmost_duck:
            orig_x1, orig_y1, orig_x2, orig_y2 = leftmost_duck
            
            # Alle 4 Ecken der Box definieren (Oben-Links, Oben-Rechts, Unten-Rechts, Unten-Links)
            pts_orig = np.array([
                [[orig_x1, orig_y1]], 
                [[orig_x2, orig_y1]], 
                [[orig_x2, orig_y2]], 
                [[orig_x1, orig_y2]]
            ], dtype=np.float32)
            
            # Alle Punkte auf einen Schlag transformieren
            warped_pts = cv2.perspectiveTransform(pts_orig, M)
            
            # Die 4 transformierten Punkte in unsere Message packen
            for i in range(4):
                p = Point32(x=int(warped_pts[i][0][0]), y=int(warped_pts[i][0][1]), z=0)
                obstacle_msg.points.append(p)
            
            # Fürs Dashboard: Die originale Box einzeichnen
            cv2.rectangle(img, (int(orig_x1), int(orig_y1)), (int(orig_x2), int(orig_y2)), (0, 0, 255), 2)
        self.pub_duck_obstacles.publish(obstacle_msg)

        # BILD AN DAS DASHBOARD SENDEN
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        self.pub_debug_duck.publish(msg_out)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = DetectDucksNode("detect_ducks_node")
    node.run()