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
        results = self.model.predict(source=img, conf=0.5, verbose=False, imgsz=320)
        obstacle_msg = Polygon()

        # ACHTUNG: Diese Punkte müssen mit den util-Parametern aus lane_node übereinstimmen!
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        
        for box in results[0].boxes:
            if int(box.cls[0]) == self.duck_class_id:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w, h = x2 - x1, y2 - y1
                orig_cx = float(x1 + w / 2)
                orig_cy = float(y1 + h / 2)
                
                # Punkt in die Vogelperspektive transformieren
                original_pt = np.array([[[orig_cx, orig_cy]]], dtype=np.float32)
                warped_pt = cv2.perspectiveTransform(original_pt, M)
                new_cx = float(warped_pt[0][0][0])
                new_cy = float(warped_pt[0][0][1])
                
                radius = float(max(w, h) / 2 + self.safety_margin)
                obstacle_msg.points.append(Point32(x=new_cx, y=new_cy, z=radius))
                
                cv2.circle(img, (int(orig_cx), int(orig_cy)), int(radius), (0, 0, 255), -1)
        
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