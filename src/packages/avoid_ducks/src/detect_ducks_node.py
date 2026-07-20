#!/usr/bin/env python3
import os
import cv2
import yaml
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Polygon, Point32
from ultralytics import YOLO

class DetectDucksNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "gundel")

        # --- YOLO MODELL LADEN ---
        current_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(current_dir, "best.onnx")
        self.model = YOLO(weights_path)

        self.duck_class_id = 0
        self.frame_counter = 0

        # --- KAMERA & ENTZERRUNG LADEN ---
        self.path_intrinsics = '/root/DuckieRace/src/packages/avoid_ducks/config/my_camera_info.yaml'
        self.K = None
        self.D = None
        self.map1 = None
        self.map2 = None
        self._load_intrinsics()

        # Publisher
        self.pub_duck_obstacles = rospy.Publisher(f"/{self._vehicle_name}/detect/duck_obstacles", Polygon, queue_size=1)
        self.pub_debug_duck = rospy.Publisher(f"/{self._vehicle_name}/debug/duck_detection", CompressedImage, queue_size=1)

        # Subscriber
        image_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        rospy.Subscriber(image_topic, CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)
        
        rospy.loginfo("Duck Detection Node läuft (Entzerrung aktiv...)")

    def _load_intrinsics(self):
        try:
            with open(self.path_intrinsics, 'r') as f:
                data = yaml.safe_load(f)
                self.K = np.array(data['K']).reshape((3,3))
                self.D = np.array(data['D'])
            rospy.loginfo("Intrinsics für Duck-Detection erfolgreich geladen.")
        except Exception as e:
            rospy.logerr(f"Konnte Intrinsics nicht laden: {e}")

    def cb_image(self, msg):
        # Frame-Skipping (Nur jedes 3. Bild verarbeiten, um CPU zu schonen)
        self.frame_counter += 1
        if self.frame_counter % 2 != 0:
            return  
            
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = img.shape[:2]

        # ==========================================
        # 1. BILD ENTZERREN (Pre-Processing)
        # ==========================================
        if self.map1 is None or self.map2 is None:
            try:
                K = self.K.astype(np.float64)
                D = self.D.astype(np.float64)
                
                # Wir gehen vom Fisheye-Modell aus (D.size == 4)
                if D.size == 4:
                    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)
                    self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
                else:
                    newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
                    self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
            except Exception as e:
                rospy.logwarn(f"Map Init Fehler: {e}")

        undistorted = img.copy()
        if self.map1 is not None and self.map2 is not None:
            undistorted = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        elif self.D is not None and self.D.size == 4:
            try:
                undistorted = cv2.fisheye.undistortImage(img, self.K, self.D)
            except Exception:
                pass

        # ==========================================
        # 2. INFERENZ AUF ENTZERRTEM BILD
        # ==========================================
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        results = self.model.predict(source=gray, task="detect", conf=0.5, verbose=False, imgsz=640)
        obstacle_msg = Polygon()
        
        # TUNING WERT: y=0 ist Himmel, y=480 ist Stoßfänger.
        y_threshold = 240 

        for box in results[0].boxes:
            if int(box.cls[0]) == self.duck_class_id:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Tiefen-Filter: Ignoriere Enten am Horizont
                if y2 < y_threshold:
                    continue
                
                # Wir packen die 4 Eckpunkte der Bounding Box direkt in die Message
                # Reihenfolge: Oben-Links, Oben-Rechts, Unten-Rechts, Unten-Links
                pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
                
                for px, py in pts:
                    p = Point32(x=px, y=py, z=0)
                    obstacle_msg.points.append(p)
                
                # Bounding Box ins Debug-Bild einzeichnen (Grün für valide Enten)
                cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(undistorted, "DUCK", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Nachricht senden (Auch wenn sie leer ist, damit alte Enten aus dem Speicher gelöscht werden)
        self.pub_duck_obstacles.publish(obstacle_msg)

        # ==========================================
        # 3. DEBUG BILD SENDEN
        # ==========================================
        # Zeige den y_threshold als rote Linie zur visuellen Kontrolle
        cv2.line(undistorted, (0, y_threshold), (w, y_threshold), (0, 0, 255), 1)
        
        #msg_out = CompressedImage()
        #msg_out.header.stamp = rospy.Time.now()
        #msg_out.format = "jpeg"
        #msg_out.data = np.array(cv2.imencode('.jpg', undistorted)[1]).tobytes()
        #self.pub_debug_duck.publish(msg_out)
        

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = DetectDucksNode("detect_ducks_node")
    node.run()