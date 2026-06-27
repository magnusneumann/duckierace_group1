#!/usr/bin/env python3
import os
import random
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from pupil_apriltags import Detector

class DetectTagsNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # IDs 1-4: Kreuzungen (Lenk-Entscheidungen treffen)
        self.tag_rules = {
            1: ["X-all_directions", ["left", "straight", "right"]],
            2: ["T-left_or_right", ["left", "right"]],
            3: ["T-straight_or_left", ["straight", "left"]],
            4: ["T-straight_or_right", ["straight", "right"]],
        }
        
        # IDs 5-10: Kanten-Marker (Werden NUR an den Mapping-Node gesendet)
        self.edge_tags = {5, 6, 7, 8, 9, 10}
        
        self.current_decision = "straight"
        self.current_tag_id = "None"
        self.min_tag_area = 600  
        self.frame_counter = 0
        
        # Bild für die Anzeige zwischenspeichern
        self.display_image = None
        
        # Detektoren initialisieren
        #self.detector_36h11 = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)
        self.detector_52h13 = Detector(families='tagStandard52h13', nthreads=1, quad_decimate=1.0)
        
        # Publisher anlegen
        self.pub_decision = rospy.Publisher(f"/{self._vehicle_name}/intersection/turn_decision", String, queue_size=1)
        self.pub_tag_id = rospy.Publisher(f"/{self._vehicle_name}/detect/tag_id", String, queue_size=1)
        
        # Kamera abonnieren
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        rospy.Subscriber(camera_topic, CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)

        # Sicherstellen, dass Fenster beim Beenden zerstört werden
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        rospy.loginfo("Fahre Tags Detection Node sicher herunter...")
        cv2.destroyAllWindows()

    def cb_image(self, msg):
        # Frame-Skipping (spart CPU, nimmt nur jedes 3. Bild)
        if self.frame_counter <= 1:
            self.frame_counter += 1
            return
        self.frame_counter = 0
        
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        img_width = img.shape[1]

        # Beide Tag-Familien suchen
        #tags_36h11 = self.detector_36h11.detect(gray)
        tags_52h13 = self.detector_52h13.detect(gray)
        tags = tags_52h13 #tags_36h11 + 

        best_tag = None
        max_area = 0

        for tag in tags:
            # Nur Tags auf der rechten Bildhälfte beachten
            if tag.center[0] > (img_width / 2):
                area = cv2.contourArea(np.array(tag.corners, dtype=np.float32))
                
                # Tag-Rahmen zeichnen
                ptA, ptB, ptC, ptD = [tuple(map(int, pt)) for pt in tag.corners]
                cv2.line(img, ptA, ptB, (0, 255, 0), 2)
                cv2.line(img, ptB, ptC, (0, 255, 0), 2)
                cv2.line(img, ptC, ptD, (0, 255, 0), 2)
                cv2.line(img, ptD, ptA, (0, 255, 0), 2)
                cv2.putText(img, f"ID: {tag.tag_id}", (int(tag.center[0]), int(tag.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                
                is_intersection = tag.tag_id in self.tag_rules
                is_edge = tag.tag_id in self.edge_tags
                
                # Tag ist valide, wenn es entweder eine Kreuzung oder ein Marker ist und groß genug ist
                if (is_intersection or is_edge) and area > max_area and area > self.min_tag_area:
                    best_tag = tag
                    max_area = area
        
        if best_tag:
            self.current_tag_id = str(best_tag.tag_id)
            
            # 1. IMMER die Tag-ID an den Mapping-Node senden (Für Kanten oder Knoten)
            self.pub_tag_id.publish(String(data=self.current_tag_id))

            # 2. NUR bei Kreuzungs-Tags eine Lenk-Entscheidung publishen
            if best_tag.tag_id in self.tag_rules:
                allowed = self.tag_rules[best_tag.tag_id][1]
                self.current_decision = random.choice(allowed)
                self.pub_decision.publish(String(data=self.current_decision))
            else:
                self.current_decision = "none (edge marker)"

        # --- Anzeige (Overlay im Bild) ---
        cv2.rectangle(img, (0,0), (450, 100), (0,0,0), -1)
        cv2.putText(img, f"Erkannte ID: {self.current_tag_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if self.current_tag_id != "None":
            tag_int = int(self.current_tag_id)
            if tag_int in self.tag_rules:
                allowed_txt = ", ".join(self.tag_rules[tag_int][1])
                cv2.putText(img, f"Erlaubt: {allowed_txt}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            elif tag_int in self.edge_tags:
                cv2.putText(img, f"Info: Kanten-Marker (Keine Lenkung)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        
        cv2.putText(img, f"ENTSCHEIDUNG: {self.current_decision.upper()}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        # WICHTIG: Hier kein imshow / waitKey mehr! Nur als Display-Variable speichern:
        self.display_image = img

    def run(self):
        cv2.namedWindow("Tags Detection (Rechte Seite)", cv2.WINDOW_NORMAL)
        rate = rospy.Rate(10)
        
        # Das Zeichnen passiert jetzt blockierungsfrei im Main-Thread (Schützt vor Abstürzen!)
        while not rospy.is_shutdown():
            if self.display_image is not None:
                cv2.imshow("Tags Detection (Rechte Seite)", self.display_image)
                
                # Lauscht auf Tastatur-Eingaben im OpenCV Fenster
                key = cv2.waitKey(50) & 0xFF
                if key == ord('q') or key == 27:
                    rospy.signal_shutdown("Benutzer hat das Fenster geschlossen.")
                    
            rate.sleep()

if __name__ == "__main__":
    node = DetectTagsNode("detect_tags_node")
    node.run()