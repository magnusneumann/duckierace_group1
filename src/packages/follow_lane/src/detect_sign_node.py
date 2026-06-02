#!/usr/bin/env python3
import os
import random
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from pupil_apriltags import Detector



class DetectSignNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # HIER LEGST DU NEUE IDs AN!
        # Aufbau: ID: ["name", ["richtung1", "richtung2"]]
        self.tag_rules = {
            1: ["X-all_directions", ["left", "straight", "right"]],
            2: ["T-left_or_right", ["left", "right"]],
            3: ["T-straight_or_left", ["straight", "left"]],
            4: ["T-straight_or_right", ["straight", "right"]],
            8: ["all_directions", ["left", "straight", "right"]],
            9: ["straight_or_right", ["straight", "right"]],
            10: ["T-left_or_right", ["left", "right"]],
        }
        
        self.current_decision = "straight"
        self.current_tag_id = "None"

        # Wir bauen für jede Familie einen eigenen Detektor auf
        self.detector_36h11 = Detector(
            families='tag36h11', 
            nthreads=1, 
            quad_decimate=1.0
        )
        
        self.detector_52h13 = Detector(
            families='tagStandard52h13', 
            nthreads=1, 
            quad_decimate=1.0
        )
        # Topics
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.pub_decision = rospy.Publisher(f"/{self._vehicle_name}/intersection/turn_decision", String, queue_size=1)
        rospy.Subscriber(camera_topic, CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)

    def cb_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        img_width = img.shape[1]

        # --- NEU: Beide Detektoren auf dasselbe Graustufenbild anwenden ---
        tags_36h11 = self.detector_36h11.detect(gray)
        tags_52h13 = self.detector_52h13.detect(gray)
        
        # Python macht es uns leicht: Wir addieren die Listen einfach!
        tags = tags_36h11 + tags_52h13
        # ----------------------------------------------------------------

        best_tag = None
        
        for tag in tags:
            # NUR TAGS AUF DER RECHTEN BILDHÄLFTE AKZEPTIEREN (x > width/2)
            if tag.center[0] > (img_width / 2):
                # Rahmen zeichnen
                ptA, ptB, ptC, ptD = [tuple(map(int, pt)) for pt in tag.corners]
                cv2.line(img, ptA, ptB, (0, 255, 0), 2)
                cv2.line(img, ptB, ptC, (0, 255, 0), 2)
                cv2.line(img, ptC, ptD, (0, 255, 0), 2)
                cv2.line(img, ptD, ptA, (0, 255, 0), 2)
                cv2.putText(img, f"ID: {tag.tag_id}", (int(tag.center[0]), int(tag.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                
                if tag.tag_id in self.tag_rules:
                    best_tag = tag

        # Wenn ein gültiger Tag rechts gefunden wurde, entscheide!
        if best_tag:
            self.current_tag_id = str(best_tag.tag_id)
            allowed = self.tag_rules[best_tag.tag_id][1]
            self.current_decision = random.choice(allowed)
            self.pub_decision.publish(String(data=self.current_decision))

        # Legende ins Bild malen (oben links)
        cv2.rectangle(img, (0,0), (400, 100), (0,0,0), -1)
        cv2.putText(img, f"Erkannte ID: {self.current_tag_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        if self.current_tag_id != "None":
            allowed_txt = ", ".join(self.tag_rules[int(self.current_tag_id)][1])
            cv2.putText(img, f"Erlaubt: {allowed_txt}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        
        cv2.putText(img, f"ENTSCHEIDUNG: {self.current_decision.upper()}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        cv2.imshow("Sign Detection (Rechte Seite)", img)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = DetectSignNode("detect_sign_node")
    node.run()