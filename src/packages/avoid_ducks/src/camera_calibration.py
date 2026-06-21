#!/usr/bin/env python3
import os
import json
import yaml
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

class AutoCameraCalibrationNode:
    def __init__(self):
        rospy.init_node("camera_calibration_node")
        self._v = os.environ.get('VEHICLE_NAME', 'gundel')

        # 1. Config laden (Hardcoded)
        config_path = '/root/DuckieRace/src/packages/avoid_ducks/config/charuco_config.json'
        with open(config_path, 'r') as f:
            self.cfg = json.load(f)

        # ChArUco Setup
        dict_id = getattr(cv2.aruco, self.cfg["charuco"]["dictionary"])
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        
        sq_x = self.cfg["charuco"]["squares_x"]
        sq_y = self.cfg["charuco"]["squares_y"]
        sq_size = self.cfg["charuco"]["square_size_m"]
        mrk_size = self.cfg["charuco"]["marker_size_m"]

        try:
            self.board = cv2.aruco.CharucoBoard_create(sq_x, sq_y, sq_size, mrk_size, self.dictionary)
        except AttributeError:
            self.board = cv2.aruco.CharucoBoard((sq_x, sq_y), sq_size, mrk_size, self.dictionary)

        # Variablen
        self.all_corners = []
        self.all_ids = []
        self.img_size = None
        self.display_image = None
        self.last_sample_time = 0.0
        self.calibration_done = False  # Schutz-Flag gegen mehrfaches Triggern

        # Target-Anzahl der Bilder
        self.target_samples = 45

        # Subscriber
        rospy.Subscriber(f"/{self._v}/camera_node/image/compressed", CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"=== Automatischer Datensammler aktiv ===")
        rospy.loginfo(f"Bewege das Board vor der Kamera. Berechnung startet automatisch bei {self.target_samples} Bildern.")

    def cb_image(self, msg):
        if self.calibration_done:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.img_size is None:
            self.img_size = (img.shape[1], img.shape[0])

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=params)

        current_time = rospy.get_time()

        if ids is not None and len(corners) > 0:
            try:
                res, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
                
                if res is not None and res > 5 and c_corners is not None and c_ids is not None: 
                    if len(c_corners) == len(c_ids):
                        cv2.aruco.drawDetectedCornersCharuco(img, c_corners, c_ids, (0, 0, 255))
                    
                        # Max. 2 Samples pro Sekunde
                        if (current_time - self.last_sample_time) > 2.5:
                            self.all_corners.append(c_corners)
                            self.all_ids.append(c_ids)
                            self.last_sample_time = current_time
                            rospy.loginfo(f"[SAVE] Sample {len(self.all_corners)}/{self.target_samples} gespeichert!")

                            # AUTOMATISCHER TRIGGER BEI 45 BILDERN
                            if len(self.all_corners) >= self.target_samples:
                                self.calibration_done = True
                                self.run_calibration_math()
                                return
            except Exception as e:
                rospy.logdebug(f"Frame uebersprungen: {str(e)}")

        cv2.putText(img, f"Samples: {len(self.all_corners)} / {self.target_samples}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.display_image = img

    def run_calibration_math(self):
        rospy.loginfo("!!! 45 Samples erreicht. Starte automatische Berechnung !!!")
        rospy.loginfo("Bitte warten, OpenCV rechnet...")
        
        try:
            ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                charucoCorners=self.all_corners,
                charucoIds=self.all_ids,
                board=self.board,
                imageSize=self.img_size,
                cameraMatrix=None,
                distCoeffs=None
            )

            if ret:
                output_path = "/root/DuckieRace/src/packages/avoid_ducks/config/my_camera_info.yaml"
                os.makedirs(os.path.dirname(output_path), exist_ok=True)
                
                yaml_data = {
                    "K": camera_matrix.flatten().tolist(),
                    "D": distortion_coefficients.flatten().tolist()
                }
                with open(output_path, 'w') as f:
                    yaml.dump(yaml_data, f, default_flow_style=False)
                
                rospy.loginfo("=========================================================")
                rospy.loginfo(f"ERFOLG! Intrinsics exportiert nach: {output_path}")
                rospy.loginfo("Der Knoten beendet sich jetzt selbststaendig.")
                rospy.loginfo("=========================================================")
                
                # Signalisiert dem Hauptthread das Ende
                rospy.signal_shutdown("Kalibrierung erfolgreich beendet.")
            else:
                rospy.logerr("OpenCV hat die Kalibrierung abgewiesen (Fehler im Algorithmus).")
                self.calibration_done = False # Erlaubt neuen Versuch
        except Exception as e:
            rospy.logerr(f"Fehler bei der Berechnung: {str(e)}")
            self.calibration_done = False

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.display_image is not None and not self.calibration_done:
                cv2.imshow("Intrinsische Datensammlung", self.display_image)
            
            cv2.waitKey(1)
            rate.sleep()
            
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = AutoCameraCalibrationNode()
    node.run()