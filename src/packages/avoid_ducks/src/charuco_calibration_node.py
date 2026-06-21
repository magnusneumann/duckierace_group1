#!/usr/bin/env python3
import os
import json
import yaml
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse

class CharucoCalibrationNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._v = os.environ.get('VEHICLE_NAME', 'gundel')

        # 1. Konfiguration laden
        config_path = '/root/DuckieRace/src/packages/avoid_ducks/config/charuco_config.json'
        rospy.loginfo(f"Lade Config von: {config_path}")
        with open(config_path, 'r') as f:
            self.cfg = json.load(f)

        # 2. ChArUco Board initialisieren
        dict_id = getattr(cv2.aruco, self.cfg["charuco"]["dictionary"])
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        
        sq_x = self.cfg["charuco"]["squares_x"]
        sq_y = self.cfg["charuco"]["squares_y"]
        sq_size = self.cfg["charuco"]["square_size_m"]
        mrk_size = self.cfg["charuco"]["marker_size_m"]

        try:
            self.board = cv2.aruco.CharucoBoard_create(sq_x, sq_y, sq_size, mrk_size, self.dictionary)
            self.board_points = self.board.chessboardCorners
        except AttributeError:
            self.board = cv2.aruco.CharucoBoard((sq_x, sq_y), sq_size, mrk_size, self.dictionary)
            self.board_points = self.board.getChessboardCorners()

        # 3. Variablen für Kamera-Parameter und GUI
        self.K = None
        self.D = None
        self.dist_model = None
        self.map1 = None
        self.map2 = None
        self.cam_width = None
        self.cam_height = None
        self.latest_charuco_corners = None
        self.latest_charuco_ids = None
        self.display_image = None # Bild für das OpenCV-Fenster

        # 4. Load camera intrinsics from YAML config (prefer this over CameraInfo topic)
        cam_info_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'my_camera_info.yaml')
        cam_info_path = os.path.abspath(cam_info_path)
        if os.path.exists(cam_info_path):
            try:
                with open(cam_info_path, 'r') as cf:
                    cam_data = yaml.safe_load(cf)
                if cam_data is not None and 'K' in cam_data and 'D' in cam_data:
                    K_list = cam_data.get('K')
                    D_list = cam_data.get('D')
                    K_arr = np.array(K_list, dtype=np.float64)
                    if K_arr.size == 9:
                        self.K = K_arr.reshape((3, 3))
                    else:
                        rospy.logwarn(f"Unexpected K size in {cam_info_path}: {K_arr.size}")
                    self.D = np.array(D_list, dtype=np.float64).flatten()
                    # choose default distortion model based on D length
                    if self.D.size == 4:
                        self.dist_model = 'fisheye'
                    else:
                        self.dist_model = 'plumb_bob'
                    rospy.loginfo(f"Camera intrinsics loaded from {cam_info_path} (using YAML values).")
                else:
                    rospy.logwarn(f"No K/D keys found in {cam_info_path}; will try CameraInfo topic instead.")
            except Exception as e:
                rospy.logwarn(f"Failed to read camera info from {cam_info_path}: {e}")
        else:
            rospy.loginfo(f"No local camera info YAML found at {cam_info_path}; will try CameraInfo topic.")

        # 5. Subscriber (only image; do NOT use CameraInfo topic since we prefer YAML)
        rospy.Subscriber(f"/{self._v}/camera_node/image/compressed", CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)
        
        # 5. Service bereitstellen
        rospy.Service(f"/{self._v}/calibration/save_homography", Trigger, self.handle_save_homography)
        
        rospy.loginfo("Charuco Calibration Node aktiv. Warte auf Kamera-Daten...")

    def cb_camera_info(self, msg):
        # Accept camera info once and prepare undistort maps later when we know image size
        if self.K is None:
            try:
                self.K = np.array(msg.K, dtype=np.float64).reshape((3, 3))
                self.D = np.array(msg.D, dtype=np.float64)
                self.dist_model = msg.distortion_model if hasattr(msg, 'distortion_model') else 'plumb_bob'
                self.cam_width = int(msg.width) if hasattr(msg, 'width') else None
                self.cam_height = int(msg.height) if hasattr(msg, 'height') else None
                rospy.loginfo("Kamera-Matrix (Intrinsics) erfolgreich empfangen.")
            except Exception as e:
                rospy.logwarn(f"Fehler beim Einlesen von CameraInfo: {e}")
                self.K = None
                self.D = None

    def cb_image(self, msg):
        # Ensure we have camera intrinsics. If not, try to load from config if present.
        if self.K is None or self.D is None:
            # Optional: allow specifying K/D in config JSON under keys 'camera_matrix' and 'dist_coeffs'
            cfg_cam = self.cfg.get('camera') or self.cfg.get('camera_matrix')
            if cfg_cam is not None:
                try:
                    self.K = np.array(cfg_cam.get('K') or cfg_cam.get('camera_matrix'), dtype=np.float64)
                    if self.K.size == 9:
                        self.K = self.K.reshape((3,3))
                    self.D = np.array(cfg_cam.get('D') or cfg_cam.get('dist_coeffs') or cfg_cam.get('dist'), dtype=np.float64)
                    rospy.loginfo('K/D aus Config geladen.')
                except Exception:
                    rospy.logwarn('K/D in Config konnten nicht geladen werden.')
            else:
                return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        h, w = img.shape[:2]

        # Initialize undistort maps once (uses image size)
        if self.map1 is None or self.map2 is None:
            try:
                K = self.K.astype(np.float64)
                D = self.D.astype(np.float64)

                # Detect fisheye-like distortion: CameraInfo.distortion_model or length of D==4
                is_fisheye = False
                if self.dist_model is not None and 'fisheye' in self.dist_model.lower():
                    is_fisheye = True
                elif D.size == 4:
                    is_fisheye = True

                if is_fisheye:
                    # Use OpenCV fisheye module
                    try:
                        newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)
                        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
                        rospy.loginfo('Fisheye-Undistort-Maps initialisiert.')
                    except Exception as e:
                        rospy.logwarn(f'Fehler beim Erstellen der fisheye maps: {e} -- fallback auf simple fisheye und remap')
                        # fallback: use direct undistortImage for each frame
                        self.map1 = None
                        self.map2 = None
                else:
                    # Standard pinhole model
                    newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
                    self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
                    rospy.loginfo('Pinhole-Undistort-Maps initialisiert.')
            except Exception as e:
                rospy.logwarn(f'Konnte Undistort-Maps nicht erstellen: {e}')

        # Apply undistortion
        undistorted = None
        try:
            if self.map1 is not None and self.map2 is not None:
                undistorted = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            else:
                # Fallbacks: try fisheye.undistortImage if D length is 4, otherwise use undistort
                if self.D.size == 4:
                    try:
                        undistorted = cv2.fisheye.undistortImage(img, self.K, self.D)
                    except Exception:
                        undistorted = img.copy()
                else:
                    try:
                        undistorted = cv2.undistort(img, self.K, self.D)
                    except Exception:
                        undistorted = img.copy()
        except Exception as e:
            rospy.logwarn(f'Fehler beim Undistort: {e}')
            undistorted = img.copy()

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=params)

        if ids is not None and len(corners) > 0:
            res, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
            
            if res > 0 and c_corners is not None:
                self.latest_charuco_corners = c_corners
                self.latest_charuco_ids = c_ids

                cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)
                cv2.aruco.drawDetectedCornersCharuco(undistorted, c_corners, c_ids, (0, 0, 255))
                cv2.putText(undistorted, f"Gefue Ecken: {len(c_corners)}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Bild an den Haupt-Thread übergeben
        self.display_image = undistorted

    def handle_save_homography(self, req):
        if self.latest_charuco_corners is None or len(self.latest_charuco_corners) < 4:
            return TriggerResponse(success=False, message="Fehler: Zu wenige Ecken erkannt.")

        img_pts = []
        robot_pts = []

        x_map = self.cfg["transformation"]["board_x_maps_to_robot"]
        y_map = self.cfg["transformation"]["board_y_maps_to_robot"]
        off_x = self.cfg["transformation"]["offset_x_m"]
        off_y = self.cfg["transformation"]["offset_y_m"]

        for c_id, c_coord in zip(self.latest_charuco_ids.flatten(), self.latest_charuco_corners):
            img_pts.append(c_coord[0]) 

            bx = self.board_points[c_id][0]
            by = self.board_points[c_id][1]

            rx = off_x
            ry = off_y

            if x_map == "X": rx += bx
            elif x_map == "-X": rx -= bx
            elif x_map == "Y": ry += bx
            elif x_map == "-Y": ry -= bx

            if y_map == "X": rx += by
            elif y_map == "-X": rx -= by
            elif y_map == "Y": ry += by
            elif y_map == "-Y": ry -= by

            robot_pts.append([rx, ry])

        img_pts = np.array(img_pts, dtype=np.float32)
        robot_pts = np.array(robot_pts, dtype=np.float32)

        H, _ = cv2.findHomography(robot_pts, img_pts, cv2.RANSAC, 5.0)

        if H is not None:
            output_path = self.cfg["output"]["yaml_path"]
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            yaml_data = {"homography": H.flatten().tolist(), "shape": [3, 3]}
            with open(output_path, 'w') as y_file:
                yaml.dump(yaml_data, y_file, default_flow_style=False)

            rospy.loginfo(f"Erfolgreich gespeichert unter: {output_path}")
            return TriggerResponse(success=True, message="Matrix erfolgreich exportiert.")
        
        return TriggerResponse(success=False, message="Fehler bei H-Berechnung.")

    def run(self):
        # HIER PASSIERT DAS RENDERN IM MAIN-THREAD (Thread-Safe)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.display_image is not None:
                cv2.imshow("Charuco Kalibrierung (Live)", self.display_image)
                cv2.waitKey(1)
            rate.sleep()
        
        # Wenn Knoten beendet wird, Fenster sauber schließen
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = CharucoCalibrationNode("charuco_calibration_node")
    node.run()