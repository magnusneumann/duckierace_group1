#!/usr/bin/env python3
import os
import json
import yaml
import rospy
import cv2
import numpy as np
import math
from shapely.geometry import Polygon as ShapelyPolygon, box as ShapelyBox
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Polygon as RosPolygon
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
import time
from collections import deque


class DuckAvoidanceNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._v = os.environ.get('VEHICLE_NAME', 'gundel')
        
        # --- KONFIGURATIONSPFHADE (Hier hart eintragen) ---
        self.path_hsv = '/root/DuckieRace/src/packages/avoid_ducks/config/detect_lane_node.json'
        self.path_homography = '/root/DuckieRace/src/packages/avoid_ducks/config/homography.yaml'
        self.path_intrinsics = '/root/DuckieRace/src/packages/avoid_ducks/config/my_camera_info.yaml'
        
        # --- KAMERA & ENTZERRUNG ---
        self.map1 = None
        self.map2 = None
        self.K = None
        self.D = None
        self.dist_model = "pinhole"
        self._load_intrinsics()
        
        # --- HOMOGRAPHIE & TRAPEZE ---
        self.H = None
        self.trapezoids_2d = [] # Pixel-Koordinaten (Numpy für cv2)
        self.trapezoids_shapely = [] # Shapely Polygone für Intersection
        self._load_homography_and_build_zones()

        # --- MASKEN PARAMETER ---
        self.hsv_limits = {}
        self._load_hsv_config()
        self.pixel_threshold = 100 # Ab wann gilt ein Trapez als durch Linien blockiert?

        # --- ODOMETRIE ---
        self.theta = 0.0
        self.ticks_left = None
        self.ticks_right = None
        self.R = 0.0318
        self.N = 135
        self.L = 0.10
        self.start_theta = 0.0 # Für Bias-Speicherung
        # Position (meters) for simple dead-reckoning
        self.x = 0.0
        self.y = 0.0
        # Drive-for-distance helper
        self._drive_target_distance = None
        self._drive_start_pose = None
        self._drive_speed = 0.15

        # --- ZUSTAND & PERCEPTION ---
        self.state = "DRIVING" # DRIVING, ROTATING
        self.zones_status = [{"white": False, "yellow": False, "duck": False} for _ in range(3)] # Status für Zone 1, 2, 3
        self.duck_bboxes = [] # Eingehende Enten [(x1,y1,x2,y2), ...]
        self.display_image = None
        self.buffer_size = 17
        # Puffer exklusiv für Zone 2
        self.z2_yellow_history = deque(maxlen=self.buffer_size)
        self.wiggle_direction = -1.0
        self.last_wiggle_time = time.monotonic()
        self.wiggle_power = 0.065
        # Tracking für Rotationsursache und Inversions-Schutz
        self.rotation_reason = None
        # use monotonic wall-clock for inversion cooldown (robust to /use_sim_time)
        self.last_inversion_time = time.monotonic()


        # --- ROS INTERFACES ---
        self.pub_cmd = rospy.Publisher(f"/{self._v}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.pub_debug = rospy.Publisher(f"/{self._v}/debug/avoidance_view/compressed", CompressedImage, queue_size=1)
        
        rospy.Subscriber(f"/{self._v}/camera_node/image/compressed", CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber(f"/{self._v}/detect/duck_obstacles", RosPolygon, self.cb_ducks)
        rospy.Subscriber(f"/{self._v}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_left)
        rospy.Subscriber(f"/{self._v}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_right)

        rospy.loginfo("Duck Avoidance Node initialisiert.")
        
        # Ensure motors are stopped on shutdown
        rospy.on_shutdown(self._on_shutdown)

    # ==========================================
    # 1. INITIALISIERUNG & SETUP
    # ==========================================
    def _load_intrinsics(self):
        try:
            with open(self.path_intrinsics, 'r') as f:
                data = yaml.safe_load(f)
                self.K = np.array(data['K']).reshape((3,3))
                self.D = np.array(data['D'])
            rospy.loginfo("Intrinsics geladen.")
        except Exception as e:
            rospy.logwarn(f"Intrinsics Fehler: {e}")

    def _ensure_undistort_maps(self, w, h):
        """Initialize undistort maps for image size (w,h) if not already set.
        Uses self.K and self.D loaded from YAML. If initialization fails, maps
        remain None and callers should fallback to per-frame undistort.
        """
        if self.map1 is not None and self.map2 is not None:
            return
        if self.K is None or self.D is None:
            return
        try:
            K = self.K.astype(np.float64)
            D = self.D.astype(np.float64)
            # choose fisheye when 4 coefficients present
            if D.size == 4:
                try:
                    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)
                    self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
                except Exception as e:
                    rospy.logwarn(f"Fisheye map init failed: {e}")
                    self.map1 = None
                    self.map2 = None
            else:
                newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
                self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
        except Exception as e:
            rospy.logwarn(f"Failed to create undistort maps: {e}")
            self.map1 = None
            self.map2 = None

    def _load_homography_and_build_zones(self):
        try:
            with open(self.path_homography, 'r') as f:
                data = yaml.safe_load(f)
                self.H = np.array(data['homography']).reshape((3,3))
            rospy.loginfo("Homographie geladen.")
        except Exception as e:
            rospy.logerr(f"Homographie Fehler: {e}")
            return

        # Zonen in echten Metern (X = Vorne, Y = Links/Rechts)
        # 15cm lang, 20cm breit (-0.1 bis 0.1)
        zones_3d = [
            [(0.1, -0.06), (0.14, -0.06), (0.14, 0.06), (0.1, 0.06)],   # Zone 0
            [(0.14, -0.08), (0.2, -0.073), (0.2, 0.073), (0.14, 0.08)],   # Zone 1
            [(0.2, -0.073), (0.3, -0.07), (0.3, 0.07), (0.2, 0.073)]       # Zone 2
            #[(0.30, -0.07), (0.42, -0.07), (0.42, 0.07), (0.30, 0.07)]      # Zone 3
        ]

        for z in zones_3d:
            pts_2d = []
            for (X, Y) in z:
                # Projektion: H * [X, Y, 1]^T
                vec = np.array([X, Y, 1.0])
                proj = np.dot(self.H, vec)
                u = int(proj[0] / proj[2])
                v = int(proj[1] / proj[2])
                pts_2d.append([u, v])
            
            pts_2d = np.array(pts_2d, dtype=np.int32)
            self.trapezoids_2d.append(pts_2d)
            self.trapezoids_shapely.append(ShapelyPolygon(pts_2d))

    def _load_hsv_config(self):
        try:
            with open(self.path_hsv, 'r') as f:
                self.hsv_limits = json.load(f)
            rospy.loginfo("HSV Config geladen.")
        except Exception as e:
            rospy.logwarn(f"HSV Config Fehler, nutze Fallback: {e}")
            self.hsv_limits = {
                "white": {"lower": [0, 0, 150], "upper": [180, 50, 255]},
                "yellow": {"lower": [20, 100, 100], "upper": [40, 255, 255]}
            }

    # ==========================================
    # 2. ODOMETRIE & EINGANGSDATEN
    # ==========================================
    def cb_left(self, msg):
        if self.ticks_left is None: self.ticks_left = msg.data
        dt = msg.data - self.ticks_left
        self.ticks_left = msg.data
        self.update_odometry(dt, 0)

    def cb_right(self, msg):
        if self.ticks_right is None: self.ticks_right = msg.data
        dt = msg.data - self.ticks_right
        self.ticks_right = msg.data
        self.update_odometry(0, dt)

    def update_odometry(self, dl_ticks, dr_ticks):
        # convert ticks to wheel travel (meters)
        dl = (dl_ticks / self.N) * (2 * math.pi * self.R)
        dr = (dr_ticks / self.N) * (2 * math.pi * self.R)
        d = 0.5 * (dr + dl)
        dth = (dr - dl) / self.L

        # update pose with a midpoint-heading approximation
        if abs(dth) < 1e-6:
            self.x += d * math.cos(self.theta)
            self.y += d * math.sin(self.theta)
        else:
            theta_mid = self.theta + dth * 0.5
            self.x += d * math.cos(theta_mid)
            self.y += d * math.sin(theta_mid)

        self.theta += dth

    def cb_ducks(self, msg):
        # Extrahiere Boxen aus dem Polygon-Array (erwartet x1,y1,x2,y2 pro Ente)
        self.duck_bboxes = []
        if len(msg.points) >= 4:
            for i in range(0, len(msg.points), 4):
                # Min/Max bestimmen für sauberes Rect
                xs = [msg.points[i+j].x for j in range(4)]
                ys = [msg.points[i+j].y for j in range(4)]
                self.duck_bboxes.append((int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))))

    # ==========================================
    # 3. VISION & PERCEPTION LOOP
    # ==========================================
    def cb_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = img.shape[:2]

        # Undistort (lean): initialize maps once and remap, fallback to per-frame undistort
        self._ensure_undistort_maps(w, h)
        if self.map1 is not None and self.map2 is not None:
            undistorted = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        else:
            try:
                if self.D is not None and self.D.size == 4:
                    undistorted = cv2.fisheye.undistortImage(img, self.K, self.D)
                else:
                    undistorted = cv2.undistort(img, self.K, self.D)
            except Exception:
                undistorted = img.copy()

        if not self.trapezoids_2d:
            return # Ohne Trapeze keine Perception

        # 1. HSV Konvertierung & Basis-Masken
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, np.array(self.hsv_limits['white']['lower']), np.array(self.hsv_limits['white']['upper']))
        mask_yellow = cv2.inRange(hsv, np.array(self.hsv_limits['yellow']['lower']), np.array(self.hsv_limits['yellow']['upper']))

        # 2. Enten aus der Gelb-Maske stanzen (manipulieren)
        for (x1, y1, x2, y2) in self.duck_bboxes:
            cv2.rectangle(mask_yellow, (x1, y1), (x2, y2), 0, -1) # Setzt BBox-Bereich in Maske auf 0 (Schwarz)
            cv2.rectangle(undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2) # Grün im Debug-Bild

        # Kombinierte Linien-Maske
        mask_lines = cv2.bitwise_or(mask_white, mask_yellow)

        # 3. Detaillierte Zonen evaluieren
        self.zones_status = [{"white": False, "yellow": False, "duck": False} for _ in range(3)]
        
        for i, (trap_pts, trap_poly) in enumerate(zip(self.trapezoids_2d, self.trapezoids_shapely)):
            # A) Enten-Kollision
            for (x1, y1, x2, y2) in self.duck_bboxes:
                if trap_poly.intersects(ShapelyBox(x1, y1, x2, y2)):
                    self.zones_status[i]["duck"] = True
                    break

            # B) Masken separiert prüfen
            trap_mask = np.zeros_like(mask_white)
            cv2.fillPoly(trap_mask, [trap_pts], 255)
            
            # Rohe Pixel in der aktuellen Maske zählen
            raw_white = cv2.countNonZero(cv2.bitwise_and(mask_white, trap_mask))
            raw_yellow = cv2.countNonZero(cv2.bitwise_and(mask_yellow, trap_mask))
            
            # --- HYBRID-FILTER LOGIK (Jetzt mit Median für Gelb) ---
            if i == 2: 
                # Zone 2 Gelb: In den Puffer schieben und Median berechnen
                self.z2_yellow_history.append(raw_yellow)
                eval_yellow = np.median(self.z2_yellow_history)
            else:
                # Zone 0, 1 und 3 Gelb: Harte Echtzeit
                eval_yellow = raw_yellow

            # Weiß läuft in ALLEN Zonen direkt in Echtzeit durch
            eval_white = raw_white

            # Entscheidung anhand der evaluierten Werte treffen
            if eval_white > self.pixel_threshold:
                self.zones_status[i]["white"] = True
                undistorted[cv2.bitwise_and(mask_white, trap_mask) > 0] = (255, 0, 0)
                
            if eval_yellow > self.pixel_threshold:
                self.zones_status[i]["yellow"] = True
                undistorted[cv2.bitwise_and(mask_yellow, trap_mask) > 0] = (0, 165, 255)
            # Weiß (Rechte Grenze -> Zwingt uns nach Links)
            pixels_white = cv2.countNonZero(cv2.bitwise_and(mask_white, trap_mask))
            if pixels_white > self.pixel_threshold:
                self.zones_status[i]["white"] = True
                undistorted[cv2.bitwise_and(mask_white, trap_mask) > 0] = (255, 0, 0) # Blau für Weiß-Erkennung
                
            # Gelb (Linke Grenze -> Zwingt uns nach Rechts)
            pixels_yellow = cv2.countNonZero(cv2.bitwise_and(mask_yellow, trap_mask))
            if pixels_yellow > self.pixel_threshold:
                self.zones_status[i]["yellow"] = True
                undistorted[cv2.bitwise_and(mask_yellow, trap_mask) > 0] = (0, 165, 255) # Orange für Gelb-Erkennung

            # C) Debug Rahmen zeichnen
            if self.zones_status[i]["white"] or self.zones_status[i]["yellow"] or self.zones_status[i]["duck"]:
                cv2.polylines(undistorted, [trap_pts], isClosed=True, color=(0, 0, 255), thickness=2) # Rot = Gefahr
            else:
                cv2.polylines(undistorted, [trap_pts], isClosed=True, color=(0, 255, 255), thickness=2) # Gelb = Frei
        
        # Bild an den Main-Thread für das OpenCV Fenster übergeben
        self.display_image = undistorted

    # ==========================================
    # 4. CONTROL LOOP & FSM
    # ==========================================
    def run(self):
        rate = rospy.Rate(10)
        IMAGE_CENTER_X = 320 
        # store monotonic timestamp for inversion cooldown
        self.last_inversion_time = time.monotonic()
        
        while not rospy.is_shutdown():
            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()

            z0 = self.zones_status[0]
            z1 = self.zones_status[1]
            z2 = self.zones_status[2]
            #z3 = self.zones_status[3]

            # --- HILFSFUNKTION: Größte Ente analysieren ---
            duck_center_x = IMAGE_CENTER_X
            if self.duck_bboxes:
                largest_duck = max(self.duck_bboxes, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
                duck_center_x = (largest_duck[0] + largest_duck[2]) / 2.0

            # ==========================================================
            # PHASE 1: ZUSTANDS-WECHSEL (Entscheidungen treffen)
            # ==========================================================
            
            # Trigger A: Linie in Zone 0
            if z0["white"] or z0["yellow"]:
                if self.state != "ROTATING":
                    rospy.loginfo("Wegen Linie in Zone 0. Rotieren")
                    self.state = "ROTATING"
                    self.rotation_reason = "line"
                    if z0 ["white"]:
                        self.escape_direction = 1.0
                        rospy.loginfo("Rotation nach links läuft...")
                    if z0["yellow"]:
                        self.escape_direction = -1.0
                        rospy.loginfo("Rotation nach rechts läuft...")
                    
            # Trigger B: Ente in Zone 1
            elif z1["duck"]:
                if self.state != "ROTATING":
                    rospy.loginfo("Ente in Z1! Freien Fahrkorridor finden.")
                    self.state = "ROTATING"
                    self.rotation_reason = "duck"
                    self.escape_direction = -1.0 if duck_center_x < IMAGE_CENTER_X else 1.0

            # Trigger C: Abbruch der Rotation (Alles frei)
            elif self.state == "ROTATING" and not z1["duck"] and not z2["duck"] and not z1["yellow"] and not z1["white"]:
                if self.rotation_reason == "duck":
                    rospy.loginfo("Korridor frei. An Ente vorbei fahren.")
                    self._drive_target_distance = 0.15
                    self._drive_start_pose = (self.x, self.y)
                    self.state = "DRIVE_FORWARD_DISTANCE"
                else:
                    rospy.loginfo("Linienrotation abgeschlossen. Weiterfahren.")
                    self.state = "DRIVING"


            # ==========================================================
            # PHASE 2: MOTOR-AKTIONEN (Ausführen, was der Zustand sagt)
            # WICHTIG: Das hier sind NEUE 'if'-Blöcke, keine 'elif' mehr!
            # ==========================================================
            
            if self.state == "ROTATING":
                current_time = time.monotonic()
                #vor und zurück setzen um Rollmoment zu überwinden
                if current_time - self.last_wiggle_time > 0.05:
                    self.wiggle_direction *= -1.0
                    self.last_wiggle_time = current_time
                cmd.v = 1.0*self.wiggle_power*self.wiggle_direction

                # INVERTIERUNGS-SCHUTZ: Läuft jetzt JEDEN Frame, 
                # egal ob die Ente noch in Zone 1 ist oder nicht!
                dt = current_time - self.last_inversion_time
                rospy.logdebug(f"Inversion check dt={dt:.3f}s, escape={self.escape_direction}")
                if dt > 1.0: 
                    if self.escape_direction == 1.0 and z2["yellow"]:
                        rospy.logwarn("Linksdrehung wegen GELB auf RECHTS wechseln")
                        self.escape_direction = -1.0
                        self.last_inversion_time = current_time
                    
                    elif self.escape_direction == -1.0 and z2["white"]:
                        rospy.logwarn("Rechtsdrehung wegen WEISS auf LINKS wechseln")
                        self.escape_direction = 1.0
                        self.last_inversion_time = current_time

                cmd.omega = 1.6 * self.escape_direction

            elif self.state == "DRIVING":
                # Spurkorrektur in Zone 1
                if (z1["white"] or z1["yellow"]) and not z1["duck"]:
                    cmd.v = 0.15
                    if z1["white"]:
                        cmd.omega = 2.7
                    elif z1["yellow"]:
                        cmd.omega = -2.7
                
                # Zone 2: Drosseln und Ausweichen
                elif z2["duck"]:
                    cmd.v = 0.11 
                    if z2["white"]:
                        cmd.omega = 1.2 
                    elif z2["yellow"]:
                        cmd.omega = -1.2 
                    else:
                        if duck_center_x < IMAGE_CENTER_X:
                            cmd.omega = 1.3 
                        else:
                            cmd.omega = -1.3  
                else:
                    cmd.v = 0.15 
                    cmd.omega = 0.0

            elif self.state == "DRIVE_FORWARD_DISTANCE":
                if self._drive_start_pose is None:
                    self._drive_start_pose = (self.x, self.y)
                cmd.v = 0.15
                cmd.omega = 0.0
                
                if self._drive_start_pose is not None and self._drive_target_distance is not None:
                    dx = self.x - self._drive_start_pose[0]
                    dy = self.y - self._drive_start_pose[1]
                    traveled = math.hypot(dx, dy)
                    if traveled >= self._drive_target_distance:
                        rospy.loginfo(f"Ente passiert, {traveled:.3f} m gefahren.")
                        self._drive_target_distance = None
                        self._drive_start_pose = None
                        self.state = "DRIVING"
                        cmd.v = 0.0
                        cmd.omega = 0.0

            # --- COMMAND PUBLISH ---
            try:
                self.pub_cmd.publish(cmd)
            except Exception:
                rospy.logwarn("Failed to publish cmd")
                
            # --- DEBUG BILD AKTUALISIEREN & ANZEIGEN ---
            if self.display_image is not None:
                # Wir machen eine Kopie, damit wir die Polygone für den nächsten Frame nicht zerstören
                debug_frame = self.display_image.copy()
                debug_frame = self._draw_debug_overlay(debug_frame, cmd.v, cmd.omega)
                cv2.imshow("Duck Avoidance Challange", debug_frame)
            cv2.waitKey(1)
                
            rate.sleep()

    def _draw_debug_overlay(self, img, cmd_v, cmd_omega):
        """Zeichnet die aktuelle Absicht des Bots ins Debug-Bild."""
        if img is None:
            return img

        # 1. Text für Translation (Vor/Zurück/Stehen)
        if abs(cmd_v) < 0.08:
            action_v = "stehen"
        else:
            action_v = "fahren"

        # 2. Text für Rotation (Geradeaus/Links/Rechts)
        if abs(cmd_omega) < 0.1:
            action_w = "geradeaus"
        elif cmd_omega > 0:
            action_w = "links"
        else:
            action_w = "rechts"

        # 3. String zusammensetzen
        intent_text = f"Ich wuerde gerne: {action_v} und {action_w}"

        # 4. Hintergrund-Balken für bessere Lesbarkeit
        overlay = img.copy()
        cv2.rectangle(overlay, (0, img.shape[0] - 40), (img.shape[1], img.shape[0]), (0, 0, 0), -1)
        
        # 5. Transparenz anwenden (Alpha-Blending)
        alpha = 0.6
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

        # 6. Text auf das Bild zeichnen
        cv2.putText(img, intent_text, (10, img.shape[0] - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return img

    def _on_shutdown(self):
        """Publish zero velocities to ensure motors stop when node exits."""
        try:
            rospy.loginfo("Shutting down DuckAvoidanceNode: publishing zero cmd to motors")
            stop = Twist2DStamped()
            stop.header.stamp = rospy.Time.now()
            stop.v = 0.0
            stop.omega = 0.0
            for _ in range(5):
                try:
                    self.pub_cmd.publish(stop)
                except Exception:
                    pass
                time.sleep(0.05)
            cv2.destroyAllWindows()
        except Exception as e:
            rospy.logwarn(f"Exception while sending stop command on shutdown: {e}")
if __name__ == '__main__':
    node = DuckAvoidanceNode('duck_avoidance_node')
    node.run()