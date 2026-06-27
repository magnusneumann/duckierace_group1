#!/usr/bin/env python3
import os
import json
import yaml
import rospy
import cv2
import numpy as np
import time
from collections import deque
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped

class DuckAvoidanceNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._v = os.environ.get('VEHICLE_NAME', 'gundel')
        self.enable = True # Wird durch /switch/lane_control gesteuert
        
        # --- KONFIGURATIONSPFADE ---
        self.path_hsv = '/root/DuckieRace/src/packages/avoid_ducks/config/detect_lane_node.json'
        self.path_homography = '/root/DuckieRace/src/packages/avoid_ducks/config/homography.yaml'
        self.path_intrinsics = '/root/DuckieRace/src/packages/avoid_ducks/config/my_camera_info.yaml'
        
        # --- KAMERA & ENTZERRUNG ---
        self.map1 = None
        self.map2 = None
        self.K = None
        self.D = None
        self._load_intrinsics()
        
        # --- HOMOGRAPHIE & TRAPEZE ---
        self.H = None
        self.trapezoids_2d = []
        self._load_homography_and_build_zones()

        # --- MASKEN PARAMETER ---
        self.hsv_limits = {}
        self._load_hsv_config()
        self.pixel_threshold_yellow = 100 
        self.pixel_threshold_red = 10000
        self.pixel_threshold_white = 100
        # --- ZUSTAND & PERCEPTION ---
        self.state = "DRIVING" # DRIVING, ROTATING, STOPPED_RED, COOLDOWN
        self.zones_status = [{"white": False, "yellow": False, "red": False} for _ in range(4)]
        self.display_image = None
        
        # Median Filter für Zone 2
        self.buffer_size = 17
        self.z2_yellow_history = deque(maxlen=self.buffer_size)
        
        # Timer & Wiggle Tracking für Rotationen
        self.stop_timer_start = 0.0
        self.cooldown_timer_start = 0.0
        self.last_inversion_time = time.monotonic()
        self.last_wiggle_time = time.monotonic()
        self.wiggle_direction = -1.0
        self.wiggle_power = 0.066
        self.escape_direction = 1.0

        # --- ROS INTERFACES ---
        self.pub_cmd = rospy.Publisher(f"/{self._v}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.pub_debug = rospy.Publisher(f"/{self._v}/debug/avoidance_view/compressed", CompressedImage, queue_size=1)
        
        # NEU: Publisher für den Switch Control Node (Meldet erkannte Stopplinie)
        self.pub_stop_line = rospy.Publisher(f"/{self._v}/detect/stop_line", Bool, queue_size=1)
        
        # Subscriber
        rospy.Subscriber(f"/{self._v}/camera_node/image/compressed", CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber(f"/{self._v}/switch/lane_control", Int32, self.cbControl, queue_size=1)

        rospy.loginfo("Duck Avoidance Node (Pure Lane Control Edition) initialisiert.")
        rospy.on_shutdown(self._on_shutdown)

    # ==========================================
    # 1. SCHNITTSTELLEN FÜR SWITCH CONTROL
    # ==========================================
    def cbControl(self, msg):
    #    # 1 bedeutet KNOTEN AKTIV, 0 bedeutet PAUSIERT (Intersection Node übernimmt)
        self.enable = (msg.data == 1)
    #    #if self.enable and msg.data != 1:
    #    #    rospy.loginfo("Lane Control AKTIVIERT.")
    #    if msg.data != 0:    
    #        rospy.loginfo("Lane Control PAUSIERT.")

    # ==========================================
    # 2. INITIALISIERUNG & SETUP
    # ==========================================
    def _load_intrinsics(self):
        try:
            with open(self.path_intrinsics, 'r') as f:
                data = yaml.safe_load(f)
                self.K = np.array(data['K']).reshape((3,3))
                self.D = np.array(data['D'])
        except Exception as e:
            rospy.logwarn(f"Intrinsics Fehler: {e}")

    def _ensure_undistort_maps(self, w, h):
        if self.map1 is not None and self.map2 is not None: return
        if self.K is None or self.D is None: return
        try:
            K = self.K.astype(np.float64)
            D = self.D.astype(np.float64)
            if D.size == 4:
                try:
                    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=0.0)
                    self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, (w, h), cv2.CV_16SC2)
                except Exception:
                    pass
            else:
                newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
                self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
        except Exception:
            pass

    def _load_homography_and_build_zones(self):
        try:
            with open(self.path_homography, 'r') as f:
                data = yaml.safe_load(f)
                self.H = np.array(data['homography']).reshape((3,3))
        except Exception:
            return

        zones_3d = [
            [(0.1, -0.06), (0.14, -0.06), (0.14, 0.06), (0.1, 0.06)],   # Zone 0
            [(0.14, -0.08), (0.2, -0.073), (0.2, 0.073), (0.14, 0.08)], # Zone 1
            [(0.2, -0.073), (0.3, -0.07), (0.3, 0.07), (0.2, 0.073)],    # Zone 2
            [(0.33, -0.11), (0.42, -0.12), (0.42, 0.12), (0.33, 0.11)]    # Zone 3
        ]

        for z in zones_3d:
            pts_2d = []
            for (X, Y) in z:
                vec = np.array([X, Y, 1.0])
                proj = np.dot(self.H, vec)
                u = int(proj[0] / proj[2])
                v = int(proj[1] / proj[2])
                pts_2d.append([u, v])
            self.trapezoids_2d.append(np.array(pts_2d, dtype=np.int32))

    def _load_hsv_config(self):
        try:
            with open(self.path_hsv, 'r') as f:
                self.hsv_limits = json.load(f)
        except Exception as e:
            rospy.logwarn(f"HSV Config Fehler, nutze Fallback: {e}")
            self.hsv_limits = {
                "white": {"lower": [0, 0, 150], "upper": [180, 50, 255]},
                "yellow": {"lower": [20, 100, 100], "upper": [40, 255, 255]},
                "red1": {"lower": [0, 100, 100], "upper": [10, 255, 255]},
                "red2": {"lower": [170, 100, 100], "upper": [180, 255, 255]}
            }

    # ==========================================
    # 3. VISION & PERCEPTION LOOP
    # ==========================================
    def cb_image(self, msg):
        if not self.enable:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = img.shape[:2]

        self._ensure_undistort_maps(w, h)
        if self.map1 is not None and self.map2 is not None:
            undistorted = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        else:
            try:
                undistorted = cv2.fisheye.undistortImage(img, self.K, self.D) if self.D.size == 4 else cv2.undistort(img, self.K, self.D)
            except Exception:
                undistorted = img.copy()

        if not self.trapezoids_2d:
            return

        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
        
        # Masken
        mask_white = cv2.inRange(hsv, np.array(self.hsv_limits['white']['lower']), np.array(self.hsv_limits['white']['upper']))
        mask_yellow = cv2.inRange(hsv, np.array(self.hsv_limits['yellow']['lower']), np.array(self.hsv_limits['yellow']['upper']))
        
        # Rote Maske aus zwei Bereichen (Wrap-Around in OpenCV)
        mask_red1 = cv2.inRange(hsv, np.array(self.hsv_limits.get('red1', {}).get('lower', [0, 100, 100])), np.array(self.hsv_limits.get('red1', {}).get('upper', [10, 255, 255])))
        mask_red2 = cv2.inRange(hsv, np.array(self.hsv_limits.get('red2', {}).get('lower', [170, 100, 100])), np.array(self.hsv_limits.get('red2', {}).get('upper', [180, 255, 255])))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        self.zones_status = [{"white": False, "yellow": False, "red": False} for _ in range(4)]
        
        for i, trap_pts in enumerate(self.trapezoids_2d):
            trap_mask = np.zeros_like(mask_white)
            cv2.fillPoly(trap_mask, [trap_pts], 255)
            
            raw_white = cv2.countNonZero(cv2.bitwise_and(mask_white, trap_mask))
            raw_yellow = cv2.countNonZero(cv2.bitwise_and(mask_yellow, trap_mask))
            raw_red = cv2.countNonZero(cv2.bitwise_and(mask_red, trap_mask))
            
            # Hybrid-Filter Logik für Gelb in Zone 2
            if i == 2: 
                self.z2_yellow_history.append(raw_yellow)
                eval_yellow = np.median(self.z2_yellow_history)
            else:
                eval_yellow = raw_yellow

            eval_white = raw_white

            # Werte eintragen
            if eval_white > self.pixel_threshold_white:
                self.zones_status[i]["white"] = True
                undistorted[cv2.bitwise_and(mask_white, trap_mask) > 0] = (255, 0, 0)
                
            if eval_yellow > self.pixel_threshold_yellow:
                self.zones_status[i]["yellow"] = True
                undistorted[cv2.bitwise_and(mask_yellow, trap_mask) > 0] = (0, 165, 255)
                
            if raw_red > self.pixel_threshold_red:
                self.zones_status[i]["red"] = True
                undistorted[cv2.bitwise_and(mask_red, trap_mask) > 0] = (0, 0, 255)

            # Polylinien zeichnen
            if self.zones_status[i]["white"] or self.zones_status[i]["yellow"] or self.zones_status[i]["red"]:
                cv2.polylines(undistorted, [trap_pts], isClosed=True, color=(0, 0, 255), thickness=2)
            else:
                cv2.polylines(undistorted, [trap_pts], isClosed=True, color=(0, 255, 255), thickness=2)
        
        self.display_image = undistorted

    # ==========================================
    # 4. CONTROL LOOP & FSM
    # ==========================================
    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            # GUI immer zeichnen, auch wenn pausiert
            #if self.display_image is not None:
            #    debug_frame = self._draw_debug_overlay(self.display_image.copy(), self.state)
            #    cv2.imshow("Duck Avoidance & Lane Control", debug_frame)
            #cv2.waitKey(1)

            # --- KONTROLLE AUSSETZEN, WENN DURCH SWITCH PAUSIERT ---
            if not self.enable:
                rate.sleep()
                continue

            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()
            current_time = time.monotonic()

            z0 = self.zones_status[0]
            z1 = self.zones_status[1]
            z2 = self.zones_status[2]
            z3 = self.zones_status[3]

            # ==========================================================
            # PHASE 1: ZUSTANDS-WECHSEL
            # ==========================================================
            
            # Prio 1: Rote Linie für Stopp erkennen
            if self.state not in ["STOPPED_RED", "COOLDOWN"] and z0["red"]:
                rospy.loginfo("Rote Linie in Zone 0 erkannt! Anhalten & Switch Control benachrichtigen.")
                self.state = "STOPPED_RED"
                self.stop_timer_start = current_time
                # Informiere den Switch Control Node!
                self.pub_stop_line.publish(Bool(True))

            elif self.state == "STOPPED_RED":
                if (current_time - self.stop_timer_start) >= 2.0:
                    rospy.loginfo("2s Stopp beendet. Überquere Linie (3s Cooldown).")
                    self.state = "COOLDOWN"
                    self.cooldown_timer_start = current_time

            elif self.state == "COOLDOWN":
                if (current_time - self.cooldown_timer_start) >= 3.0:
                    rospy.loginfo("Cooldown beendet. Achte wieder auf rote Linien.")
                    self.state = "DRIVING"

            # Prio 2: Weiße/Gelbe Linie in Zone 0 (Notfall-Rotation)
            elif self.state not in ["ROTATING", "STOPPED_RED", "COOLDOWN"] and (z0["white"] or z0["yellow"]):
                rospy.loginfo("Wegen Linie in Zone 0 rotieren.")
                self.state = "ROTATING"
                self.escape_direction = 1.0 if z0["white"] else -1.0
                self.last_wiggle_time = current_time
                self.wiggle_direction = -1.0
                
            # Abbruch der Rotation
            elif self.state == "ROTATING" and not z1["yellow"] and not z1["white"]:
                rospy.loginfo("Linienrotation abgeschlossen. Weiterfahren.")
                self.state = "DRIVING"


            # ==========================================================
            # PHASE 2: MOTOR-AKTIONEN AUSFÜHREN (Ohne PID!)
            # ==========================================================
            
            if self.state == "STOPPED_RED":
                cmd.v = 0.0
                cmd.omega = 0.0

            elif self.state == "ROTATING":
                if current_time - self.last_wiggle_time > 0.06:
                    self.wiggle_direction *= -1.0
                    self.last_wiggle_time = current_time
                cmd.v = 1.0 * 0.08 * self.wiggle_direction

                # Invertierungsschutz
                dt = current_time - self.last_inversion_time
                if dt > 1.0: 
                    if self.escape_direction == 1.0 and z2["yellow"]:
                        self.escape_direction = -1.0
                        self.last_inversion_time = current_time
                    elif self.escape_direction == -1.0 and z2["white"]:
                        self.escape_direction = 1.0
                        self.last_inversion_time = current_time

                cmd.omega = 1.6 * self.escape_direction

            elif self.state in ["DRIVING", "COOLDOWN"]:
                # 1. Stufe: Harte Spurkorrektur in Zone 1
                if z1["white"] or z1["yellow"]:
                    cmd.v = 0.15
                    cmd.omega = 2.7 if z1["white"] else -2.7
                
                # 2. Stufe: Weiche Spurkorrektur in Zone 2
                elif z2["white"] or z2["yellow"]:
                    cmd.v = 0.15 
                    cmd.omega = 1.3 if z2["white"] else -1.3 
                    
                # 3. Stufe: Gelb und weiß in Zone 3
                elif z3["white"] or z3["yellow"]:
                    cmd.v = 0.18
                    cmd.omega = 0.6 if z3["white"] else -0.6
                
                # 4. Stufe ALLES FREI -> geradeaus fahren
                else:
                    cmd.v = 0.18 
                    cmd.omega = 0.0

            # --- COMMAND PUBLISH ---
            try:
                self.pub_cmd.publish(cmd)
            except Exception:
                pass
                
            rate.sleep()

    def _draw_debug_overlay(self, img, state):
        if img is None: return img
    #    overlay = img.copy()
    #    cv2.rectangle(overlay, (0, img.shape[0] - 40), (img.shape[1], img.shape[0]), (0, 0, 0), -1)
    #    alpha = 0.6
    #    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    #    
    #    status_text = f"Modus: {state}"
    #    if not self.enable:
    #        status_text += " [PAUSIERT VON SWITCH_CONTROL]"
    #        
    #    cv2.putText(img, status_text, (10, img.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return img

    def _on_shutdown(self):
        try:
            stop = Twist2DStamped()
            stop.header.stamp = rospy.Time.now()
            stop.v = 0.0
            stop.omega = 0.0
            for _ in range(5):
                self.pub_cmd.publish(stop)
                time.sleep(0.05)
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == '__main__':
    node = DuckAvoidanceNode('duck_avoidance_node')
    node.run()