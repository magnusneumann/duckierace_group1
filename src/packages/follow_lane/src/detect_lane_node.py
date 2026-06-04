#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml
import util
from geometry_msgs.msg import Polygon

class DetectLaneNode:
    def __init__(self, node_name):
        # ROS-Knoten initialisieren
        rospy.init_node(node_name)
        self.duck_obstacles = None
        self.stop_cooldown = 0
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)
                
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        
        # OPTIMIERUNG: buff_size drastisch erhöht, um Kamera-Lag im ROS-Netzwerk zu verhindern
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic, 
            CompressedImage, 
            self.cbFindLane, 
            queue_size=1, 
            buff_size=2**24
        )
        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size=1)

        self._crop_im_size = 400
        self.is_running = False
        self.counter = 0

        # Debug-Kanäle initialisieren
        self.pub_debug_lane = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_croped', CompressedImage, queue_size=1)
        self.pub_debug_white = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_white', CompressedImage, queue_size=1)
        self.pub_debug_yellow = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_yellow', CompressedImage, queue_size=1)
        self.pub_debug_red = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_red', CompressedImage, queue_size=1)

        self.pub_stopline = rospy.Publisher(f'/{self._vehicle_name}/detect/stopline', String, queue_size=1)
        
        # Übernahme der erkannten Enten als Polygon-Nachricht
        rospy.Subscriber(f"/{self._vehicle_name}/detect/duck_obstacles", Polygon, self.cb_duck_obstacles, queue_size=1)
    
    def cb_duck_obstacles(self, msg):
        self.duck_obstacles = msg

    def cbUpdateParameters(self, parameters):
        # Parameter für weiße Linie
        self.hue_white_l = parameters["white"]["hl"]["default"]
        self.hue_white_h = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l = parameters["white"]["vl"]["default"]
        self.lightness_white_h = parameters["white"]["vh"]["default"]
        
        # Parameter für gelbe Linie
        self.hue_yellow_l = parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h = parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l = parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h = parameters["yellow"]["vh"]["default"]
        
        # Perspektiventransformations-Punkte
        self.top_left_x = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

        # Parameter für rote Stopplinie
        self.hue_red_l1 = parameters["red"]["hl1"]["default"]
        self.hue_red_h1 = parameters["red"]["hh1"]["default"]
        self.hue_red_l2 = parameters["red"]["hl2"]["default"]
        self.hue_red_h2 = parameters["red"]["hh2"]["default"]
        self.saturation_red_l = parameters["red"]["sl"]["default"]
        self.saturation_red_h = parameters["red"]["sh"]["default"]
        self.lightness_red_l = parameters["red"]["vl"]["default"]
        self.lightness_red_h = parameters["red"]["vh"]["default"]
        self.red_pixel_threshold = parameters["red"]["threshold"]["default"]

    def detect_stopline(self, img):
        # OPTIMIERUNG: Nur die unteren 30% ausschneiden BEVOR konvertiert und gefiltert wird
        h = img.shape[0]
        roi_bgr = img[int(h * 0.7):h, :]
        
        # Farbraumkonvertierung läuft jetzt blitzschnell auf der ROI
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv,
            (self.hue_red_l1, self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h1, self.saturation_red_h, self.lightness_red_h))
        mask2 = cv2.inRange(hsv,
            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask_red = cv2.bitwise_or(mask1, mask2)

        red_pixels = np.count_nonzero(mask_red)

        if self.stop_cooldown > 0:
            self.stop_cooldown -= 1
            return mask_red

        if red_pixels > self.red_pixel_threshold:
            rospy.loginfo(f"Stopline detected! ({red_pixels} px)")
            self.pub_stopline.publish(String(data="stop"))
            self.stop_cooldown = 100  # ~4 Sekunden Cooldown bei 10Hz
        return mask_red
  
    def crop_img(self, img):
        img = img.copy()
        
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(img, M, (self._crop_im_size, self._crop_im_size))

    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        y_start = max(0, distance - 50)
        y_end = min(mask.shape[0], distance + 50)
        roi = mask[y_start:y_end, :]

        grad = cv2.Sobel(roi, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        _, th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        row_max = th1.max(axis=1)
        valid_rows = (row_max == 255)

        if np.sum(valid_rows) < 10:
            return no_lane_value

        if left_line:
            flipped_th1 = np.fliplr(th1)
            first_max_flipped = flipped_th1.argmax(axis=1)
            edge_x_values = mask.shape[1] - 1 - first_max_flipped[valid_rows]
        else:
            edge_x_values = th1.argmax(axis=1)[valid_rows]

        return np.median(edge_x_values)
        
    def cbFindLane(self, image_msg):
        if self.counter <= 3:
            self.counter += 1   
            return

        if self.is_running:
            return
        
        self.is_running = True
        self.counter = 0 # OPTIMIERUNG: Tippfehler korrigiert (war vorher conunter)

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.debug_img_red = self.detect_stopline(cv_image)
                
        # --- HIER DIE ZWEI ZEILEN EINFÜGEN ---
        cv2.imshow('Rote Maske (Stopplinie)', self.debug_img_red)
        cv2.waitKey(1) # Wichtig, damit das Fenster aktualisiert und nicht einfriert
        # -------------------------------------

        img = self.crop_img(cv_image)
        img = self.crop_img(cv_image)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l, self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h, self.saturation_white_h, self.lightness_white_h),)
        
        # --- ENTE: MASKEN MANIPULATION ---
        if self.duck_obstacles and len(self.duck_obstacles.points) == 4:
            tl = self.duck_obstacles.points[0] # Top-Left
            tr = self.duck_obstacles.points[1] # Top-Right
            br = self.duck_obstacles.points[2] # Bottom-Right
            bl = self.duck_obstacles.points[3] # Bottom-Left

            # 1. ADDITION (Weiß)
            ramp_pts = np.array([
                [int(tl.x)-50, 0],
                [self._crop_im_size, 0],
                [self._crop_im_size, self._crop_im_size],
                [int(tl.x)-50, self._crop_im_size]], np.int32)
            cv2.fillPoly(mask_white, [ramp_pts], 255)
            
            # 2. SUBTRAKTION (Schwarz)
            bb_pts = np.array([
                [int(tl.x), int(tl.y)],
                [int(tr.x), int(tr.y)],
                [int(br.x), int(br.y)],
                [int(bl.x), int(bl.y)]
            ], np.int32)
            cv2.fillPoly(mask_yellow, [bb_pts], 0)
            cv2.circle(mask_yellow, (int(tl.x), int(tl.y)), 170, 0, -1)
        
        white_alternative = int(len(img[0]) * 0.99)
        yellow_alternative = int(len(img[0]) * 0.01)

        self.lookahead = 0.75

        center_white = self.get_x_for_driving(mask_white, int(len(img)*self.lookahead), white_alternative, left_line=False)
        center_yellow = self.get_x_for_driving(mask_yellow, int(len(img)*self.lookahead), yellow_alternative, left_line=False)

        if center_white <= center_yellow:
            if center_white > int(len(img[0]) * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative

        lane_center = (center_white + center_yellow) / 2

        msg_error = Float64()
        msg_error.data = 1 - (lane_center / len(img) * 2)

        self.pub_lane.publish(msg_error)
        
        # OPTIMIERUNG: Blockierendes print() durch gedrosseltes rospy-Logging ersetzt (loggt max. 1x pro Sekunde)
        rospy.loginfo_throttle(1.0, f"Lane error: {msg_error.data:.4f} range [-1,1]")

        # Daten für Debug-Thread sichern
        self.img = img
        self.lane_center = lane_center
        self.white_alternative = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white = center_white
        self.center_yellow = center_yellow

        self.debug_img_white = mask_white
        self.debug_img_yellow = mask_yellow

        self.is_running = False
            
    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not hasattr(self, 'img'):
                rate.sleep()
                continue
            
            # Debug-Bilder erzeugen und publishen, wenn Subscriber zuhören
            if self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()
                debug_img = cv2.circle(debug_img, (int(self.lane_center), int(len(debug_img) / 2)), 3, (255,0,0))
                debug_img = cv2.line(debug_img, (self.white_alternative, 0), (self.white_alternative, 1000), color=(255,255,255)) 
                debug_img = cv2.line(debug_img, (self.yellow_alternative, 0), (self.yellow_alternative, 1000), color=(255,255,0))
                debug_img = cv2.line(debug_img, (0, int(len(debug_img) * self.lookahead) + 100), (len(debug_img[0]), int(len(debug_img) * self.lookahead) + 100), color=(255,255,255))
                debug_img = cv2.line(debug_img, (0, int(len(debug_img) * self.lookahead) - 100), (len(debug_img[0]), int(len(debug_img) * self.lookahead) - 100), color=(255,255,255))
                debug_img = cv2.line(debug_img, (int(len(debug_img[0])/2), 0), (int(len(debug_img[0])/2), len(debug_img)), (0,255,0))
                debug_img = cv2.circle(debug_img, (int(self.center_white), int(len(debug_img) * self.lookahead)), 5, (255,255,255))
                debug_img = cv2.circle(debug_img, (int(self.center_yellow), int(len(debug_img) * self.lookahead)), 5, (0,255,255))

                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
                self.pub_debug_lane.publish(debug_msg)

            if self.pub_debug_white.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_white)[1]).tobytes()
                self.pub_debug_white.publish(debug_msg)

            if self.pub_debug_yellow.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_yellow)[1]).tobytes()
                self.pub_debug_yellow.publish(debug_msg)

            if self.pub_debug_red.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_red)[1]).tobytes()
                self.pub_debug_red.publish(debug_msg)

            rate.sleep()
        
if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()