#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Int32, String
from duckietown_msgs.msg import Twist2DStamped
import os
from switch_control_node import ControlType
import util

class ControlLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.enable = True
        self.is_stopped = False  # NEU: Eigene Variable für die Stopplinie
        self._vehicle_name = os.environ['VEHICLE_NAME']
        rospy.init_node(node_name)
        self.lastError = 0
        self.v = 0.1
        self.a = 0
        self.integral = 0

        util.init_parameters(node_name, self.cbUpdateParameters)

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(detect_lane_topic, Float64, self.cbFollowLane, queue_size=1)
        control_change_topic = f"/{self._vehicle_name}/switch/control"
        self.sub_control = rospy.Subscriber(control_change_topic, Int32, self.cbControl, queue_size=1)
        self.sub_stopline = rospy.Subscriber(f"/{self._vehicle_name}/detect/stopline", String, self.cbStopline, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self, msg):
        if msg.data == ControlType.Lane.value:
            self.enable = True
        else:
            self.enable = False

    def cbUpdateParameters(self, parameters):
        self.kp = parameters["pid"]["p"]["default"]
        self.ki = parameters["pid"]["i"]["default"]
        self.kd = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]

    def cbFollowLane(self, error):
        error = error.data
        self.v = 0.2

        proportional = self.kp * error

        self.integral += error
        self.integral = max(-5.0, min(5.0, self.integral))
        if (error > 0) != (self.lastError > 0):
            self.integral = 0.0

        integral = self.ki * self.integral
        derivative = self.kd * (error - self.lastError)
        self.a = proportional + integral + derivative
        self.lastError = error

    def cbStopline(self, msg):
        # NEU: Nur auslösen, wenn wir nicht sowieso schon stehen
        if not self.is_stopped:
            rospy.loginfo("Stopline! Stopping for 2 seconds.")
            self.is_stopped = True
            
            # NEU: Den Motoren EXPLIZIT sagen, dass sie anhalten sollen!
            twist = Twist2DStamped()
            twist.header.stamp = rospy.Time.now()
            twist.v = 0.0
            twist.omega = 0.0
            self.pub_cmd_vel.publish(twist)
            
            rospy.Timer(rospy.Duration(3.0), self.cbRestoreAfterStop, oneshot=True)

    def cbRestoreAfterStop(self, event):
        self.is_stopped = False
        rospy.loginfo("Weiterfahren!")

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # NEU: Er darf nur fahren, wenn enabled UND NICHT gestoppt
            if self.enable and not self.is_stopped:
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v = self.v
                twist.omega = self.a
                self.pub_cmd_vel.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    node = ControlLaneNode('control_lane_node')
    node.run()
    rospy.spin()