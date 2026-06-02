#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Int32
from duckietown_msgs.msg import Twist2DStamped
import os
#from switch_control_node import ControlType # wurde von KI entfernt, vielleicht braucht es das aber ja
import util

class ControlLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.enable = True
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        self.lastError = 0
        self.v = 0.2
        self.a = 0
        self.integral = 0

        util.init_parameters(node_name, self.cbUpdateParameters)

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        
        rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbFollowLane, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self, msg):
        # 1 bedeutet LANE_FOLLOWING, 0 bedeutet AUS
        self.enable = (msg.data == 1)

    def cbUpdateParameters(self, parameters):
        self.kp = parameters["pid"]["p"]["default"]
        self.ki = parameters["pid"]["i"]["default"]
        self.kd = parameters["pid"]["d"]["default"]

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

    def fnShutDown(self):
        self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable:
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v = self.v
                twist.omega = self.a
                self.pub_cmd_vel.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    node = ControlLaneNode('control_lane_node')
    node.run()