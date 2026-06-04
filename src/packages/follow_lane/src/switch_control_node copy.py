#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Int32, String
from duckietown_msgs.msg import Twist2DStamped
from enum import Enum

class State(Enum):
    LANE_FOLLOWING = 1
    STOPPED_AT_LINE = 2
    CROSSING_INTERSECTION = 3

class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        self.state = State.LANE_FOLLOWING
        self.turn_decision = "straight" # Standardwert, falls kein Schild da ist

        # Publisher
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher(f"/{self._vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.pub_execute_turn = rospy.Publisher(f"/{self._vehicle_name}/intersection/execute_turn", String, queue_size=1)

        # Subscriber
        rospy.Subscriber(f"/{self._vehicle_name}/detect/stopline", String, self.cbStopline, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/intersection/turn_decision", String, self.cbTurnDecision, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/intersection/turn_completed", String, self.cbTurnCompleted, queue_size=1)

        rospy.loginfo("Switch Control (State Machine) gestartet.")

    def cbTurnDecision(self, msg):
        # Aktualisiert die Entscheidung immer, wenn der Sign Node etwas Neues sieht
        self.turn_decision = msg.data

    def cbStopline(self, msg):
        if self.state == State.LANE_FOLLOWING:
            rospy.loginfo("Stopplinie erkannt! Wechsle in STOPPED_AT_LINE.")
            self.state = State.STOPPED_AT_LINE
            
            # 1. Spurfolge deaktivieren
            self.pub_control.publish(Int32(0)) 
            
            # 2. Aktive Vollbremsung
            twist = Twist2DStamped(v=0.0, omega=0.0)
            self.pub_cmd_vel.publish(twist)
            
            # 3. Warte 2 Sekunden, dann fahre über die Kreuzung
            rospy.Timer(rospy.Duration(2.0), self.trigger_intersection_crossing, oneshot=True)

    def trigger_intersection_crossing(self, event):
        if self.state == State.STOPPED_AT_LINE:
            rospy.loginfo(f"Fahre über Kreuzung. Entscheidung: {self.turn_decision}")
            self.state = State.CROSSING_INTERSECTION
            # Sende den Befehl (z.B. "left", "right") an den Cross Intersection Node
            self.pub_execute_turn.publish(String(data=self.turn_decision))

    def cbTurnCompleted(self, msg):
        if self.state == State.CROSSING_INTERSECTION:
            rospy.loginfo("Kreuzung überquert. Wechsle zurück zu LANE_FOLLOWING.")
            self.state = State.LANE_FOLLOWING
            # Spurfolge wieder aktivieren
            self.pub_control.publish(Int32(1))
            self.turn_decision = "straight" # Reset

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Während wir stehen, spammen wir sicherheitshalber v=0
            if self.state == State.STOPPED_AT_LINE:
                self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))
            
            # Wenn wir im normalen Modus sind, senden wir stetig das "Enable"-Signal (1)
            elif self.state == State.LANE_FOLLOWING:
                self.pub_control.publish(Int32(1))
                
            rate.sleep()

if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()