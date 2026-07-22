#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Int32, String, Bool
from duckietown_msgs.msg import Twist2DStamped
from enum import Enum

class State(Enum):
    STANDBY = 0
    LANE_FOLLOWING = 1
    STOPPED_AT_LINE = 2
    CROSSING_INTERSECTION = 3

class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        # ROS-Parameter einlesen (Standard: False -> fängt sofort an zu fahren)
        # Dadurch werden andere Launcher nicht kaputt gemacht!
        start_in_standby = rospy.get_param("~start_in_standby", False)
        
        if start_in_standby:
            self.state = State.STANDBY
            rospy.loginfo("Switch Control (FSM) gestartet. Befindet sich im STANDBY.")
        else:
            self.state = State.LANE_FOLLOWING
            rospy.loginfo("Switch Control (FSM) gestartet. Bereit für alle Manöver (LANE_FOLLOWING).")
            
        self.turn_decision = None # Standard: Wir wissen noch nicht, wohin.
        
        # Timer: Bis wann ignorieren wir rote Linien? (Gegen das sofortige Halten nach der Kurve)
        self.ignore_stopline_until = rospy.Time(0)

        # --- Publisher ---
        self.pub_lane_control = rospy.Publisher(f"/{self._vehicle_name}/switch/lane_control", Int32, queue_size=1)
        self.pub_execute_turn = rospy.Publisher(f"/{self._vehicle_name}/intersection/execute_turn", String, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher(f"/{self._vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # --- Subscriber ---
        rospy.Subscriber(f"/{self._vehicle_name}/detect/stop_line", Bool, self.cbStopline, queue_size=1)
        
        rospy.Subscriber(f"/{self._vehicle_name}/intersection/turn_decision", String, self.cbTurnDecision, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/intersection/turn_completed", String, self.cbTurnCompleted, queue_size=1)

    def cbTurnDecision(self, msg):
        # Wenn wir noch im Standby sind, wachen wir auf, da wir jetzt ein Ziel haben!
        if self.state == State.STANDBY:
            rospy.loginfo("Routen-Befehl empfangen! Wecke Roboter aus dem Standby auf.")
            self.state = State.LANE_FOLLOWING
            
        # Wir updaten die Entscheidung asynchron. 
        # (Wird ignoriert, falls wir gerade schon auf der Kreuzung sind)
        if self.state != State.CROSSING_INTERSECTION:
            self.turn_decision = msg.data

    def cbStopline(self, msg):
        now = rospy.Time.now()
        
        # Wir reagieren nur auf Linien, wenn wir im Spur-Modus sind UND die Ignorier-Zeit abgelaufen ist.
        if self.state == State.LANE_FOLLOWING and now > self.ignore_stopline_until:
            rospy.loginfo("Stopplinie erreicht! Friere aktuelle Entscheidung ein.")
            self.state = State.STOPPED_AT_LINE
            
            self.pub_lane_control.publish(Int32(0)) # Spurfolge aus
            self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0)) # Vollbremsung
            
            # Wartepflicht von 2 Sekunden absitzen
            rospy.Timer(rospy.Duration(2.0), self.trigger_intersection_crossing, oneshot=True)

    def trigger_intersection_crossing(self, event):
        # Nur losfahren, wenn nicht in der Zwischenzeit eine Ente aufs Bild gelaufen ist!
        if self.state == State.STOPPED_AT_LINE:
            
            # Fallback: Wenn wir nie ein Schild gesehen haben, fahren wir geradeaus.
            if self.turn_decision is None:
                rospy.logwarn("Kein Schild gesehen! Fallback: Fahre GERADEAUS.")
                self.turn_decision = "straight"
                
            rospy.loginfo(f"Fahre über Kreuzung: {self.turn_decision}")
            self.state = State.CROSSING_INTERSECTION
            self.pub_execute_turn.publish(String(data=self.turn_decision))

    def cbTurnCompleted(self, msg):
        if self.state == State.CROSSING_INTERSECTION:
            rospy.loginfo("Kreuzung beendet. Ignoriere Stopplinien für 3 Sekunden.")
            
            # Gehörlos-Phase aktivieren, um Reste der roten Linie zu überfahren
            self.ignore_stopline_until = rospy.Time.now() + rospy.Duration(3.0)
            
            self.state = State.LANE_FOLLOWING
            self.turn_decision = None # Reset für die nächste Kreuzung

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Der Boss verteilt hier die Arbeitserlaubnis (Enable-Signale) stetig an die Arbeiter
            if self.state == State.LANE_FOLLOWING:
                self.pub_lane_control.publish(Int32(1))
            elif self.state == State.STOPPED_AT_LINE or self.state == State.STANDBY:
                self.pub_lane_control.publish(Int32(0))
                # Haltekommando kontinuierlich senden, damit er wirklich steht
                self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))
                
            rate.sleep()

if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()