#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped

class CrossIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self.pub_cmd_vel = rospy.Publisher(f"/{self._vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.pub_done = rospy.Publisher(f"/{self._vehicle_name}/intersection/turn_completed", String, queue_size=1)
        
        rospy.Subscriber(f"/{self._vehicle_name}/intersection/execute_turn", String, self.cbExecuteTurn, queue_size=1)
        rospy.loginfo("Cross Intersection Node ist bereit.")

    def drive(self, v, omega, duration):
        """Hilfsfunktion: Fährt für 'duration' Sekunden mit Geschwindigkeit 'v' und Lenkung 'omega'"""
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        
        # Sende den Befehl
        self.pub_cmd_vel.publish(twist)
        
        # Warte die angegebene Zeit
        rospy.sleep(duration)

    def cbExecuteTurn(self, msg):
        decision = msg.data
        rospy.loginfo(f"Starte Manöver: {decision}")

        # ACHTUNG TUNING: Diese Werte musst du an eure Matte anpassen!
        if decision == "straight":
            self.drive(v=0.2, omega=0.2, duration=2.0)
            
        elif decision == "right":
            # 1. Kurz geradeaus in die Kreuzung
            self.drive(v=0.2, omega=0.3, duration=1.2)
            # 2. Hart rechts lenken
            self.drive(v=0.15, omega=-3.6, duration=0.9)
            # 3. Kurz geradeaus aus der Kreuzung raus
            self.drive(v=0.2, omega=0.1, duration=0.5)
            
        elif decision == "left":
            # 1. Etwas weiter geradeaus in die Kreuzung (Linksabbieger fahren einen weiteren Bogen)
            self.drive(v=0.2, omega=0.3, duration=1.2)
            # 2. Hart links lenken
            self.drive(v=0.2, omega=3.0, duration=1.6)
            # 3. Kurz geradeaus
            #self.drive(v=0.2, omega=0.0, duration=0.5)

        # Vollbremsung am Ende des Manövers
        self.drive(v=0.0, omega=0.0, duration=0.1)

        rospy.loginfo("Manöver beendet. Gebe Kontrolle zurück.")
        self.pub_done.publish(String(data="done"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = CrossIntersectionNode('cross_intersection_node')
    node.run()