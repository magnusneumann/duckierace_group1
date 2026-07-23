#!/usr/bin/env python3

import json
import os

import rospy
import time
from std_msgs.msg import Bool, String, Int32
from duckietown_msgs.msg import Twist2DStamped
from std_srvs.srv import Trigger, TriggerResponse

from topological_graph import (
    TopologicalGraph,
    load_challenge_config,
    turn_from_ports,
)


class NavigatorNode:

    def __init__(self):

        rospy.init_node("navigator_node")

        self.vehicle = os.environ.get(
            "VEHICLE_NAME",
            "duckiebot",
        )

        # ----------------------------------------------------------
        # Dateien
        # ----------------------------------------------------------

        package_dir = os.path.dirname(
            os.path.dirname(__file__)
        )

        config_path = os.path.join(
            package_dir,
            "config",
            "graph.json",
        )

        self.mapping_path = os.path.join(
            package_dir,
            "config",
            "challenge4_mapping.json",
        )

        self.default_route_path = os.path.join(
            package_dir,
            "config",
            "default_route.json",
        )

        # ----------------------------------------------------------
        # Graph laden
        # ----------------------------------------------------------

        config = load_challenge_config(
            config_path
        )

        self.graph = TopologicalGraph(
            config["graph"],
            default_weight=config.get(
                "default_edge_weight",
                1.0,
            ),
        )

        # Startwerte aus graph.json
        self.current_node = config["start"][
            "current_node"
        ]

        self.incoming_port = int(
            config["start"][
                "incoming_port"
            ]
        )

        # Mapping ergänzen:
        # Gates + gemessene Fahrzeiten
        self.load_mapping()

        # ----------------------------------------------------------
        # Navigationszustand
        # ----------------------------------------------------------

        self.route_steps = []

        self.active_step = None

        self.edge_started = False

        self.active = False

        # ----------------------------------------------------------
        # ROS
        # ----------------------------------------------------------

        self.pub_turn = rospy.Publisher(
            f"/{self.vehicle}/intersection/turn_decision",
            String,
            queue_size=1,
            latch=True,
        )

        self.pub_lane_switch = rospy.Publisher(
            f"/{self.vehicle}/switch/lane_control",
            Int32,
            queue_size=1,
            latch=True,
        )

        self.pub_cmd = rospy.Publisher(
            f"/{self.vehicle}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1,
        )

        self.pub_standby = rospy.Publisher(
            f"/{self.vehicle}/switch/standby",
            Bool,
            queue_size=1,
            latch=True,
        )

        self.pub_status = rospy.Publisher(
            f"/{self.vehicle}/navigation/status",
            String,
            queue_size=1,
            latch=True,
        )
        
        self.target_gates = []

        rospy.Subscriber(
            f"/{self.vehicle}/intersection/turn_completed",
            String,
            self.cb_turn_completed,
            queue_size=1,
        )

        rospy.Subscriber(
            f"/{self.vehicle}/detect/stop_line",
            Bool,
            self.cb_stopline,
            queue_size=1,
        )

        rospy.Subscriber(
            f"/{self.vehicle}/navigation/route",
            String,
            self.cb_route,
            queue_size=1,
        )

        rospy.Service(
            f"/{self.vehicle}/navigation/stop",
            Trigger,
            self.srv_stop,
        )

        rospy.loginfo(
            f"Navigator bereit | "
            f"Start: {self.current_node}, "
            f"Port {self.incoming_port}"
        )

        # ----------------------------------------------------------
        # Default-Route laden und autostart prüfen
        # ----------------------------------------------------------
        
        self.load_default_route()


    # ==============================================================
    # Mapping laden
    # ==============================================================

    def load_mapping(self):

        state = self.graph.load_mapping(
            self.mapping_path
        )

        # Optional:
        # letzten gespeicherten Zustand übernehmen
        if state:

            self.current_node = state.get(
                "current_node",
                self.current_node,
            )

            self.incoming_port = int(
                state.get(
                    "incoming_port",
                    self.incoming_port,
                )
            )

        rospy.loginfo(
            f"Gates: "
            f"{sorted(self.graph.gates_to_edges().keys())}"
        )

    # ==============================================================
    # Default-Route laden und autostart
    # ==============================================================

    def load_default_route(self):
        """
        Lädt die Standard-Route aus default_route.json.
        Falls auto_start=true, wird die Route automatisch gestartet.
        """
        try:
            with open(self.default_route_path, "r") as f:
                route_config = json.load(f)

            gates = route_config.get("gates", [])
            auto_start = route_config.get("auto_start", False)

            if not gates:
                rospy.logwarn("Keine Gates in default_route.json definiert.")
                return

            rospy.loginfo(
                f"Default-Route geladen: Gates {gates}, "
                f"auto_start={auto_start}"
            )

            if auto_start:
                rospy.loginfo("Starte Route automatisch...")
                self.plan_route(gates)
                self.active = True
                self.pub_lane_switch.publish(Int32(1))
                self.publish_next_turn()

        except FileNotFoundError:
            rospy.logwarn(
                f"default_route.json nicht gefunden unter "
                f"{self.default_route_path}"
            )
        except Exception as e:
            rospy.logerr(
                f"Fehler beim Laden der default_route.json: {e}"
            )

    # ==============================================================
    # Ziel-Gate empfangen
    # ==============================================================

    def cb_route(self, msg):
        """
        Beispiele:

        Einzelnes Gate:
            8

        Mehrere Gates:
            [6, 8, 10]
        """

        try:

            data = json.loads(
                msg.data
            )

            if isinstance(data, int):
                gates = [data]

            elif isinstance(data, list):
                gates = [
                    int(gate)
                    for gate in data
                ]

            else:
                raise ValueError(
                    "Ungültiges Format"
                )

            self.plan_route(
                gates
            )

            self.active = True

            self.pub_lane_switch.publish(Int32(1))

            self.publish_next_turn()

        except Exception as e:

            rospy.logerr(
                f"Navigation konnte nicht "
                f"gestartet werden: {e}"
            )

    # ==============================================================
    # Route berechnen
    # ==============================================================

    def plan_route(self, target_gates):

        # Wenn wir eine neue Route planen, wecken wir alles aus dem Standby auf
        self.pub_standby.publish(Bool(False))

        self.route_steps = []
        self.target_gates = target_gates

        planning_node = (
            self.current_node
        )
        
        planning_incoming_port = (
            self.incoming_port
        )

        total_cost = 0.0

        known_gates = (
            self.graph.gates_to_edges()
        )

        for gate_id in target_gates:

            if gate_id not in known_gates:

                raise ValueError(
                    f"Gate {gate_id} "
                    f"ist unbekannt."
                )

            cost, steps = (
                self.graph.route_to_gate(
                    planning_node,
                    gate_id,
                    planning_incoming_port,
                    allowed_gates=[gate_id]
                )
            )

            total_cost += cost

            self.route_steps.extend(
                steps
            )

            if steps:

                planning_node = (
                    steps[-1]["to"]
                )
                
                last_edge = self.graph.edges[steps[-1]["edge_key"]]
                if planning_node == last_edge["node_a"]:
                    planning_incoming_port = int(last_edge["port_a"])
                else:
                    planning_incoming_port = int(last_edge["port_b"])

        rospy.loginfo(
            "================================"
        )

        rospy.loginfo(
            f"Route zu Gates: "
            f"{target_gates}"
        )

        rospy.loginfo(
            f"Geschätzte Zeit: "
            f"{total_cost:.2f} s"
        )

        for index, step in enumerate(
            self.route_steps,
            start=1,
        ):

            rospy.loginfo(
                f"{index}: "
                f"{step['from']} "
                f"--P{step['exit_port']}--> "
                f"{step['to']} "
                f"| {step['edge_key']}"
            )

        rospy.loginfo(
            "================================"
        )
        
        self.publish_status()

    # ==============================================================
    # Nächste Kreuzung
    # ==============================================================

    def publish_next_turn(self):

        if not self.active:
            return

        if not self.route_steps:

            rospy.loginfo(
                "Navigation abgeschlossen."
            )

            self.active = False
            self.active_step = None

            self.perform_duck_dance_and_standby()

            return

        self.active_step = (
            self.route_steps.pop(0)
        )

        self.edge_started = False

        # Route muss am aktuellen Knoten starten
        if (
            self.active_step["from"]
            != self.current_node
        ):

            rospy.logerr(
                "Route passt nicht zur "
                "aktuellen Position."
            )

            self.active = False

            return

        exit_port = int(
            self.active_step[
                "exit_port"
            ]
        )

        turn = turn_from_ports(
            self.incoming_port,
            exit_port,
        )

        rospy.loginfo(
            f"{self.current_node}: "
            f"P{self.incoming_port} "
            f"-> P{exit_port} "
            f"= {turn}"
        )

        self.pub_turn.publish(
            String(
                data=turn
            )
        )
        self.publish_status()

    # ==============================================================
    # Kreuzung fertig
    # ==============================================================

    def cb_turn_completed(self, msg):

        if not self.active:
            return

        if self.active_step is None:
            return

        self.edge_started = True

        rospy.loginfo(
            f"Fahre jetzt auf "
            f"{self.active_step['edge_key']}"
        )
        
        self.publish_status()

    # ==============================================================
    # Nächste Stopplinie
    # ==============================================================

    def cb_stopline(self, msg):

        if not msg.data:
            return

        if not self.active:
            return

        if self.active_step is None:
            return

        # Stopplinie der aktuellen Kreuzung ignorieren
        if not self.edge_started:
            return

        edge = self.graph.edges[
            self.active_step[
                "edge_key"
            ]
        ]

        # ----------------------------------------------------------
        # Neue Position übernehmen
        # ----------------------------------------------------------

        self.current_node = (
            self.active_step[
                "to"
            ]
        )

        # Eingangsport am neuen Knoten
        if (
            self.current_node
            == edge["node_a"]
        ):

            self.incoming_port = int(
                edge["port_a"]
            )

        else:

            self.incoming_port = int(
                edge["port_b"]
            )

        rospy.loginfo(
            f"Angekommen bei "
            f"{self.current_node}, "
            f"Port {self.incoming_port}"
        )

        self.active_step = None
        self.edge_started = False

        # nächste Kreuzung
        self.publish_next_turn()

    # ==============================================================
    # Stop-Service
    # ==============================================================

    def srv_stop(self, req):

        self.active = False

        self.route_steps = []

        self.active_step = None

        self.edge_started = False
        
        self.publish_status()

        return TriggerResponse(
            True,
            "Navigation gestoppt",
        )

    # ==============================================================
    # Status
    # ==============================================================
    
    def publish_status(self):
        status = {
            "current_node": self.current_node,
            "incoming_port": self.incoming_port,
            "current_edge": self.active_step["edge_key"] if (self.active_step and self.edge_started) else None,
            "edge_drive_started": self.edge_started,
            "route_steps": [step["edge_key"] for step in self.route_steps],
            "active_step": self.active_step["edge_key"] if self.active_step else None,
            "target_gates": self.target_gates,
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    # ==============================================================
    # Ententanz & Standby
    # ==============================================================

    def perform_duck_dance_and_standby(self):
        rospy.loginfo("Navigation beendet. Pausiere Lane Control und starte Ententanz!")
        
        # Lane Control pausieren, damit wir die Motoren übernehmen können
        self.pub_lane_switch.publish(Int32(0))
        
        start_time = time.time()
        last_wiggle = start_time
        wiggle_dir = -1.0
        
        rate = rospy.Rate(10)
        while time.time() - start_time < 3.0 and not rospy.is_shutdown():
            current_time = time.time()
            if current_time - last_wiggle > 0.2:
                wiggle_dir *= -1.0
                last_wiggle = current_time
                
            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()
            # 0.08 war die wiggle_power im duck_avoidance_node
            cmd.v = 1.0 * 0.08 * wiggle_dir
            cmd.omega = 2.0 * wiggle_dir
            
            self.pub_cmd.publish(cmd)
            rate.sleep()
            
        rospy.loginfo("Ententanz beendet. Gehe in STANDBY-Modus.")
        self.pub_standby.publish(Bool(True))
        
        # Standby: Alles auf 0 setzen
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        
        for _ in range(5):
            if rospy.is_shutdown():
                break
            self.pub_cmd.publish(cmd)
            rospy.sleep(0.1)

    # ==============================================================
    # Start
    # ==============================================================

    def run(self):

        rospy.spin()


if __name__ == "__main__":

    NavigatorNode().run()