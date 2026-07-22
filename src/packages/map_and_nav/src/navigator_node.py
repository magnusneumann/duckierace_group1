#!/usr/bin/env python3

import json
import os

import rospy
from std_msgs.msg import Bool, String
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
        )

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

        self.route_steps = []

        planning_node = (
            self.current_node
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

        return TriggerResponse(
            True,
            "Navigation gestoppt",
        )

    # ==============================================================
    # Start
    # ==============================================================

    def run(self):

        rospy.spin()


if __name__ == "__main__":

    NavigatorNode().run()