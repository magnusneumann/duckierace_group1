#!/usr/bin/env python3
import json
import os

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from topological_graph import TopologicalGraph, load_challenge_config, turn_from_ports


class NavigatorNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")

        package_root = os.path.dirname(os.path.dirname(__file__))
        default_config = os.path.join(package_root, "config", "graph.json")
        default_mapping = os.path.join(package_root, "config", "challenge4_mapping.json")

        self.config_path = rospy.get_param("~config_path", default_config)
        self.mapping_path = rospy.get_param("~mapping_path", default_mapping)
        self.config = load_challenge_config(self.config_path)
        self.graph = TopologicalGraph(
            self.config["graph"],
            default_weight=self.config.get("default_edge_weight", 1.0),
        )

        self.current_node = str(rospy.get_param(
            "~start_node",
            self.config.get("start", {}).get("current_node", "A"),
        ))
        self.incoming_port = int(rospy.get_param(
            "~incoming_port",
            self.config.get("start", {}).get("incoming_port", 3),
        ))
        self.target_gates = list(rospy.get_param(
            "~target_gates",
            self.config.get("target_gates", list(range(5, 14))),
        ))
        self.route_steps = []
        self.active_step = None
        self.active_step_started = False
        self.active = False

        self.pub_turn = rospy.Publisher(
            f"/{self._vehicle_name}/intersection/turn_decision",
            String,
            queue_size=1,
            latch=True,
        )
        self.pub_status = rospy.Publisher(
            f"/{self._vehicle_name}/navigation/status",
            String,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(
            f"/{self._vehicle_name}/intersection/turn_completed",
            String,
            self.cb_turn_completed,
            queue_size=1,
        )
        rospy.Subscriber(f"/{self._vehicle_name}/detect/stop_line", Bool, self.cb_stopline, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/navigation/route", String, self.cb_route, queue_size=1)

        rospy.Service(f"/{self._vehicle_name}/navigation/load_mapping", Trigger, self.srv_load_mapping)
        rospy.Service(f"/{self._vehicle_name}/navigation/start", Trigger, self.srv_start)
        rospy.Service(f"/{self._vehicle_name}/navigation/stop", Trigger, self.srv_stop)

        rospy.loginfo("Challenge-4 Navigator bereit.")

    def srv_load_mapping(self, req):
        try:
            state = self.graph.load_mapping(self.mapping_path)
            self.current_node = str(state.get("current_node", self.current_node))
            self.incoming_port = int(state.get("incoming_port", self.incoming_port))
            return TriggerResponse(True, f"Mapping geladen: {self.mapping_path}")
        except Exception as e:
            return TriggerResponse(False, f"Mapping konnte nicht geladen werden: {e}")

    def srv_start(self, req):
        try:
            self.plan_route()
            self.active = True
            self.publish_next_turn()
            return TriggerResponse(True, f"Navigation gestartet: {self.target_gates}")
        except Exception as e:
            self.active = False
            return TriggerResponse(False, f"Navigation konnte nicht starten: {e}")

    def srv_stop(self, req):
        self.active = False
        self.route_steps = []
        self.active_step = None
        self.active_step_started = False
        self.publish_status("Navigation gestoppt")
        return TriggerResponse(True, "Navigation gestoppt")

    def cb_route(self, msg):
        try:
            self.target_gates = [int(value) for value in json.loads(msg.data)]
            self.plan_route()
            self.active = True
            self.publish_next_turn()
        except Exception as e:
            rospy.logerr(f"Route konnte nicht verarbeitet werden: {e}")

    def plan_route(self):
        self.route_steps = []
        planning_node = self.current_node
        # Die Gate-Reihenfolge bleibt vorgegeben; nur die jeweils schnellste Anfahrt
        # durch den gemappten, gewichteten Graphen wird berechnet.
        for gate_id in self.target_gates:
            _, steps = self.graph.route_to_gate(planning_node, int(gate_id))
            self.route_steps.extend(steps)
            if steps:
                planning_node = steps[-1]["to"]

        self.publish_status(json.dumps({
            "current_node": self.current_node,
            "incoming_port": self.incoming_port,
            "target_gates": self.target_gates,
            "steps": self.route_steps,
        }))

    def publish_next_turn(self):
        if not self.active or not self.route_steps:
            self.active_step = None
            self.publish_status("Navigation fertig")
            return

        self.active_step = self.route_steps.pop(0)
        self.active_step_started = False
        turn = turn_from_ports(self.incoming_port, self.active_step["exit_port"])
        if turn == "u_turn":
            rospy.logwarn("U-Turn geplant. CrossIntersectionNode kann das aktuell nicht direkt ausfuehren.")
        self.pub_turn.publish(String(data=turn))
        self.publish_status(json.dumps({
            "next_turn": turn,
            "from": self.active_step["from"],
            "to": self.active_step["to"],
            "edge": self.active_step["edge_key"],
            "target_gate": self.active_step.get("target_gate"),
            "remaining_steps": len(self.route_steps),
        }))

    def cb_turn_completed(self, msg):
        if self.active and self.active_step is not None:
            self.active_step_started = True

    def cb_stopline(self, msg):
        if not msg.data:
            return
        if not self.active or self.active_step is None or not self.active_step_started:
            return
        edge = self.graph.edges[self.active_step["edge_key"]]
        self.current_node = self.active_step["to"]
        # Nach Erreichen der Stopplinie ist der Zielport der Eingangsport fuer
        # die Abbiegentscheidung an der neuen Kreuzung.
        if self.current_node == edge["node_a"]:
            self.incoming_port = edge["port_a"]
        else:
            self.incoming_port = edge["port_b"]
        self.active_step_started = False
        self.publish_next_turn()

    def publish_status(self, text):
        self.pub_status.publish(String(data=str(text)))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = NavigatorNode("navigator_node")
    node.run()
