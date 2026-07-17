#!/usr/bin/env python3
import json
import os

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from topological_graph import TopologicalGraph, is_gate_tag, load_challenge_config, turn_from_ports


class TopologicalMappingNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")

        package_root = os.path.dirname(os.path.dirname(__file__))
        default_config = os.path.join(package_root, "config", "challenge4_graph.json")
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
        self.current_edge = None
        self.expected_next_node = None
        self.edge_started_at = None
        self.edge_drive_started = False
        self.active = False
        self.visited_edges = set()
        self.pending_exit_ports = []

        self.pub_turn = rospy.Publisher(
            f"/{self._vehicle_name}/intersection/turn_decision",
            String,
            queue_size=1,
            latch=True,
        )
        self.pub_status = rospy.Publisher(
            f"/{self._vehicle_name}/mapping/status",
            String,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber(f"/{self._vehicle_name}/detect/tag_id", String, self.cb_tag, queue_size=1)
        rospy.Subscriber(
            f"/{self._vehicle_name}/intersection/turn_completed",
            String,
            self.cb_turn_completed,
            queue_size=1,
        )
        rospy.Subscriber(f"/{self._vehicle_name}/detect/stop_line", Bool, self.cb_stopline, queue_size=1)

        rospy.Service(f"/{self._vehicle_name}/mapping4/start", Trigger, self.srv_start)
        rospy.Service(f"/{self._vehicle_name}/mapping4/export", Trigger, self.srv_export)
        rospy.Service(f"/{self._vehicle_name}/mapping4/stop", Trigger, self.srv_stop)

        rospy.loginfo("Challenge-4 Topological Mapping bereit.")

    def srv_start(self, req):
        self.active = True
        self.current_edge = None
        self.expected_next_node = None
        self.edge_started_at = None
        self.edge_drive_started = False
        self.visited_edges = set()
        self.pending_exit_ports = []
        self.choose_next_edge()
        return TriggerResponse(True, "Topologisches Mapping gestartet")

    def srv_stop(self, req):
        self.active = False
        return TriggerResponse(True, "Topologisches Mapping gestoppt")

    def srv_export(self, req):
        try:
            self.graph.save_mapping(self.mapping_path, self.state_dict())
            return TriggerResponse(True, f"Mapping gespeichert: {self.mapping_path}")
        except Exception as e:
            return TriggerResponse(False, f"Mapping konnte nicht gespeichert werden: {e}")

    def cb_tag(self, msg):
        if not self.active or not is_gate_tag(msg.data) or self.current_edge is None:
            return
        self.graph.set_gate(self.current_edge, int(msg.data))
        self.publish_status({"mapped_gate": int(msg.data), "edge": self.current_edge})

    def cb_turn_completed(self, msg):
        if self.active and self.current_edge is not None:
            self.edge_started_at = rospy.Time.now()
            self.edge_drive_started = True

    def cb_stopline(self, msg):
        if not msg.data:
            return
        if not self.active or self.current_edge is None or not self.edge_drive_started:
            return

        elapsed = (rospy.Time.now() - self.edge_started_at).to_sec() if self.edge_started_at else None
        if elapsed and elapsed > 0.1:
            self.graph.set_travel_time(self.current_edge, elapsed)
        self.visited_edges.add(self.current_edge)

        edge = self.graph.edges[self.current_edge]
        self.current_node = self.expected_next_node
        if self.current_node == edge["node_a"]:
            self.incoming_port = edge["port_a"]
        else:
            self.incoming_port = edge["port_b"]

        self.current_edge = None
        self.expected_next_node = None
        self.edge_started_at = None
        self.edge_drive_started = False
        self.choose_next_edge()

    def choose_next_edge(self):
        ports = self.graph.ports(self.current_node)
        if not ports:
            self.active = False
            self.publish_status({"done": True, "reason": "current node has no ports"})
            return

        exit_port = self.next_exit_port()
        if exit_port is None:
            self.active = False
            self.publish_status({
                "done": True,
                "reason": "all graph edges visited",
                "visited_edges": len(self.visited_edges),
                "total_edges": len(self.graph.edges),
            })
            return

        next_node, next_incoming_port, edge_key = self.graph.neighbor(self.current_node, exit_port)
        self.current_edge = edge_key
        self.expected_next_node = next_node
        self.edge_started_at = None
        self.edge_drive_started = False

        turn = turn_from_ports(self.incoming_port, exit_port)
        self.pub_turn.publish(String(data=turn))
        self.publish_status({
            "current_node": self.current_node,
            "incoming_port": self.incoming_port,
            "exit_port": exit_port,
            "turn": turn,
            "current_edge": self.current_edge,
            "expected_next_node": self.expected_next_node,
            "expected_next_incoming_port": next_incoming_port,
            "visited_edges": len(self.visited_edges),
            "total_edges": len(self.graph.edges),
        })

    def next_exit_port(self):
        if self.pending_exit_ports:
            return self.pending_exit_ports.pop(0)

        for port in self.graph.ports(self.current_node):
            candidate = self.graph.edge_from_port(self.current_node, port)
            if candidate not in self.visited_edges:
                return port

        best_path = None
        best_cost = None
        for node in self.graph.adjacency.keys():
            has_unvisited = any(
                self.graph.edge_from_port(node, port) not in self.visited_edges
                for port in self.graph.ports(node)
            )
            if not has_unvisited or node == self.current_node:
                continue
            node_path = self.graph.path_to_node(self.current_node, node)
            if node_path is None:
                continue
            cost, steps = node_path
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_path = steps

        if not best_path:
            return None

        self.pending_exit_ports = [step["exit_port"] for step in best_path]
        return self.pending_exit_ports.pop(0)

    def state_dict(self):
        return {
            "current_node": self.current_node,
            "incoming_port": self.incoming_port,
            "current_edge": self.current_edge,
            "expected_next_node": self.expected_next_node,
        }

    def publish_status(self, value):
        self.pub_status.publish(String(data=json.dumps(value, sort_keys=True)))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = TopologicalMappingNode("mapping_topological_node")
    node.run()
