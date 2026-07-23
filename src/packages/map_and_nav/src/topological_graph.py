#!/usr/bin/env python3
import heapq
import json
import os
import time
from dataclasses import dataclass


GATE_TAG_MIN = 5
GATE_TAG_MAX = 13


@dataclass(frozen=True)
class EdgeKey:
    node_a: str
    port_a: int
    node_b: str
    port_b: int

    @classmethod
    def make(cls, node_a, port_a, node_b, port_b):
        # Kanonische Sortierung: Beide Fahrtrichtungen bezeichnen dieselbe ungerichtete Kante.
        left = (str(node_a), int(port_a))
        right = (str(node_b), int(port_b))
        if right < left:
            left, right = right, left
        return cls(left[0], left[1], right[0], right[1])

    @classmethod
    def parse(cls, value):
        left, right = value.split("--", 1)
        node_a, port_a = left.rsplit(".", 1)
        node_b, port_b = right.rsplit(".", 1)
        return cls.make(node_a, int(port_a), node_b, int(port_b))

    def __str__(self):
        return f"{self.node_a}.{self.port_a}--{self.node_b}.{self.port_b}"

    def as_dict(self):
        return {
            "node_a": self.node_a,
            "port_a": self.port_a,
            "node_b": self.node_b,
            "port_b": self.port_b,
        }


class TopologicalGraph:
    def __init__(self, graph_input, default_weight=1.0):
        self.default_weight = float(default_weight)
        self.adjacency = {}
        self.edges = {}
        self._parse(graph_input)

    def _parse(self, graph_input):
        for node, ports in graph_input.items():
            node = str(node)
            self.adjacency.setdefault(node, {})
            for raw_port, target in ports.items():
                port = int(raw_port)
                neighbor, neighbor_port = target
                neighbor = str(neighbor)
                neighbor_port = int(neighbor_port)
                key = EdgeKey.make(node, port, neighbor, neighbor_port)
                self.adjacency[node][port] = {
                    "to_node": neighbor,
                    "to_port": neighbor_port,
                    "edge_key": str(key),
                }
                self.edges.setdefault(str(key), {
                    **key.as_dict(),
                    "gates": [],
                    "travel_time": None,
                    "samples": 0,
                })

        for key, edge in self.edges.items():
            a_ok = edge["port_a"] in self.adjacency.get(edge["node_a"], {})
            b_ok = edge["port_b"] in self.adjacency.get(edge["node_b"], {})
            if not a_ok or not b_ok:
                raise ValueError(f"Graph edge {key} is not bidirectionally defined")

    def ports(self, node):
        return sorted(self.adjacency.get(str(node), {}).keys())

    def edge_from_port(self, node, port):
        return self.adjacency[str(node)][int(port)]["edge_key"]

    def neighbor(self, node, exit_port):
        data = self.adjacency[str(node)][int(exit_port)]
        return data["to_node"], data["to_port"], data["edge_key"]

    def weight(self, edge_key):
        value = self.edges[str(edge_key)].get("travel_time")
        if value is None:
            return self.default_weight
        return float(value)

    def set_gate(self, edge_key, gate_id):
        edge = self.edges[str(edge_key)]
        gate_id = int(gate_id)
        if gate_id not in edge["gates"]:
            edge["gates"].append(gate_id)
            edge["gates"].sort()

    def set_travel_time(self, edge_key, elapsed):
        edge = self.edges[str(edge_key)]
        elapsed = float(elapsed)
        old = edge.get("travel_time")
        samples = int(edge.get("samples", 0))
        if old is None:
            edge["travel_time"] = elapsed
        else:
            # Laufender Mittelwert glaettet Messabweichungen aus mehreren Befahrungen.
            edge["travel_time"] = ((old * samples) + elapsed) / (samples + 1)
        edge["samples"] = samples + 1

    def gates_to_edges(self):
        result = {}
        for key, edge in self.edges.items():
            for gate_id in edge.get("gates", []):
                result[int(gate_id)] = key
        return result

    def dijkstra(self, start_node, start_incoming_port=None, allowed_gates=None):
        start_node = str(start_node)
        if start_incoming_port is not None:
            start_incoming_port = int(start_incoming_port)
        start_state = (start_node, start_incoming_port)
        
        dist = {start_state: 0.0}
        previous = {}
        queue = [(0.0, start_state)]
        
        while queue:
            cost, state = heapq.heappop(queue)
            node, inc_port = state
            
            if cost != dist[state]:
                continue
                
            for exit_port, data in self.adjacency.get(node, {}).items():
                exit_port = int(exit_port)
                # U-Turn verbieten
                if inc_port is not None and exit_port == inc_port:
                    continue
                    
                edge_key = data["edge_key"]
                
                if allowed_gates is not None:
                    gates_on_edge = self.edges[edge_key].get("gates", [])
                    # If edge has gates, all of them must be in allowed_gates
                    if gates_on_edge and any(int(g) not in allowed_gates for g in gates_on_edge):
                        continue
                        
                next_node = data["to_node"]
                next_inc_port = int(data["to_port"])
                
                next_cost = cost + self.weight(edge_key)
                next_state = (next_node, next_inc_port)
                
                if next_cost < dist.get(next_state, float("inf")):
                    dist[next_state] = next_cost
                    previous[next_state] = (state, exit_port, edge_key)
                    heapq.heappush(queue, (next_cost, next_state))
                    
        return dist, previous

    def path_to_node(self, start_node, goal_node, start_incoming_port=None, allowed_gates=None):
        start_node = str(start_node)
        goal_node = str(goal_node)
        
        if start_incoming_port is not None:
            start_incoming_port = int(start_incoming_port)
        start_state = (start_node, start_incoming_port)
        
        dist, previous = self.dijkstra(start_node, start_incoming_port, allowed_gates)
        
        # Finde den besten Zielzustand (beliebiger incoming_port)
        best_goal_state = None
        best_cost = float("inf")
        for state, cost in dist.items():
            if state[0] == goal_node and cost < best_cost:
                best_cost = cost
                best_goal_state = state
                
        if best_goal_state is None:
            return None
            
        steps = []
        state = best_goal_state
        while state != start_state:
            prev_state, exit_port, edge_key = previous[state]
            steps.append({
                "from": prev_state[0],
                "to": state[0],
                "exit_port": exit_port,
                "edge_key": edge_key,
            })
            state = prev_state
            
        steps.reverse()
        return best_cost, steps

    def route_to_gate(self, start_node, gate_id, start_incoming_port=None, allowed_gates=None):
        gate_edge = self.gates_to_edges().get(int(gate_id))
        if gate_edge is None:
            raise ValueError(f"Gate {gate_id} is not mapped to any edge")

        edge = self.edges[gate_edge]
        candidates = []
        # Ein Tor liegt auf einer ungerichteten Kante und kann daher von beiden Enden
        # angefahren werden; die guenstigere Richtung wird per Dijkstra ausgewaehlt.
        for approach_node, exit_port, target_node in (
            (edge["node_a"], edge["port_a"], edge["node_b"]),
            (edge["node_b"], edge["port_b"], edge["node_a"]),
        ):
            node_path = self.path_to_node(start_node, approach_node, start_incoming_port, allowed_gates)
            if node_path is None:
                continue
            cost, steps = node_path
            total = cost + self.weight(gate_edge)
            candidates.append((total, steps + [{
                "from": approach_node,
                "to": target_node,
                "exit_port": exit_port,
                "edge_key": gate_edge,
                "target_gate": int(gate_id),
            }]))

        if not candidates:
            raise ValueError(f"No route from {start_node} to gate {gate_id}")
        candidates.sort(key=lambda item: item[0])
        return candidates[0]

    def to_data(self):
        return {
            "default_edge_weight": self.default_weight,
            "edges": self.edges,
            "graph": self.adjacency,
        }

    def save_mapping(self, path, state=None):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = self.to_data()
        data["state"] = state or {}
        data["updated_at"] = time.time()
        with open(path, "w") as f:
            json.dump(data, f, indent=2, sort_keys=True)

    def load_mapping(self, path):
        with open(path, "r") as f:
            data = json.load(f)
        for key, edge_data in data.get("edges", {}).items():
            if key in self.edges:
                self.edges[key].update(edge_data)
        return data.get("state", {})


def turn_from_ports(entry_port, exit_port):
    delta = (int(exit_port) - int(entry_port)) % 4
    if delta == 1:
        return "right"
    if delta == 2:
        return "straight"
    if delta == 3:
        return "left"
    return "u_turn"


def is_gate_tag(tag_id):
    try:
        tag_id = int(tag_id)
    except (TypeError, ValueError):
        return False
    return GATE_TAG_MIN <= tag_id <= GATE_TAG_MAX


def load_challenge_config(path):
    with open(path, "r") as f:
        return json.load(f)
