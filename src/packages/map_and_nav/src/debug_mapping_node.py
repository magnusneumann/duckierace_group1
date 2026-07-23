#!/usr/bin/env python3
import os
import json
import rospy
from std_msgs.msg import String
import matplotlib
matplotlib.use('Agg') # use non-interactive backend
import matplotlib.pyplot as plt
import networkx as nx
from collections import defaultdict
import cv2
import numpy as np

class DebugMappingNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        
        # Load the graph
        self.package_root = os.path.dirname(os.path.dirname(__file__))
        config_path = os.path.join(self.package_root, "config", "graph.json")
        with open(config_path, "r") as f:
            self.graph_input = json.load(f)
            
        self.edges = {}
        for node, ports in self.graph_input["graph"].items():
            for port, target in ports.items():
                node_b, port_b = target
                # canonical key
                left = (str(node), int(port))
                right = (str(node_b), int(port_b))
                if right < left:
                    left, right = right, left
                key = f"{left[0]}.{left[1]}--{right[0]}.{right[1]}"
                self.edges[key] = {
                    "node_a": left[0], "port_a": left[1],
                    "node_b": right[0], "port_b": right[1]
                }
                
        self.pos = {
            "C": (0, 0),
            "A": (-4, -4),    # vorher (4, -4)
            "B": (-8, -2),    # vorher (8, -2)
            "D": (-4, 0),     # vorher (4, 0)
            "E": (0, 8),
            "F": (-4, 8),     # vorher (4, 8)
            "G": (-8, 8),     # vorher (8, 8)
            "H": (0, 12),
            "I": (-4, 12)     # vorher (4, 12)
        }
        
        self.current_node = None
        self.current_edge = None
        self.edge_drive_started = False
        self.visited_edges = []
        self.edge_to_gates = defaultdict(list)
        
        rospy.Subscriber(f"/{self._vehicle_name}/mapping/status", String, self.cb_status, queue_size=1)
        
        self.fig = plt.figure(figsize=(10, 8))
        
    def cb_status(self, msg):
        try:
            status = json.loads(msg.data)
            self.current_node = status.get("current_node")
            self.current_edge = status.get("current_edge")
            self.edge_drive_started = status.get("edge_drive_started", False)
            self.visited_edges = status.get("visited_edges", [])
            
            mapped = status.get("mapped_edges", {})
            self.edge_to_gates.clear()
            for gate, edge in mapped.items():
                self.edge_to_gates[edge].append(gate)
        except Exception as e:
            pass

    def render(self):
        self.fig.clf()
        ax = self.fig.gca()
        graph_vis = nx.MultiGraph()
        
        for node in self.graph_input["graph"].keys():
            graph_vis.add_node(str(node))
            
        for edge_key, edge in self.edges.items():
            graph_vis.add_edge(edge["node_a"], edge["node_b"], key=edge_key)
            
        # Draw nodes
        node_colors = []
        for node in graph_vis.nodes():
            if node == self.current_node and not self.edge_drive_started:
                node_colors.append('red')
            else:
                node_colors.append('#1f78b4') # default blue
                
        nx.draw_networkx_nodes(graph_vis, self.pos, node_size=2000, node_color=node_colors, ax=ax)
        nx.draw_networkx_labels(graph_vis, self.pos, font_size=18, font_color="white", font_weight="bold", ax=ax)
        
        # Draw edges
        edge_groups = defaultdict(list)
        for edge_key, edge in self.edges.items():
            pair = tuple(sorted([edge["node_a"], edge["node_b"]]))
            edge_groups[pair].append((edge_key, edge))
            
        for pair, grouped_edges in edge_groups.items():
            grouped_edges.sort(key=lambda item: item[0])
            n_edges = len(grouped_edges)
            if n_edges == 1:
                curvatures = [0.0]
            elif n_edges == 2:
                curvatures = [-0.25, 0.25]
            elif n_edges == 3:
                curvatures = [-0.35, 0.0, 0.35]
            else:
                center = (n_edges - 1) / 2
                curvatures = [(i - center) * 0.15 for i in range(n_edges)]
                
            for (edge_key, edge), curvature in zip(grouped_edges, curvatures):
                visited = edge_key in self.visited_edges
                is_active = (edge_key == self.current_edge and self.edge_drive_started)
                
                width = 3.0 if is_active else (2.0 if visited else 1.0)
                style = "solid" if visited else "dashed"
                color = "red" if is_active else ("green" if visited else "black")
                
                nx.draw_networkx_edges(
                    graph_vis, self.pos,
                    edgelist=[(edge["node_a"], edge["node_b"], edge_key)],
                    width=width, style=style, edge_color=color,
                    connectionstyle=f"arc3,rad={curvature}",
                    arrows=True,
                    ax=ax
                )
                
                # Draw edge labels
                x1, y1 = self.pos[edge["node_a"]]
                x2, y2 = self.pos[edge["node_b"]]
                middle_x = (x1 + x2) / 2
                middle_y = (y1 + y2) / 2
                dx = x2 - x1
                dy = y2 - y1
                length = (dx ** 2 + dy ** 2) ** 0.5
                if length > 0:
                    normal_x = -dy / length
                    normal_y = dx / length
                else:
                    normal_x = 0
                    normal_y = 0
                    
                offset = curvature * 2.0
                label_x = middle_x + normal_x * offset
                label_y = middle_y + normal_y * offset
                
                gates = self.edge_to_gates.get(edge_key, [])
                gate_str = ", ".join(str(g) for g in sorted(gates))
                if is_active:
                    label = f"{edge_key}\nACTIVE"
                elif visited:
                    label = f"{edge_key}\nGate: {gate_str}" if gate_str else f"{edge_key}"
                else:
                    label = f"{edge_key}\nunbekannt"
                    
                ax.text(label_x, label_y, label, horizontalalignment="center", verticalalignment="center", 
                        bbox=dict(facecolor="white", alpha=0.9, edgecolor="gray"), fontsize=10)

        ax.axis("off")
        self.fig.tight_layout()
        
        # Save the current graph state as PNG to the config folder
        save_path = os.path.join(self.package_root, "config", "challenge4_mapping_graph.png")
        try:
            self.fig.savefig(save_path, dpi=100, bbox_inches='tight')
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"Konnte Graphen nicht speichern: {e}")
            
        self.fig.canvas.draw()
        img = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        cv2.imshow("Mapping Graph", img)
        cv2.waitKey(50)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.render()
            rate.sleep()

if __name__ == "__main__":
    node = DebugMappingNode("debug_mapping_node")
    node.run()
