#!/usr/bin/env python3

import json
import os
import time
from collections import defaultdict

import matplotlib.pyplot as plt
import networkx as nx

from topological_graph import (
    TopologicalGraph,
    is_gate_tag,
    load_challenge_config,
    turn_from_ports,
)


class OfflineMappingSimulator:
    """
    Offline-Test für das topologische Mapping.

    Die eigentliche Mapping-Logik ist bereits so aufgebaut,
    dass sie später leicht in den ROS-Node übernommen werden kann.

    Simulierte Ereignisse:
    - on_turn_completed()
    - on_tag_detected(tag_id)
    - on_stopline_detected()

    Später entsprechen diese Funktionen den ROS-Callbacks.
    """

    def __init__(self, config_path, output_path):
        self.config_path = config_path
        self.output_path = output_path

        self.config = load_challenge_config(config_path)

        self.graph = TopologicalGraph(
            self.config["graph"],
            default_weight=self.config.get(
                "default_edge_weight",
                1.0,
            ),
        )

        # ----------------------------------------------------------
        # Startposition
        #
        # Aktuell noch aus graph.json.
        # Später kann das z.B. über GUI, ROS-Parameter oder
        # automatische Lokalisierung erfolgen.
        # ----------------------------------------------------------

        self.current_node = self.config["start"]["current_node"]
        self.incoming_port = int(
            self.config["start"]["incoming_port"]
        )

        # ----------------------------------------------------------
        # Mapping-Zustand
        # ----------------------------------------------------------

        self.current_edge = None
        self.next_node = None
        self.next_incoming_port = None

        self.edge_started_at = None

        self.visited_edges = set()
        self.history = []

        # ----------------------------------------------------------
        # NUR FÜR DEN OFFLINE-TEST
        #
        # Diese Werte simulieren später echte Sensordaten.
        #
        # Gate-ID:
        #   später z.B. /<vehicle>/detect/tag_id
        #
        # Fahrzeit:
        #   später aus turn_completed bis stop_line gemessen
        # ----------------------------------------------------------

        self.simulated_world = {
            "A.1--B.1": {
                "gate": 6,
                "travel_time": 4.2,
            },
            "A.4--B.2": {
                "gate": 7,
                "travel_time": 5.1,
            },
            "B.3--C.4": {
                "gate": 8,
                "travel_time": 6.0,
            },
            "A.3--C.1": {
                "gate": 9,
                "travel_time": 4.5,
            },
            "A.2--C.2": {
                "gate": 10,
                "travel_time": 3.8,
            },
        }

        # ----------------------------------------------------------
        # Positionen nur für die grafische Darstellung
        #
        # Diese Koordinaten haben keine Bedeutung für das Mapping.
        # ----------------------------------------------------------

        self.pos = {
            "A": (0, 1),
            "B": (4, 1),
            "C": (2, -2),
        }

        plt.ion()

    # ==============================================================
    # Mapping-Logik
    # ==============================================================

    def mapping_complete(self):
        """
        Mapping ist abgeschlossen, sobald alle eindeutigen Kanten
        mindestens einmal vollständig befahren wurden.
        """

        return (
            len(self.visited_edges)
            == len(self.graph.edges)
        )

    def choose_next_edge(self):
        """
        Wählt am aktuellen Knoten die nächste noch nicht besuchte Kante.

        Direkte U-Turns werden vermieden.

        Diese Funktion ist bewusst einfach gehalten und dient zunächst
        zum Testen des Mapping-Prinzips.
        """

        for exit_port in self.graph.ports(
            self.current_node
        ):

            edge = self.graph.edge_from_port(
                self.current_node,
                exit_port,
            )

            # Bereits gemappte Straße überspringen
            if edge in self.visited_edges:
                continue

            # Direkten U-Turn vermeiden
            if exit_port == self.incoming_port:
                continue

            (
                self.next_node,
                self.next_incoming_port,
                self.current_edge,
            ) = self.graph.neighbor(
                self.current_node,
                exit_port,
            )

            turn = turn_from_ports(
                self.incoming_port,
                exit_port,
            )

            print("\nNeue Kante:")
            print(
                f"{self.current_node} "
                f"-> {self.next_node}"
            )
            print(
                f"Port {self.incoming_port} "
                f"-> Port {exit_port}"
            )
            print(
                f"Manöver: {turn}"
            )
            print(
                f"Kante: {self.current_edge}"
            )

            # ------------------------------------------------------
            # SPÄTER IM ECHTEN ROS-NODE
            #
            # Hier würde die Entscheidung veröffentlicht werden:
            #
            # self.pub_turn.publish(...)
            #
            # Der Intersection-Node führt dann das Manöver aus.
            # ------------------------------------------------------

            return True

        return False

    # ==============================================================
    # Spätere ROS-Ereignisse
    # ==============================================================

    def on_turn_completed(self):
        """
        Später:
        Callback von /intersection/turn_completed

        Ab diesem Zeitpunkt beginnt die Messung der reinen Fahrzeit
        auf der Straße.
        """

        self.edge_started_at = time.monotonic()

        print(
            "[EVENT] Kreuzungsmanöver abgeschlossen"
        )

    def on_tag_detected(self, tag_id):
        """
        Später:
        Callback von /detect/tag_id

        Ein erkanntes Gate wird der aktuell befahrenen Kante
        zugeordnet.
        """

        if self.current_edge is None:
            return

        if not is_gate_tag(tag_id):
            return

        self.graph.set_gate(
            self.current_edge,
            int(tag_id),
        )

        print(
            f"[EVENT] Gate {tag_id} "
            f"auf {self.current_edge}"
        )

        # Zwischenstand speichern und direkt darstellen
        self.save_mapping()
        self.draw_mapping()

    def on_stopline_detected(
        self,
        simulated_travel_time=None,
    ):
        """
        Später:
        Callback von /detect/stop_line

        Die Kante gilt dann als vollständig befahren.

        Im echten System wird die Fahrzeit aus dem Zeitunterschied
        zwischen on_turn_completed() und dieser Funktion berechnet.
        """

        if self.current_edge is None:
            return

        # ----------------------------------------------------------
        # Fahrzeit bestimmen
        # ----------------------------------------------------------

        if simulated_travel_time is None:

            travel_time = (
                time.monotonic()
                - self.edge_started_at
            )

        else:

            # Nur für den Offline-Test
            travel_time = float(
                simulated_travel_time
            )

        self.graph.set_travel_time(
            self.current_edge,
            travel_time,
        )

        self.visited_edges.add(
            self.current_edge
        )

        edge_data = self.graph.edges[
            self.current_edge
        ]

        self.history.append(
            {
                "from": self.current_node,
                "to": self.next_node,
                "edge": self.current_edge,
                "gates": list(
                    edge_data["gates"]
                ),
                "travel_time": travel_time,
            }
        )

        print(
            f"[EVENT] Stopplinie erkannt "
            f"nach {travel_time:.2f} s"
        )

        # ----------------------------------------------------------
        # Duckiebot ist nun an der nächsten Kreuzung angekommen
        # ----------------------------------------------------------

        self.current_node = self.next_node

        self.incoming_port = (
            self.next_incoming_port
        )

        self.current_edge = None

        self.save_mapping()
        self.draw_mapping()

    # ==============================================================
    # NUR OFFLINE-SIMULATION
    # ==============================================================

    def simulate_current_edge(self):
        """
        Simuliert die Ereignisse, die später von echten ROS-Nodes
        eintreffen.

        Reihenfolge:

        1. Kreuzungsmanöver abgeschlossen
        2. Gate wird auf der Straße erkannt
        3. nächste Stopplinie wird erreicht

        Diese Funktion wird später nicht benötigt.
        """

        data = self.simulated_world[
            self.current_edge
        ]

        # ----------------------------------------------------------
        # Später durch echten ROS-Callback:
        #
        # cb_turn_completed(...)
        # ----------------------------------------------------------

        self.on_turn_completed()

        # ----------------------------------------------------------
        # Später durch echten Kamera-Callback:
        #
        # cb_tag(...)
        # ----------------------------------------------------------

        self.on_tag_detected(
            data["gate"]
        )

        # ----------------------------------------------------------
        # Später durch echten Stoplinien-Callback:
        #
        # cb_stopline(...)
        # ----------------------------------------------------------

        self.on_stopline_detected(
            simulated_travel_time=
                data["travel_time"]
        )

    # ==============================================================
    # Mapping speichern
    # ==============================================================

    def save_mapping(self):
        """
        Speichert den aktuellen Mapping-Zustand.

        Das gleiche Prinzip kann später für die echte
        challenge4_mapping.json verwendet werden.
        """

        state = {
            "current_node":
                self.current_node,

            "incoming_port":
                self.incoming_port,
        }

        self.graph.save_mapping(
            self.output_path,
            state,
        )

        # ----------------------------------------------------------
        # Zusätzliche Informationen nur für Debugging und Test
        # ----------------------------------------------------------

        with open(
            self.output_path,
            "r",
            encoding="utf-8",
        ) as file:

            data = json.load(file)

        data["visited_edges"] = sorted(
            self.visited_edges
        )

        data["history"] = self.history

        with open(
            self.output_path,
            "w",
            encoding="utf-8",
        ) as file:

            json.dump(
                data,
                file,
                indent=2,
            )

    # ==============================================================
    # Visualisierung
    # ==============================================================

    def draw_mapping(self):
        """
        Zeichnet den Graphen aus der gespeicherten
        test_mapping_result.json.

        Darstellung:

        - Knoten
        - 5 getrennte Straßen
        - Ports
        - Gate-IDs
        - Fahrzeiten
        - Linienstärke abhängig von Fahrzeit
        - noch unbekannte Kanten gestrichelt

        Wichtig:
        Zwischen A-B und A-C existieren jeweils zwei verschiedene
        Straßen. Diese werden mit unterschiedlicher Krümmung
        dargestellt.
        """

        with open(
            self.output_path,
            "r",
            encoding="utf-8",
        ) as file:

            data = json.load(file)

        plt.clf()

        graph_vis = nx.MultiGraph()

        # ----------------------------------------------------------
        # Knoten hinzufügen
        # ----------------------------------------------------------

        for node in self.graph.adjacency:

            graph_vis.add_node(
                node
            )

        # ----------------------------------------------------------
        # Kanten hinzufügen
        # ----------------------------------------------------------

        for edge_key, edge in data[
            "edges"
        ].items():

            graph_vis.add_edge(
                edge["node_a"],
                edge["node_b"],
                key=edge_key,
            )

        # ----------------------------------------------------------
        # Knoten zeichnen
        # ----------------------------------------------------------

        nx.draw_networkx_nodes(
            graph_vis,
            self.pos,
            node_size=1800,
        )

        nx.draw_networkx_labels(
            graph_vis,
            self.pos,
            font_size=16,
        )

        visited_edges = set(
            data.get(
                "visited_edges",
                []
            )
        )

        # ----------------------------------------------------------
        # Kanten nach Knotenpaar gruppieren
        #
        # Beispiel:
        #
        # A-B:
        #   A.1--B.1
        #   A.4--B.2
        #
        # Diese müssen getrennt gezeichnet werden.
        # ----------------------------------------------------------

        edge_groups = defaultdict(
            list
        )

        for edge_key, edge in data[
            "edges"
        ].items():

            pair = tuple(
                sorted(
                    [
                        edge["node_a"],
                        edge["node_b"],
                    ]
                )
            )

            edge_groups[
                pair
            ].append(
                (
                    edge_key,
                    edge,
                )
            )

        # ----------------------------------------------------------
        # Gruppierte Kanten zeichnen
        # ----------------------------------------------------------

        for pair, grouped_edges in (
            edge_groups.items()
        ):

            # Stabile Reihenfolge
            grouped_edges.sort(
                key=lambda item: item[0]
            )

            number_of_edges = len(
                grouped_edges
            )

            # ------------------------------------------------------
            # Krümmungen für parallele Straßen bestimmen
            # ------------------------------------------------------

            if number_of_edges == 1:

                curvatures = [
                    0.0
                ]

            elif number_of_edges == 2:

                curvatures = [
                    -0.25,
                    0.25,
                ]

            elif number_of_edges == 3:

                curvatures = [
                    -0.35,
                    0.0,
                    0.35,
                ]

            else:

                center = (
                    number_of_edges
                    - 1
                ) / 2

                curvatures = [

                    (
                        index
                        - center
                    ) * 0.15

                    for index in range(
                        number_of_edges
                    )
                ]

            # ------------------------------------------------------
            # Jede Straße einzeln zeichnen
            # ------------------------------------------------------

            for (
                edge_key,
                edge,
            ), curvature in zip(
                grouped_edges,
                curvatures,
            ):

                node_a = edge[
                    "node_a"
                ]

                node_b = edge[
                    "node_b"
                ]

                travel_time = edge.get(
                    "travel_time"
                )

                visited = (
                    edge_key
                    in visited_edges
                )

                # --------------------------------------------------
                # Linienstärke und Stil
                # --------------------------------------------------

                if (
                    visited
                    and travel_time
                    is not None
                ):

                    width = (
                        1.5
                        + travel_time
                        * 0.4
                    )

                    style = "solid"

                else:

                    width = 1.0
                    style = "dashed"

                # --------------------------------------------------
                # Kante zeichnen
                # --------------------------------------------------

                nx.draw_networkx_edges(
                    graph_vis,
                    self.pos,

                    edgelist=[
                        (
                            node_a,
                            node_b,
                            edge_key,
                        )
                    ],

                    width=width,

                    style=style,

                    connectionstyle=(
                        f"arc3,rad="
                        f"{curvature}"
                    ),
                )

                # --------------------------------------------------
                # Position für Beschriftung berechnen
                #
                # Die Beschriftung wird abhängig von der Krümmung
                # seitlich von der direkten Verbindung verschoben.
                # --------------------------------------------------

                x1, y1 = self.pos[
                    node_a
                ]

                x2, y2 = self.pos[
                    node_b
                ]

                middle_x = (
                    x1 + x2
                ) / 2

                middle_y = (
                    y1 + y2
                ) / 2

                dx = x2 - x1
                dy = y2 - y1

                length = (
                    dx ** 2
                    + dy ** 2
                ) ** 0.5

                if length > 0:

                    normal_x = (
                        -dy
                        / length
                    )

                    normal_y = (
                        dx
                        / length
                    )

                else:

                    normal_x = 0
                    normal_y = 0

                # Beschriftung weiter weg von der Mitte verschieben
                # als die eigentliche Krümmung.
                offset = (
                    curvature
                    * 3.0
                )

                label_x = (
                    middle_x
                    + normal_x
                    * offset
                )

                label_y = (
                    middle_y
                    + normal_y
                    * offset
                )

                # --------------------------------------------------
                # Beschriftung erstellen
                # --------------------------------------------------

                ports = (
                    f"P{edge['port_a']} "
                    f"↔ "
                    f"P{edge['port_b']}"
                )

                if visited:

                    gates = ", ".join(
                        map(
                            str,
                            edge.get(
                                "gates",
                                [],
                            ),
                        )
                    )

                    label = (
                        f"{ports}\n"
                        f"Gate {gates}\n"
                        f"{travel_time:.1f}s"
                    )

                else:

                    label = (
                        f"{ports}\n"
                        "unbekannt"
                    )

                plt.text(
                    label_x,
                    label_y,
                    label,

                    horizontalalignment=
                        "center",

                    verticalalignment=
                        "center",

                    bbox=dict(
                        facecolor=
                            "white",

                        alpha=0.85,
                    ),
                )

        # ----------------------------------------------------------
        # Titel
        # ----------------------------------------------------------

        plt.title(
            "Topologisches Mapping\n"
            f"{len(visited_edges)} / "
            f"{len(self.graph.edges)} "
            "Kanten gemappt"
        )

        plt.axis(
            "off"
        )

        plt.tight_layout()

        # Kurze Pause, damit der Aufbau schrittweise sichtbar ist
        plt.pause(
            1.0
        )

    # ==============================================================
    # Hauptablauf
    # ==============================================================

    def run(self):

        print(
            "\nStarte Offline-Mapping"
        )

        print(
            f"Start: "
            f"{self.current_node}, "
            f"Port "
            f"{self.incoming_port}"
        )

        # ----------------------------------------------------------
        # Initialen Mapping-Zustand speichern und darstellen
        # ----------------------------------------------------------

        self.save_mapping()
        self.draw_mapping()

        # ----------------------------------------------------------
        # Schrittweise alle Straßen abfahren
        # ----------------------------------------------------------

        while not self.mapping_complete():

            found_edge = (
                self.choose_next_edge()
            )

            if not found_edge:

                print(
                    "\nKeine weitere "
                    "U-Turn-freie Kante gefunden."
                )

                break

            self.simulate_current_edge()

        # ----------------------------------------------------------
        # Ergebnis
        # ----------------------------------------------------------

        print(
            "\nMapping abgeschlossen."
        )

        print(
            f"{len(self.visited_edges)} / "
            f"{len(self.graph.edges)} "
            "Kanten gemappt."
        )

        print(
            "\nFahrverlauf:"
        )

        for index, entry in enumerate(
            self.history,
            start=1,
        ):

            print(
                f"{index}. "
                f"{entry['from']} "
                f"-> "
                f"{entry['to']} "
                f"| "
                f"{entry['edge']} "
                f"| Gates "
                f"{entry['gates']} "
                f"| "
                f"{entry['travel_time']:.1f}s"
            )

        plt.ioff()

        plt.show()


# ==============================================================
# MAIN
# ==============================================================

if __name__ == "__main__":

    # --------------------------------------------------------------
    # Erwartete Ordnerstruktur:
    #
    # map_and_nav/
    #
    # ├── config/
    # │   ├── graph.json
    # │   └── test_mapping_result.json
    #
    # └── src/
    #     ├── topological_graph.py
    #     └── test_mapping_offline.py
    # --------------------------------------------------------------

    src_dir = os.path.dirname(
        os.path.abspath(
            __file__
        )
    )

    package_dir = os.path.dirname(
        src_dir
    )

    config_path = os.path.join(
        package_dir,
        "config",
        "graph.json",
    )

    output_path = os.path.join(
        package_dir,
        "config",
        "test_mapping_result.json",
    )

    simulator = OfflineMappingSimulator(
        config_path,
        output_path,
    )

    simulator.run()