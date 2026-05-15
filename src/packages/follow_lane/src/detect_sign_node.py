#!/usr/bin/env python3

# Erkennt AprilTags an Kreuzungen und leitet daraus eine Fahrentscheidung ab.
# - empfängt AprilTag-Detektionen
# - wählt den besten gültigen Tag aus
# - bestimmt erlaubte Fahrtrichtungen
# - wählt zufällig links/rechts/geradeaus
# - veröffentlicht Entscheidung über ROS Topics
# - Cooldown verhindert Mehrfachauslösung
# Ziel: automatische Kreuzungsentscheidung für den Duckiebot

import os
import random
from dataclasses import dataclass
from typing import Dict, List, Optional

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String


@dataclass(frozen=True)
class IntersectionSign:
    name: str
    allowed_directions: List[str]


class DetectSignNode:
    DIRECTIONS = {"left", "straight", "right"}

# Relevante AprilTags mit ID und erlaubten Fahrtrichtungen

# april.tag.Tag36h11 id=...

# ID=8: links, geradeaus, rechts
# ID=9: geradeaus, rechts
# ID=10: links, geradeaus

    DEFAULT_SIGN_RULES: Dict[int, IntersectionSign] = {
        8: IntersectionSign("all_directions", ["left", "straight", "right"]),
        9: IntersectionSign("straight_or_right", ["straight", "right"]),
        10: IntersectionSign("left_or_straight", ["left", "straight"]),
    }

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self.tag_rules = self.load_tag_rules()
        self.cooldown_seconds = float(rospy.get_param("~cooldown_seconds", 5.0))
        self.min_decision_margin = float(rospy.get_param("~min_decision_margin", 20.0))
        self.last_decision_time = rospy.Time(0)
        self.last_tag_id: Optional[int] = None

        detections_topic = rospy.get_param(
            "~detections_topic",
            f"/{self._vehicle_name}/apriltag_detector_node/detections",
        )
        decision_topic = rospy.get_param(
            "~decision_topic",
            f"/{self._vehicle_name}/intersection/turn_decision",
        )
        sign_topic = rospy.get_param(
            "~sign_topic",
            f"/{self._vehicle_name}/detect/sign",
        )
        test_output_topic = rospy.get_param(
            "~test_output_topic",
            f"/{self._vehicle_name}/debug/sign_decision",
        )

        self.sub_detections = rospy.Subscriber(
            detections_topic,
            AprilTagDetectionArray,
            self.cb_detections,
            queue_size=1,
        )
        self.pub_decision = rospy.Publisher(decision_topic, String, queue_size=1, latch=True)
        self.pub_sign = rospy.Publisher(sign_topic, String, queue_size=1)
        self.pub_test_output = rospy.Publisher(test_output_topic, String, queue_size=1)

        rospy.loginfo(
            "detect_sign_node listening on %s and publishing decisions on %s. Test output on %s",
            detections_topic,
            decision_topic,
            test_output_topic,
        )

    def load_tag_rules(self) -> Dict[int, IntersectionSign]:
        raw_rules = rospy.get_param("~tag_rules", None)
        if raw_rules is None:
            return self.DEFAULT_SIGN_RULES

        tag_rules = {}
        for raw_tag_id, rule in raw_rules.items():
            tag_id = int(raw_tag_id)
            name = rule.get("name", f"tag_{tag_id}")
            allowed = rule.get("allowed_directions", [])
            valid_allowed = [direction for direction in allowed if direction in self.DIRECTIONS]

            if not valid_allowed:
                rospy.logwarn("Ignoring tag %s because it has no valid directions: %s", tag_id, allowed)
                continue

            tag_rules[tag_id] = IntersectionSign(name, valid_allowed)

        return tag_rules

    def cb_detections(self, msg):
        detection = self.select_best_known_detection(msg)
        if detection is None:
            return

        now = rospy.Time.now()
        if self.is_in_cooldown(now, detection.tag_id):
            return

        sign = self.tag_rules[detection.tag_id]
        decision = random.choice(sign.allowed_directions)

        self.last_decision_time = now
        self.last_tag_id = detection.tag_id

        self.pub_sign.publish(String(data=sign.name))
        self.pub_decision.publish(String(data=decision))
        self.publish_test_output(detection.tag_id, decision)

        rospy.loginfo(
            "Detected sign '%s' via AprilTag %s. Allowed=%s, selected=%s",
            sign.name,
            detection.tag_id,
            sign.allowed_directions,
            decision,
        )

    def select_best_known_detection(self, msg):
        known_detections = [
            detection
            for detection in msg.detections
            if detection.tag_id in self.tag_rules
            and detection.decision_margin >= self.min_decision_margin
        ]

        if not known_detections:
            return None

        return max(known_detections, key=lambda detection: detection.decision_margin)

    def publish_test_output(self, tag_id, decision):
        test_message = f"AprilTag ID={tag_id} -> random direction={decision}"
        self.pub_test_output.publish(String(data=test_message))
        rospy.loginfo("[SIGN TEST] %s", test_message)

    def is_in_cooldown(self, now, tag_id):
        if self.last_tag_id != tag_id:
            return False

        elapsed = (now - self.last_decision_time).to_sec()
        return elapsed < self.cooldown_seconds

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = DetectSignNode("detect_sign_node")
    node.run()
