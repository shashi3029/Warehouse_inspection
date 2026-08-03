#!/usr/bin/env python3
"""
Mission Planner Node

Converts natural language user prompts into structured mission.json format.
Supports all warehouse operation types: patrol, inspection, inventory scan,
package verification, pickup, delivery, tracking, formation, and return home.
"""

import json
import os
import uuid
import re
from datetime import datetime, timezone
from typing import Dict, List, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from warehouse_msgs.msg import MissionStatus


ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]

SHELF_LOCATIONS = ["Shelf_A", "Shelf_B", "Shelf_C", "Shelf_D"]

PATROL_WAYPOINTS = [
    "Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4",
    "Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8",
]

HOME_POSITIONS = ["Home_1", "Home_2", "Home_3", "Home_4"]

CHARGING_STATIONS = ["Charging_Station_1", "Charging_Station_2"]

WAREHOUSE_ZONES = ["Zone_A", "Zone_B", "Zone_C", "Zone_D"]


class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__("mission_planner_node")

        self.declare_parameter("output_dir", "/tmp/warehouse_missions")
        self.declare_parameter("default_priority", 5)
        self.declare_parameter("battery_threshold", 20)
        self.declare_parameter("loop_missions", True)
        self.declare_parameter("default_speed", 0.3)

        self._output_dir = self.get_parameter("output_dir").value
        self._default_priority = self.get_parameter("default_priority").value
        self._battery_threshold = self.get_parameter("battery_threshold").value
        self._loop_missions = self.get_parameter("loop_missions").value
        self._default_speed = self.get_parameter("default_speed").value

        os.makedirs(self._output_dir, exist_ok=True)

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._mission_pub = self.create_publisher(String, "/warehouse/mission_json", latching_qos)
        self._status_pub = self.create_publisher(
            MissionStatus, "/warehouse/mission_planner/status", 10
        )

        self._prompt_sub = self.create_subscription(
            String, "/warehouse/user_prompt", self._prompt_callback, 10
        )

        self._mission_counter = 0
        self.get_logger().info("Mission Planner ready — waiting for prompts on /warehouse/user_prompt")

    def _prompt_callback(self, msg: String) -> None:
        prompt = msg.data.strip()
        if not prompt:
            return

        self.get_logger().info(f'Received prompt: "{prompt}"')
        try:
            mission = self.generate_mission(prompt)
            self._publish_and_save(mission, prompt)
        except Exception as exc:
            self.get_logger().error(f"Mission generation failed: {exc}")

    def generate_mission(self, prompt: str) -> Dict[str, Any]:
        p = prompt.lower()

        if self._matches(p, ["return home", "go home", "all robots home", "recall"]):
            return self._gen_return_home(prompt)
        if self._matches(p, ["charge", "recharge", "charging station"]):
            return self._gen_charging_mission(prompt)
        if self._matches(p, ["emergency", "alarm", "alert"]):
            return self._gen_emergency_inspection(prompt)
        if self._matches(p, ["formation"]):
            return self._gen_formation_mission(prompt, self._parse_formation_type(p))
        if self._matches(p, ["track", "follow", "moving package", "target"]):
            return self._gen_tracking_mission(prompt)
        if self._matches(p, ["inventory scan", "inventory check", "stock check", "inventory"]):
            return self._gen_inventory_scan(prompt)
        if self._matches(p, ["verify packages", "verify", "package verification", "check packages"]):
            return self._gen_package_verification(prompt)
        if self._matches(p, ["pickup", "pick up", "collect"]):
            return self._gen_package_pickup(prompt)
        if self._matches(p, ["deliver", "delivery", "drop off"]):
            return self._gen_package_delivery(prompt)
        if self._matches(p, ["inspect all shelves", "all shelves", "inspect all"]):
            return self._gen_full_inspection(prompt)
        if self._matches(p, ["inspect shelf a", "shelf a inspection", "check shelf a"]):
            return self._gen_shelf_inspection(prompt, ["Shelf_A"])
        if self._matches(p, ["inspect shelf b", "shelf b inspection", "check shelf b"]):
            return self._gen_shelf_inspection(prompt, ["Shelf_B"])
        if self._matches(p, ["inspect shelf c", "shelf c inspection", "check shelf c"]):
            return self._gen_shelf_inspection(prompt, ["Shelf_C"])
        if self._matches(p, ["inspect shelf d", "shelf d inspection", "check shelf d"]):
            return self._gen_shelf_inspection(prompt, ["Shelf_D"])
        if self._matches(p, ["inspect", "inspection"]):
            return self._gen_full_inspection(prompt)
        if self._matches(p, ["patrol", "patrol warehouse", "perimeter", "sweep"]):
            return self._gen_patrol(prompt)
        if self._matches(p, ["monitor", "monitoring", "continuous monitoring", "surveillance"]):
            return self._gen_continuous_monitoring(prompt)

        return self._gen_comprehensive_inspection(prompt)

    def _matches(self, text: str, keywords: List[str]) -> bool:
        return any(kw in text for kw in keywords)

    def _parse_formation_type(self, prompt: str) -> str:
        for formation in ["square", "diamond", "column", "line", "escort", "adaptive"]:
            if formation in prompt:
                return formation
        return "square"

    def _base_mission(
        self,
        name: str,
        mission_type: str,
        priority: int = 5,
        loop: Optional[bool] = None,
    ) -> Dict[str, Any]:
        return {
            "mission_id": str(uuid.uuid4()),
            "mission_name": name,
            "mission_type": mission_type,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "priority": priority,
            "loop": loop if loop is not None else self._loop_missions,
            "battery_threshold": self._battery_threshold,
            "resume_after_charge": True,
            "default_speed": self._default_speed,
            "commands": [],
        }

    def _gen_patrol(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission("Warehouse Patrol", "WAREHOUSE_PATROL", priority=5, loop=True)
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 5,
                "parameters": {
                    "waypoints": ["Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4"],
                    "loop": True,
                    "speed": self._default_speed,
                    "zone": "Zone_A",
                },
            },
            {
                "robot": "Robot_2",
                "task": "patrol",
                "goal": "Waypoint_5",
                "priority": 5,
                "parameters": {
                    "waypoints": ["Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8"],
                    "loop": True,
                    "speed": self._default_speed,
                    "zone": "Zone_B",
                },
            },
            {
                "robot": "Robot_3",
                "task": "inspect",
                "goal": "Shelf_A",
                "priority": 4,
                "parameters": {
                    "shelves": ["Shelf_A", "Shelf_B"],
                    "scan_depth": True,
                    "record_findings": True,
                },
            },
            {
                "robot": "Robot_4",
                "task": "inspect",
                "goal": "Shelf_C",
                "priority": 4,
                "parameters": {
                    "shelves": ["Shelf_C", "Shelf_D"],
                    "scan_depth": True,
                    "record_findings": True,
                },
            },
        ]
        return mission

    def _gen_full_inspection(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Full Warehouse Inspection", "SHELF_INSPECTION", priority=7, loop=False
        )
        mission["commands"] = [
            self._inspect_cmd("Robot_1", "Shelf_A", priority=7, speed=0.2),
            self._inspect_cmd("Robot_2", "Shelf_B", priority=7, speed=0.2),
            self._inspect_cmd("Robot_3", "Shelf_C", priority=7, speed=0.2),
            self._inspect_cmd("Robot_4", "Shelf_D", priority=7, speed=0.2),
        ]
        return mission

    def _gen_shelf_inspection(self, prompt: str, shelves: List[str]) -> Dict[str, Any]:
        shelf_str = ", ".join(shelves)
        mission = self._base_mission(
            f"Shelf Inspection: {shelf_str}", "SHELF_INSPECTION", priority=7, loop=False
        )
        robots_needed = min(len(shelves), len(ROBOT_IDS))
        for i in range(robots_needed):
            mission["commands"].append(
                self._inspect_cmd(ROBOT_IDS[i], shelves[i], priority=7, speed=0.2)
            )
        return mission

    def _inspect_cmd(
        self, robot: str, shelf: str, priority: int = 6, speed: float = 0.2
    ) -> Dict[str, Any]:
        return {
            "robot": robot,
            "task": "inspect",
            "goal": shelf,
            "priority": priority,
            "parameters": {
                "shelves": [shelf],
                "scan_depth": True,
                "scan_rfid": True,
                "record_findings": True,
                "rotation_scan": True,
                "speed": speed,
            },
        }

    def _gen_inventory_scan(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission("Inventory Scan", "INVENTORY_SCAN", priority=8, loop=False)
        for i, shelf in enumerate(SHELF_LOCATIONS):
            mission["commands"].append({
                "robot": ROBOT_IDS[i],
                "task": "inventory_scan",
                "goal": shelf,
                "priority": 8,
                "parameters": {
                    "shelves": [shelf],
                    "scan_type": "full",
                    "record_inventory": True,
                    "barcode_scan": True,
                    "rfid_scan": True,
                    "speed": 0.15,
                },
            })
        return mission

    def _gen_package_verification(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Package Verification", "PACKAGE_VERIFICATION", priority=8, loop=False
        )
        for i, shelf in enumerate(SHELF_LOCATIONS):
            mission["commands"].append({
                "robot": ROBOT_IDS[i],
                "task": "package_verify",
                "goal": shelf,
                "priority": 8,
                "parameters": {
                    "zone": shelf,
                    "verify_barcodes": True,
                    "verify_weight": False,
                    "verify_dimensions": False,
                    "record_discrepancies": True,
                    "speed": 0.2,
                },
            })
        return mission

    def _gen_package_pickup(self, prompt: str) -> Dict[str, Any]:
        pickup_zone = self._extract_zone(prompt) or "Shelf_A"
        mission = self._base_mission(
            "Package Pickup", "PACKAGE_PICKUP", priority=9, loop=False
        )
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "navigate",
                "goal": pickup_zone,
                "priority": 9,
                "parameters": {
                    "action": "pickup",
                    "zone": pickup_zone,
                    "speed": 0.3,
                },
            },
            {
                "robot": "Robot_2",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 4,
                "parameters": {
                    "waypoints": ["Waypoint_1", "Waypoint_2"],
                    "loop": False,
                    "role": "escort_support",
                },
            },
        ]
        return mission

    def _gen_package_delivery(self, prompt: str) -> Dict[str, Any]:
        delivery_zone = self._extract_zone(prompt) or "Waypoint_1"
        mission = self._base_mission(
            "Package Delivery", "PACKAGE_DELIVERY", priority=9, loop=False
        )
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "navigate",
                "goal": delivery_zone,
                "priority": 9,
                "parameters": {
                    "action": "deliver",
                    "destination": delivery_zone,
                    "speed": 0.25,
                },
            },
            {
                "robot": "Robot_2",
                "task": "formation",
                "goal": delivery_zone,
                "priority": 7,
                "parameters": {
                    "formation_type": "escort",
                    "leader": "Robot_1",
                    "spacing": 1.2,
                },
            },
        ]
        return mission

    def _gen_tracking_mission(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Target Tracking", "TARGET_TRACKING", priority=9, loop=False
        )
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "track_target",
                "goal": "target_zone",
                "priority": 9,
                "parameters": {
                    "target_class": "moving_package",
                    "safe_follow_distance": 1.5,
                    "max_follow_speed": 0.4,
                    "resume_task_after_lost": True,
                    "lost_timeout_seconds": 10.0,
                },
            },
            {
                "robot": "Robot_2",
                "task": "track_target",
                "goal": "target_zone",
                "priority": 9,
                "parameters": {
                    "target_class": "moving_package",
                    "safe_follow_distance": 2.0,
                    "max_follow_speed": 0.4,
                    "resume_task_after_lost": True,
                    "lost_timeout_seconds": 10.0,
                    "role": "backup_tracker",
                },
            },
            {
                "robot": "Robot_3",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 4,
                "parameters": {
                    "waypoints": ["Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4"],
                    "loop": True,
                    "role": "area_coverage",
                },
            },
            {
                "robot": "Robot_4",
                "task": "inspect",
                "goal": "Shelf_A",
                "priority": 4,
                "parameters": {
                    "shelves": ["Shelf_A", "Shelf_B", "Shelf_C", "Shelf_D"],
                    "scan_depth": True,
                },
            },
        ]
        return mission

    def _gen_formation_mission(self, prompt: str, formation_type: str) -> Dict[str, Any]:
        mission = self._base_mission(
            f"Formation: {formation_type.capitalize()}", "FORMATION", priority=6, loop=False
        )
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "formation",
                "goal": "Waypoint_1",
                "priority": 6,
                "parameters": {
                    "formation_type": formation_type,
                    "role": "leader",
                    "spacing": 1.5,
                    "waypoints": ["Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4",
                                  "Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8"],
                    "speed": 0.25,
                },
            },
            {
                "robot": "Robot_2",
                "task": "formation",
                "goal": "Waypoint_1",
                "priority": 6,
                "parameters": {
                    "formation_type": formation_type,
                    "role": "follower",
                    "leader": "Robot_1",
                    "slot_index": 0,
                    "spacing": 1.5,
                },
            },
            {
                "robot": "Robot_3",
                "task": "formation",
                "goal": "Waypoint_1",
                "priority": 6,
                "parameters": {
                    "formation_type": formation_type,
                    "role": "follower",
                    "leader": "Robot_1",
                    "slot_index": 1,
                    "spacing": 1.5,
                },
            },
            {
                "robot": "Robot_4",
                "task": "formation",
                "goal": "Waypoint_1",
                "priority": 6,
                "parameters": {
                    "formation_type": formation_type,
                    "role": "follower",
                    "leader": "Robot_1",
                    "slot_index": 2,
                    "spacing": 1.5,
                },
            },
        ]
        return mission

    def _gen_return_home(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Return All Robots Home", "RETURN_HOME", priority=10, loop=False
        )
        for i, robot_id in enumerate(ROBOT_IDS):
            mission["commands"].append({
                "robot": robot_id,
                "task": "navigate",
                "goal": HOME_POSITIONS[i],
                "priority": 10,
                "parameters": {
                    "action": "return_home",
                    "home_position": HOME_POSITIONS[i],
                    "speed": 0.35,
                },
            })
        return mission

    def _gen_charging_mission(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Fleet Charging Mission", "CHARGING_MISSION", priority=10, loop=False
        )
        stations = ["Charging_Station_1", "Charging_Station_1",
                    "Charging_Station_2", "Charging_Station_2"]
        for i, robot_id in enumerate(ROBOT_IDS):
            mission["commands"].append({
                "robot": robot_id,
                "task": "navigate_to_charger",
                "goal": stations[i],
                "priority": 10,
                "parameters": {
                    "station": stations[i],
                    "charge_to_percentage": 100.0,
                    "speed": 0.35,
                },
            })
        return mission

    def _gen_emergency_inspection(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Emergency Warehouse Inspection", "EMERGENCY_INSPECTION", priority=10, loop=False
        )
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 10,
                "parameters": {
                    "waypoints": PATROL_WAYPOINTS,
                    "loop": False,
                    "speed": 0.45,
                    "mode": "emergency",
                },
            },
            {
                "robot": "Robot_2",
                "task": "patrol",
                "goal": "Waypoint_5",
                "priority": 10,
                "parameters": {
                    "waypoints": list(reversed(PATROL_WAYPOINTS)),
                    "loop": False,
                    "speed": 0.45,
                    "mode": "emergency",
                },
            },
            {
                "robot": "Robot_3",
                "task": "inspect",
                "goal": "Shelf_A",
                "priority": 10,
                "parameters": {
                    "shelves": ["Shelf_A", "Shelf_B"],
                    "scan_depth": True,
                    "mode": "emergency",
                    "speed": 0.3,
                },
            },
            {
                "robot": "Robot_4",
                "task": "inspect",
                "goal": "Shelf_C",
                "priority": 10,
                "parameters": {
                    "shelves": ["Shelf_C", "Shelf_D"],
                    "scan_depth": True,
                    "mode": "emergency",
                    "speed": 0.3,
                },
            },
        ]
        return mission

    def _gen_continuous_monitoring(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Continuous Warehouse Monitoring", "CONTINUOUS_MONITORING", priority=6, loop=True
        )
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 6,
                "parameters": {
                    "waypoints": ["Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4"],
                    "loop": True,
                    "speed": 0.25,
                    "scan_while_moving": True,
                },
            },
            {
                "robot": "Robot_2",
                "task": "patrol",
                "goal": "Waypoint_5",
                "priority": 6,
                "parameters": {
                    "waypoints": ["Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8"],
                    "loop": True,
                    "speed": 0.25,
                    "scan_while_moving": True,
                },
            },
            {
                "robot": "Robot_3",
                "task": "inspect",
                "goal": "Shelf_A",
                "priority": 5,
                "parameters": {
                    "shelves": ["Shelf_A", "Shelf_B"],
                    "scan_depth": True,
                    "continuous": True,
                    "speed": 0.2,
                },
            },
            {
                "robot": "Robot_4",
                "task": "inspect",
                "goal": "Shelf_C",
                "priority": 5,
                "parameters": {
                    "shelves": ["Shelf_C", "Shelf_D"],
                    "scan_depth": True,
                    "continuous": True,
                    "speed": 0.2,
                },
            },
        ]
        return mission

    def _gen_comprehensive_inspection(self, prompt: str) -> Dict[str, Any]:
        mission = self._base_mission(
            "Comprehensive Warehouse Inspection", "WAREHOUSE_PATROL", priority=6, loop=True
        )
        mission["description"] = f'Generated from: "{prompt}"'
        mission["commands"] = [
            {
                "robot": "Robot_1",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 6,
                "parameters": {
                    "waypoints": ["Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4"],
                    "loop": True,
                    "speed": self._default_speed,
                },
            },
            {
                "robot": "Robot_2",
                "task": "inventory_scan",
                "goal": "Shelf_A",
                "priority": 6,
                "parameters": {
                    "shelves": ["Shelf_A", "Shelf_B"],
                    "scan_type": "full",
                    "record_inventory": True,
                },
            },
            {
                "robot": "Robot_3",
                "task": "inspect",
                "goal": "Shelf_C",
                "priority": 5,
                "parameters": {
                    "shelves": ["Shelf_C", "Shelf_D"],
                    "scan_depth": True,
                    "record_findings": True,
                },
            },
            {
                "robot": "Robot_4",
                "task": "patrol",
                "goal": "Waypoint_5",
                "priority": 4,
                "parameters": {
                    "waypoints": ["Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8"],
                    "loop": True,
                    "speed": self._default_speed,
                },
            },
        ]
        return mission

    def _extract_zone(self, prompt: str) -> Optional[str]:
        p = prompt.lower()
        for shelf in SHELF_LOCATIONS:
            if shelf.lower() in p:
                return shelf
        for wp in PATROL_WAYPOINTS:
            if wp.lower().replace("_", " ") in p:
                return wp
        return None

    def _publish_and_save(self, mission: Dict[str, Any], prompt: str) -> None:
        self._mission_counter += 1
        mission_json_str = json.dumps(mission, indent=2)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            self._output_dir, f"mission_{self._mission_counter:04d}_{ts}.json"
        )
        with open(filename, "w") as f:
            f.write(mission_json_str)

        out_msg = String()
        out_msg.data = mission_json_str
        self._mission_pub.publish(out_msg)

        status = MissionStatus()
        status.mission_id = mission["mission_id"]
        status.mission_name = mission["mission_name"]
        status.mission_type = mission["mission_type"]
        status.state = "PLANNED"
        status.total_commands = len(mission["commands"])
        self._status_pub.publish(status)

        self.get_logger().info(
            f'Mission planned: "{mission["mission_name"]}" '
            f'({len(mission["commands"])} commands) -> {filename}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
