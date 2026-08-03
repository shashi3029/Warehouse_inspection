#!/usr/bin/env python3
"""
Mission Validator Node

Validates mission.json against schema, safety constraints, and warehouse
operational rules. Produces a validated_mission.json on success or
publishes detailed rejection errors.
"""

import json
import uuid
import math
from datetime import datetime, timezone
from typing import Dict, List, Any, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from warehouse_msgs.msg import MissionStatus


VALID_ROBOT_IDS: Set[str] = {"Robot_1", "Robot_2", "Robot_3", "Robot_4"}

VALID_MISSION_TYPES: Set[str] = {
    "WAREHOUSE_PATROL",
    "SHELF_INSPECTION",
    "INVENTORY_SCAN",
    "PACKAGE_VERIFICATION",
    "PACKAGE_PICKUP",
    "PACKAGE_DELIVERY",
    "TARGET_TRACKING",
    "TARGET_SEARCH",
    "RETURN_HOME",
    "CHARGING_MISSION",
    "CONTINUOUS_MONITORING",
    "EMERGENCY_INSPECTION",
    "FORMATION",
}

VALID_TASK_TYPES: Set[str] = {
    "navigate",
    "patrol",
    "inspect",
    "inventory_scan",
    "package_verify",
    "package_pickup",
    "package_delivery",
    "track_target",
    "formation",
    "navigate_to_charger",
    "return_home",
    "wait",
    "rotate",
}

VALID_GOALS: Set[str] = {
    "Shelf_A", "Shelf_B", "Shelf_C", "Shelf_D",
    "Home_1", "Home_2", "Home_3", "Home_4",
    "Charging_Station_1", "Charging_Station_2",
    "Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4",
    "Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8",
    "target_zone",
}

WAREHOUSE_BOUNDS = {"x_min": -20.0, "x_max": 20.0, "y_min": -20.0, "y_max": 20.0}

MAX_SPEED = 0.5
MIN_BATTERY_THRESHOLD = 10
MAX_BATTERY_THRESHOLD = 50
MAX_PRIORITY = 10
MIN_PRIORITY = 1


class ValidationError:
    def __init__(self, code: str, message: str, field: str = "", severity: str = "ERROR"):
        self.code = code
        self.message = message
        self.field = field
        self.severity = severity

    def to_dict(self) -> Dict[str, str]:
        return {
            "code": self.code,
            "message": self.message,
            "field": self.field,
            "severity": self.severity,
        }

    def __str__(self) -> str:
        return f"[{self.severity}] {self.code}: {self.message} (field: {self.field})"


class MissionValidatorNode(Node):

    def __init__(self):
        super().__init__("mission_validator_node")

        self.declare_parameter("strict_mode", True)
        self.declare_parameter("allow_unknown_goals", False)

        self._strict_mode = self.get_parameter("strict_mode").value
        self._allow_unknown_goals = self.get_parameter("allow_unknown_goals").value

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._validated_pub = self.create_publisher(
            String, "/warehouse/validated_mission", latching_qos
        )
        self._status_pub = self.create_publisher(
            MissionStatus, "/warehouse/mission_validator/status", 10
        )
        self._error_pub = self.create_publisher(
            String, "/warehouse/mission_validator/errors", 10
        )

        self._mission_sub = self.create_subscription(
            String, "/warehouse/mission_json", self._mission_callback, latching_qos
        )

        self.get_logger().info("Mission Validator ready — listening on /warehouse/mission_json")

    def _mission_callback(self, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._reject(None, [ValidationError("JSON_PARSE_ERROR", str(exc), "root")])
            return

        errors, warnings = self.validate(raw)
        critical = [e for e in errors if e.severity == "ERROR"]

        if critical:
            self._reject(raw, critical, warnings)
        else:
            validated = self._enrich(raw, warnings)
            self._accept(validated, warnings)

    def validate(self, mission: Dict[str, Any]) -> Tuple[List[ValidationError], List[ValidationError]]:
        errors: List[ValidationError] = []
        warnings: List[ValidationError] = []

        self._validate_required_fields(mission, errors)
        self._validate_mission_type(mission, errors)
        self._validate_priority(mission, errors, warnings)
        self._validate_battery_threshold(mission, errors, warnings)
        self._validate_commands(mission, errors, warnings)
        self._validate_robot_assignments(mission, errors, warnings)
        self._validate_goal_uniqueness(mission, warnings)
        self._validate_consistency(mission, errors, warnings)

        return errors, warnings

    def _validate_required_fields(
        self, mission: Dict, errors: List[ValidationError]
    ) -> None:
        required = ["mission_name", "mission_type", "commands"]
        for field in required:
            if field not in mission:
                errors.append(ValidationError(
                    "MISSING_REQUIRED_FIELD",
                    f'Required field "{field}" is missing',
                    field,
                ))
            elif not mission[field] and field != "commands":
                errors.append(ValidationError(
                    "EMPTY_REQUIRED_FIELD",
                    f'Required field "{field}" is empty',
                    field,
                ))

        if "commands" in mission and not isinstance(mission["commands"], list):
            errors.append(ValidationError(
                "INVALID_COMMANDS_TYPE",
                '"commands" must be a list',
                "commands",
            ))

        if "commands" in mission and isinstance(mission["commands"], list):
            if len(mission["commands"]) == 0:
                errors.append(ValidationError(
                    "EMPTY_COMMANDS",
                    "Mission has no commands",
                    "commands",
                ))

    def _validate_mission_type(self, mission: Dict, errors: List[ValidationError]) -> None:
        mt = mission.get("mission_type", "")
        if mt and mt not in VALID_MISSION_TYPES:
            errors.append(ValidationError(
                "INVALID_MISSION_TYPE",
                f'Unknown mission type "{mt}". Valid types: {sorted(VALID_MISSION_TYPES)}',
                "mission_type",
            ))

    def _validate_priority(
        self,
        mission: Dict,
        errors: List[ValidationError],
        warnings: List[ValidationError],
    ) -> None:
        p = mission.get("priority", 5)
        if not isinstance(p, int):
            errors.append(ValidationError(
                "INVALID_PRIORITY_TYPE", "Priority must be an integer", "priority"
            ))
        elif not (MIN_PRIORITY <= p <= MAX_PRIORITY):
            errors.append(ValidationError(
                "PRIORITY_OUT_OF_RANGE",
                f"Priority {p} out of range [{MIN_PRIORITY}, {MAX_PRIORITY}]",
                "priority",
            ))

    def _validate_battery_threshold(
        self,
        mission: Dict,
        errors: List[ValidationError],
        warnings: List[ValidationError],
    ) -> None:
        bt = mission.get("battery_threshold", 20)
        if not isinstance(bt, (int, float)):
            errors.append(ValidationError(
                "INVALID_BATTERY_THRESHOLD_TYPE",
                "battery_threshold must be numeric",
                "battery_threshold",
            ))
            return
        if bt < MIN_BATTERY_THRESHOLD:
            errors.append(ValidationError(
                "BATTERY_THRESHOLD_TOO_LOW",
                f"battery_threshold {bt}% is critically low (min: {MIN_BATTERY_THRESHOLD}%)",
                "battery_threshold",
            ))
        elif bt > MAX_BATTERY_THRESHOLD:
            warnings.append(ValidationError(
                "BATTERY_THRESHOLD_HIGH",
                f"battery_threshold {bt}% is high — robots will charge frequently",
                "battery_threshold",
                severity="WARNING",
            ))

    def _validate_commands(
        self,
        mission: Dict,
        errors: List[ValidationError],
        warnings: List[ValidationError],
    ) -> None:
        commands = mission.get("commands", [])
        if not isinstance(commands, list):
            return

        for idx, cmd in enumerate(commands):
            prefix = f"commands[{idx}]"

            if not isinstance(cmd, dict):
                errors.append(ValidationError(
                    "INVALID_COMMAND_FORMAT",
                    f"Command at index {idx} is not an object",
                    prefix,
                ))
                continue

            if "robot" not in cmd:
                errors.append(ValidationError(
                    "MISSING_ROBOT_ID",
                    f"Command {idx} missing 'robot' field",
                    f"{prefix}.robot",
                ))
            elif cmd["robot"] not in VALID_ROBOT_IDS:
                errors.append(ValidationError(
                    "INVALID_ROBOT_ID",
                    f'Unknown robot "{cmd["robot"]}" in command {idx}. '
                    f"Valid robots: {sorted(VALID_ROBOT_IDS)}",
                    f"{prefix}.robot",
                ))

            if "task" not in cmd:
                errors.append(ValidationError(
                    "MISSING_TASK_TYPE",
                    f"Command {idx} missing 'task' field",
                    f"{prefix}.task",
                ))
            elif cmd["task"] not in VALID_TASK_TYPES:
                errors.append(ValidationError(
                    "INVALID_TASK_TYPE",
                    f'Unknown task type "{cmd["task"]}" in command {idx}',
                    f"{prefix}.task",
                ))

            goal = cmd.get("goal", "")
            if goal and goal not in VALID_GOALS and not self._allow_unknown_goals:
                warnings.append(ValidationError(
                    "UNKNOWN_GOAL",
                    f'Unknown goal "{goal}" in command {idx} — verify warehouse location map',
                    f"{prefix}.goal",
                    severity="WARNING",
                ))

            cmd_priority = cmd.get("priority", 5)
            if not isinstance(cmd_priority, int) or not (
                MIN_PRIORITY <= cmd_priority <= MAX_PRIORITY
            ):
                warnings.append(ValidationError(
                    "COMMAND_PRIORITY_ISSUE",
                    f"Command {idx} has invalid priority {cmd_priority}, will use default",
                    f"{prefix}.priority",
                    severity="WARNING",
                ))

            params = cmd.get("parameters", {})
            if isinstance(params, dict):
                speed = params.get("speed", None)
                if speed is not None and speed > MAX_SPEED:
                    errors.append(ValidationError(
                        "UNSAFE_SPEED",
                        f"Command {idx} speed {speed} m/s exceeds maximum {MAX_SPEED} m/s",
                        f"{prefix}.parameters.speed",
                    ))

    def _validate_robot_assignments(
        self,
        mission: Dict,
        errors: List[ValidationError],
        warnings: List[ValidationError],
    ) -> None:
        commands = mission.get("commands", [])
        if not isinstance(commands, list):
            return

        robot_command_count: Dict[str, int] = {}
        for cmd in commands:
            if not isinstance(cmd, dict):
                continue
            robot = cmd.get("robot", "")
            if robot in VALID_ROBOT_IDS:
                robot_command_count[robot] = robot_command_count.get(robot, 0) + 1

        for robot, count in robot_command_count.items():
            if count > 3:
                warnings.append(ValidationError(
                    "ROBOT_OVERLOADED",
                    f"Robot {robot} assigned {count} commands simultaneously — may cause scheduling conflicts",
                    "commands",
                    severity="WARNING",
                ))

    def _validate_goal_uniqueness(
        self,
        mission: Dict,
        warnings: List[ValidationError],
    ) -> None:
        commands = mission.get("commands", [])
        if not isinstance(commands, list):
            return

        goal_assignments: Dict[str, List[str]] = {}
        for cmd in commands:
            if not isinstance(cmd, dict):
                continue
            goal = cmd.get("goal", "")
            robot = cmd.get("robot", "")
            if goal and robot and goal not in ["target_zone"]:
                if goal not in goal_assignments:
                    goal_assignments[goal] = []
                goal_assignments[goal].append(robot)

        for goal, robots in goal_assignments.items():
            if len(robots) > 1:
                warnings.append(ValidationError(
                    "DUPLICATE_GOAL_ASSIGNMENT",
                    f'Goal "{goal}" assigned to multiple robots: {robots} — possible collision',
                    "commands",
                    severity="WARNING",
                ))

    def _validate_consistency(
        self,
        mission: Dict,
        errors: List[ValidationError],
        warnings: List[ValidationError],
    ) -> None:
        mission_type = mission.get("mission_type", "")
        commands = mission.get("commands", [])

        if mission_type == "RETURN_HOME":
            for cmd in commands:
                if isinstance(cmd, dict) and cmd.get("task") not in [
                    "navigate", "return_home"
                ]:
                    warnings.append(ValidationError(
                        "INCONSISTENT_TASK_FOR_TYPE",
                        f'Task "{cmd.get("task")}" is unexpected in a RETURN_HOME mission',
                        "commands",
                        severity="WARNING",
                    ))

        if mission_type == "CHARGING_MISSION":
            for cmd in commands:
                if isinstance(cmd, dict) and cmd.get("task") not in [
                    "navigate_to_charger", "navigate"
                ]:
                    warnings.append(ValidationError(
                        "INCONSISTENT_TASK_FOR_TYPE",
                        f'Task "{cmd.get("task")}" is unexpected in a CHARGING_MISSION',
                        "commands",
                        severity="WARNING",
                    ))

        if mission.get("loop") and not mission.get("resume_after_charge", True):
            warnings.append(ValidationError(
                "LOOP_WITHOUT_RESUME",
                "Looping mission has resume_after_charge=false — robots may not complete full loops",
                "resume_after_charge",
                severity="WARNING",
            ))

    def _enrich(
        self, mission: Dict[str, Any], warnings: List[ValidationError]
    ) -> Dict[str, Any]:
        validated = dict(mission)
        validated["validated"] = True
        validated["validation_timestamp"] = datetime.now(timezone.utc).isoformat()
        validated["validation_id"] = str(uuid.uuid4())
        validated["validation_warnings"] = [w.to_dict() for w in warnings]

        validated.setdefault("mission_id", str(uuid.uuid4()))
        validated.setdefault("priority", 5)
        validated.setdefault("loop", False)
        validated.setdefault("battery_threshold", 20)
        validated.setdefault("resume_after_charge", True)

        for cmd in validated.get("commands", []):
            if isinstance(cmd, dict):
                cmd.setdefault("priority", validated["priority"])
                params = cmd.setdefault("parameters", {})
                params.setdefault("speed", 0.3)

        return validated

    def _accept(
        self, validated: Dict[str, Any], warnings: List[ValidationError]
    ) -> None:
        validated_str = json.dumps(validated, indent=2)
        out_msg = String()
        out_msg.data = validated_str
        self._validated_pub.publish(out_msg)

        status = MissionStatus()
        status.mission_id = validated.get("mission_id", "")
        status.mission_name = validated.get("mission_name", "")
        status.mission_type = validated.get("mission_type", "")
        status.state = "VALIDATED"
        status.total_commands = len(validated.get("commands", []))
        self._status_pub.publish(status)

        warning_count = len(warnings)
        self.get_logger().info(
            f'Mission VALIDATED: "{validated["mission_name"]}" '
            f'({warning_count} warnings)'
        )
        for w in warnings:
            self.get_logger().warning(f"  {w}")

    def _reject(
        self,
        mission: Optional[Dict],
        errors: List[ValidationError],
        warnings: Optional[List[ValidationError]] = None,
    ) -> None:
        name = mission.get("mission_name", "Unknown") if mission else "Unknown"
        error_report = {
            "rejected": True,
            "mission_name": name,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "errors": [e.to_dict() for e in errors],
            "warnings": [w.to_dict() for w in (warnings or [])],
        }

        err_msg = String()
        err_msg.data = json.dumps(error_report, indent=2)
        self._error_pub.publish(err_msg)

        status = MissionStatus()
        status.mission_name = name
        status.state = "REJECTED"
        status.error_message = "; ".join(str(e) for e in errors[:3])
        self._status_pub.publish(status)

        self.get_logger().error(
            f'Mission REJECTED: "{name}" ({len(errors)} errors)'
        )
        for err in errors:
            self.get_logger().error(f"  {err}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
