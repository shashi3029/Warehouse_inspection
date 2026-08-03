#!/usr/bin/env python3
"""
Battery Manager Node

Simulates and monitors battery state for all warehouse robots.
Publishes BatteryStatus at 1 Hz per robot. Drain rates depend on
the robot's current activity; charging restores at charge_rate.

When battery falls below the configured threshold the node publishes
a low-battery event so the Mission Coordinator can trigger charging.
"""

import math
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from warehouse_msgs.msg import BatteryStatus, RobotState, TaskStatus


ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]

DRAIN_RATES: Dict[str, float] = {
    "IDLE":                  0.04,
    "WAITING":               0.04,
    "MOVING":                0.55,
    "PATROLLING":            0.48,
    "NAVIGATING":            0.55,
    "INSPECTING":            0.35,
    "FORMATION":             0.45,
    "TRACKING":              0.50,
    "RETURNING_TO_CHARGER":  0.50,
    "RESUMING":              0.45,
    "EMERGENCY_STOP":        0.02,
    "ERROR":                 0.02,
    "CHARGING":              -2.5,
}

NOMINAL_VOLTAGE = 24.0
CRITICAL_THRESHOLD = 10.0


class RobotBattery:
    def __init__(self, robot_id: str, initial_pct: float = 100.0):
        self.robot_id = robot_id
        self.percentage = initial_pct
        self.voltage = NOMINAL_VOLTAGE
        self.current_draw = 0.0
        self.temperature = 25.0
        self.is_charging = False
        self.is_healthy = True
        self.charging_station: str = ""
        self.state = "IDLE"
        self.charge_start_pct = 100.0
        self.last_update = time.monotonic()

    def update(self, dt: float) -> None:
        drain = DRAIN_RATES.get(self.state, 0.05)
        self.percentage -= drain * dt
        self.percentage = max(0.0, min(100.0, self.percentage))

        self.is_charging = (drain < 0.0)
        self.current_draw = abs(drain) * 0.5
        self.voltage = NOMINAL_VOLTAGE * (0.7 + 0.3 * self.percentage / 100.0)
        self.temperature = 25.0 + abs(drain) * 2.0
        self.is_healthy = self.percentage > 5.0

    def start_charge(self, station_id: str) -> None:
        self.state = "CHARGING"
        self.is_charging = True
        self.charging_station = station_id
        self.charge_start_pct = self.percentage

    def stop_charge(self) -> None:
        self.is_charging = False
        self.charging_station = ""

    def estimated_remaining_minutes(self, threshold: float = 20.0) -> float:
        drain_pct_per_sec = DRAIN_RATES.get(self.state, 0.05)
        if drain_pct_per_sec <= 0.0:
            return float("inf")
        remaining_pct = max(0.0, self.percentage - threshold)
        return (remaining_pct / drain_pct_per_sec) / 60.0

    def time_to_full_minutes(self) -> float:
        if not self.is_charging:
            return float("inf")
        charge_rate = abs(DRAIN_RATES.get("CHARGING", 2.5))
        remaining = 100.0 - self.percentage
        return (remaining / charge_rate) / 60.0


class BatteryManagerNode(Node):

    def __init__(self):
        super().__init__("battery_manager_node")

        self.declare_parameter("battery_threshold", 20.0)
        self.declare_parameter("update_rate_hz", 1.0)
        self.declare_parameter("robot_ids", ROBOT_IDS)
        self.declare_parameter("initial_battery_pct", 100.0)

        threshold = self.get_parameter("battery_threshold").value
        update_rate = self.get_parameter("update_rate_hz").value
        robot_ids = self.get_parameter("robot_ids").value
        initial_pct = self.get_parameter("initial_battery_pct").value

        self._threshold = float(threshold)

        import random
        self._batteries: Dict[str, RobotBattery] = {
            rid: RobotBattery(rid, float(initial_pct) - random.uniform(0, 15.0))
            for rid in robot_ids
        }

        cb_group = MutuallyExclusiveCallbackGroup()

        self._battery_pubs: Dict[str, Any] = {}
        for rid in robot_ids:
            self._battery_pubs[rid] = self.create_publisher(
                BatteryStatus, f"/warehouse/{rid}/battery_status", 10
            )

        self._low_battery_pub = self.create_publisher(
            String, "/warehouse/fleet/low_battery_alert", 10
        )

        for rid in robot_ids:
            self.create_subscription(
                RobotState,
                f"/warehouse/{rid}/robot_state",
                lambda msg, robot_id=rid: self._robot_state_callback(msg, robot_id),
                10,
            )
            self.create_subscription(
                TaskStatus,
                f"/warehouse/{rid}/task_status",
                lambda msg, robot_id=rid: self._task_status_callback(msg, robot_id),
                10,
            )

        self._update_timer = self.create_timer(
            1.0 / update_rate,
            self._battery_update_loop,
            callback_group=cb_group,
        )

        self._last_update_time = time.monotonic()
        self.get_logger().info(
            f"Battery Manager started for {len(self._batteries)} robots "
            f"(threshold: {self._threshold}%)"
        )

    def _robot_state_callback(self, msg: RobotState, robot_id: str) -> None:
        battery = self._batteries.get(robot_id)
        if battery:
            battery.state = msg.state

    def _task_status_callback(self, msg: TaskStatus, robot_id: str) -> None:
        battery = self._batteries.get(robot_id)
        if not battery:
            return
        if msg.task_type == "navigate_to_charger" and msg.completed:
            battery.start_charge(f"Charging_Station_1")
        elif msg.task_type not in ("navigate_to_charger",) and battery.is_charging:
            battery.stop_charge()

    def _battery_update_loop(self) -> None:
        now = time.monotonic()
        dt = now - self._last_update_time
        self._last_update_time = now

        for rid, battery in self._batteries.items():
            battery.update(dt)
            self._publish_battery_status(rid, battery)
            self._check_low_battery(rid, battery)

    def _publish_battery_status(self, robot_id: str, battery: RobotBattery) -> None:
        msg = BatteryStatus()
        msg.robot_id = robot_id
        msg.percentage = battery.percentage
        msg.voltage = battery.voltage
        msg.current_draw = battery.current_draw
        msg.temperature = battery.temperature
        msg.is_charging = battery.is_charging
        msg.is_healthy = battery.is_healthy
        msg.is_critical = battery.percentage < CRITICAL_THRESHOLD
        msg.estimated_remaining_minutes = battery.estimated_remaining_minutes(self._threshold)
        msg.charge_rate = abs(DRAIN_RATES.get("CHARGING", 2.5)) if battery.is_charging else 0.0
        msg.drain_rate = DRAIN_RATES.get(battery.state, 0.05) if not battery.is_charging else 0.0
        msg.charging_station_id = battery.charging_station
        msg.timestamp = self.get_clock().now().to_msg()
        self._battery_pubs[robot_id].publish(msg)

    def _check_low_battery(self, robot_id: str, battery: RobotBattery) -> None:
        if battery.percentage <= self._threshold and not battery.is_charging:
            import json
            alert = {
                "robot_id": robot_id,
                "battery_percentage": battery.percentage,
                "threshold": self._threshold,
                "state": battery.state,
                "estimated_remaining_minutes": battery.estimated_remaining_minutes(self._threshold),
            }
            msg = String()
            msg.data = json.dumps(alert)
            self._low_battery_pub.publish(msg)
            self.get_logger().warning(
                f"LOW BATTERY: {robot_id} at {battery.percentage:.1f}% "
                f"(state: {battery.state})"
            )

        if battery.percentage >= 99.5 and battery.is_charging:
            battery.stop_charge()
            battery.state = "IDLE"
            self.get_logger().info(f"{robot_id} fully charged (100%)")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
