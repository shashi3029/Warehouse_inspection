#!/usr/bin/env python3
"""
Fleet Dashboard Node

Aggregates all robot and mission data and publishes a human-readable
status summary to the terminal at a configurable rate. Also publishes
structured JSON status for external monitoring tools.

Dashboard output includes:
  - Robot name, state, task, goal, battery, position, charging status
  - Mission name, progress, loop count
  - Completed / failed / pending task counts
  - Active warnings
"""

import json
import time
from typing import Dict, Any, List, Optional
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String
from warehouse_msgs.msg import FleetStatus, RobotInfo, BatteryStatus, MissionStatus


ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]

STATE_COLORS = {
    "IDLE":                  "[  IDLE  ]",
    "MOVING":                "[ MOVING ]",
    "PATROLLING":            "[PATROL  ]",
    "NAVIGATING":            "[NAVIG.  ]",
    "INSPECTING":            "[INSPECT ]",
    "FORMATION":             "[FORMTN  ]",
    "TRACKING":              "[TRACK   ]",
    "WAITING":               "[WAITING ]",
    "RETURNING_TO_CHARGER":  "[->CHRGR ]",
    "CHARGING":              "[CHRGNG  ]",
    "RESUMING":              "[RESUMING]",
    "EMERGENCY_STOP":        "[EMRGNCY!]",
    "ERROR":                 "[ERROR!! ]",
}

BATTERY_BARS = {
    (90, 101): "████████████",
    (75, 90):  "█████████░░░",
    (50, 75):  "███████░░░░░",
    (25, 50):  "█████░░░░░░░",
    (10, 25):  "███░░░░░░░░░",
    (0,  10):  "█░░░░░░░░░░░",
}


def battery_bar(pct: float) -> str:
    for (lo, hi), bar in BATTERY_BARS.items():
        if lo <= pct < hi:
            return bar
    return "░░░░░░░░░░░░"


def battery_symbol(pct: float) -> str:
    if pct >= 80.0:
        return "[=====]"
    elif pct >= 60.0:
        return "[====.]"
    elif pct >= 40.0:
        return "[===..]"
    elif pct >= 20.0:
        return "[==...]"
    elif pct >= 10.0:
        return "[=....]"
    else:
        return "[!....]"


class FleetDashboardNode(Node):

    def __init__(self):
        super().__init__("fleet_dashboard_node")

        self.declare_parameter("display_rate_hz", 0.5)
        self.declare_parameter("publish_json_rate_hz", 1.0)
        self.declare_parameter("show_terminal_output", True)

        display_rate = self.get_parameter("display_rate_hz").value
        publish_rate = self.get_parameter("publish_json_rate_hz").value
        self._show_terminal = self.get_parameter("show_terminal_output").value

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._fleet_status: Optional[FleetStatus] = None
        self._mission_status: Optional[MissionStatus] = None
        self._battery_status: Dict[str, BatteryStatus] = {}
        self._station_status: Dict[str, Any] = {}
        self._start_time = time.monotonic()

        self.create_subscription(
            FleetStatus, "/warehouse/fleet_status",
            self._fleet_status_callback, 10
        )
        self.create_subscription(
            MissionStatus, "/warehouse/mission_status",
            self._mission_status_callback, 10
        )
        self.create_subscription(
            String, "/warehouse/charging/station_status",
            self._station_status_callback, 10
        )
        for rid in ROBOT_IDS:
            self.create_subscription(
                BatteryStatus, f"/warehouse/{rid}/battery_status",
                lambda msg, robot_id=rid: self._battery_callback(msg, robot_id), 10
            )

        self._json_pub = self.create_publisher(
            String, "/warehouse/dashboard/json_status", 10
        )
        self._summary_pub = self.create_publisher(
            String, "/warehouse/dashboard/summary", 10
        )

        self.create_timer(1.0 / display_rate, self._display_dashboard)
        self.create_timer(1.0 / publish_rate, self._publish_json_status)

        self.get_logger().info("Fleet Dashboard active — publishing on /warehouse/dashboard/*")

    def _fleet_status_callback(self, msg: FleetStatus) -> None:
        self._fleet_status = msg

    def _mission_status_callback(self, msg: MissionStatus) -> None:
        self._mission_status = msg

    def _battery_callback(self, msg: BatteryStatus, robot_id: str) -> None:
        self._battery_status[robot_id] = msg

    def _station_status_callback(self, msg: String) -> None:
        try:
            self._station_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _display_dashboard(self) -> None:
        if not self._show_terminal:
            return

        lines = self._build_dashboard_lines()
        output = "\n".join(lines)

        self.get_logger().info("\n" + output)

        summary_msg = String()
        summary_msg.data = output
        self._summary_pub.publish(summary_msg)

    def _build_dashboard_lines(self) -> List[str]:
        uptime = time.monotonic() - self._start_time
        h, rem = divmod(int(uptime), 3600)
        m, s = divmod(rem, 60)

        lines = [
            "",
            "╔══════════════════════════════════════════════════════════════════════╗",
            "║         WAREHOUSE FLEET MANAGEMENT SYSTEM — LIVE DASHBOARD          ║",
            f"║  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   Uptime: {h:02d}:{m:02d}:{s:02d}                    ║",
            "╠══════════════════════════════════════════════════════════════════════╣",
        ]

        if self._fleet_status:
            fs = self._fleet_status
            progress_pct = fs.mission_progress * 100.0
            bar_len = 30
            filled = int(bar_len * fs.mission_progress)
            bar = "█" * filled + "░" * (bar_len - filled)

            lines += [
                f"║  MISSION: {fs.mission_name[:38]:<38}      ║",
                f"║  Type: {fs.mission_type:<15}  Loop: {'YES' if fs.is_looping else 'NO ':3}  Loops: {fs.mission_loop_count:<5}  ║",
                f"║  Progress: [{bar}] {progress_pct:5.1f}%  ║",
                f"║  Tasks:  Done={fs.completed_tasks:<4} Failed={fs.failed_tasks:<4} Pending={fs.pending_tasks:<4}  Active={len(fs.active_robot_ids):<3}   ║",
                "╠══════════════════════════════════════════════════════════════════════╣",
                "║  ROBOT STATUS                                                        ║",
                "║  Robot     State        Task        Battery      Position    Charg   ║",
                "║  ───────── ──────────── ──────────  ───────────  ──────────  ─────   ║",
            ]

            for robot in fs.robots:
                rid = robot.robot_id
                state_str = STATE_COLORS.get(robot.state, f"[{robot.state[:8]:8}]")
                task_str = (robot.current_task or "—")[:10]
                bat_pct = robot.battery_percentage
                bat_sym = battery_symbol(bat_pct)
                charg_str = "CHG" if robot.is_charging else "   "
                pos_str = f"({robot.position.x:5.1f},{robot.position.y:5.1f})"

                battery_info = self._battery_status.get(rid)
                critical_mark = "!" if (battery_info and battery_info.is_critical) else " "

                lines.append(
                    f"║  {rid:<9} {state_str} {task_str:<10}  "
                    f"{bat_sym}{bat_pct:5.1f}%{critical_mark} "
                    f"{pos_str}  {charg_str}   ║"
                )
        else:
            lines.append("║  Waiting for fleet data...                                           ║")

        lines.append("╠══════════════════════════════════════════════════════════════════════╣")

        if self._station_status:
            lines.append("║  CHARGING STATIONS                                                   ║")
            for sid, station in self._station_status.items():
                occupants = station.get("occupants", [])
                queue = station.get("queue", [])
                cap = station.get("capacity", 2)
                avail = station.get("available_slots", cap)
                occ_str = ", ".join(occupants) if occupants else "—"
                q_str = f"Queue: {', '.join(queue)}" if queue else "Queue: —"
                lines.append(
                    f"║  {sid[:22]:<22}: [{avail}/{cap} free] Charging: {occ_str[:15]:<15}  ║"
                )
        else:
            lines.append("║  No charging station data yet...                                     ║")

        lines.append("╠══════════════════════════════════════════════════════════════════════╣")

        if self._fleet_status:
            fs = self._fleet_status
            lines += [
                f"║  Fleet: {fs.total_robots} robots  "
                f"Active={fs.active_robots}  "
                f"Charging={fs.charging_robots}  "
                f"Idle={fs.idle_robots}  "
                f"Errors={fs.error_robots}     ║",
            ]

        lines.append("╚══════════════════════════════════════════════════════════════════════╝")
        return lines

    def _publish_json_status(self) -> None:
        data: Dict[str, Any] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "uptime_seconds": time.monotonic() - self._start_time,
        }

        if self._fleet_status:
            fs = self._fleet_status
            data["mission"] = {
                "name": fs.mission_name,
                "type": fs.mission_type,
                "progress": fs.mission_progress,
                "is_looping": fs.is_looping,
                "loop_count": fs.mission_loop_count,
                "completed_tasks": fs.completed_tasks,
                "failed_tasks": fs.failed_tasks,
                "pending_tasks": fs.pending_tasks,
            }
            data["fleet"] = {
                "total_robots": fs.total_robots,
                "active_robots": fs.active_robots,
                "charging_robots": fs.charging_robots,
                "idle_robots": fs.idle_robots,
                "error_robots": fs.error_robots,
            }
            data["robots"] = []
            for r in fs.robots:
                bat = self._battery_status.get(r.robot_id)
                data["robots"].append({
                    "robot_id": r.robot_id,
                    "state": r.state,
                    "current_task": r.current_task,
                    "goal": r.goal,
                    "battery_percentage": r.battery_percentage,
                    "is_charging": r.is_charging,
                    "is_available": r.is_available,
                    "position": {"x": r.position.x, "y": r.position.y},
                    "speed": r.speed,
                    "zone": r.zone,
                    "completed_tasks": r.completed_tasks,
                    "failed_tasks": r.failed_tasks,
                    "is_critical_battery": bat.is_critical if bat else False,
                    "estimated_remaining_minutes": (
                        bat.estimated_remaining_minutes if bat else None
                    ),
                })

        if self._station_status:
            data["charging_stations"] = self._station_status

        msg = String()
        msg.data = json.dumps(data, indent=2)
        self._json_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
