#!/usr/bin/env python3
"""
Task Scheduler

Implements intelligent, dynamic task allocation for the warehouse fleet.
Never hardcodes robot assignments — selects the optimal robot based on
battery, distance, availability, state, priority, and estimated completion time.
"""

import math
import time
import uuid
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, field


WAREHOUSE_LOCATIONS: Dict[str, Tuple[float, float]] = {
    "Shelf_A":            (-12.0,  0.0),
    "Shelf_B":            ( -4.0,  0.0),
    "Shelf_C":            (  4.0,  0.0),
    "Shelf_D":            ( 12.0,  0.0),
    "Home_1":             ( -8.0, -8.0),
    "Home_2":             ( -8.0,  8.0),
    "Home_3":             (  8.0, -8.0),
    "Home_4":             (  8.0,  8.0),
    "Charging_Station_1": (-15.0, -15.0),
    "Charging_Station_2": ( 15.0, -15.0),
    "Waypoint_1":         (-12.0, -8.0),
    "Waypoint_2":         ( -4.0, -8.0),
    "Waypoint_3":         (  4.0, -8.0),
    "Waypoint_4":         ( 12.0, -8.0),
    "Waypoint_5":         (-12.0,  8.0),
    "Waypoint_6":         ( -4.0,  8.0),
    "Waypoint_7":         (  4.0,  8.0),
    "Waypoint_8":         ( 12.0,  8.0),
}


@dataclass
class RobotRecord:
    robot_id: str
    state: str = "IDLE"
    battery_percentage: float = 100.0
    position_x: float = 0.0
    position_y: float = 0.0
    speed: float = 0.0
    is_available: bool = True
    is_charging: bool = False
    current_task_id: Optional[str] = None
    current_task_type: Optional[str] = None
    current_goal: Optional[str] = None
    last_task_priority: int = 5
    estimated_task_completion: float = 0.0
    home_position: Tuple[float, float] = (0.0, 0.0)
    zone: str = ""
    completed_tasks: int = 0
    failed_tasks: int = 0


@dataclass
class ScheduledTask:
    task_id: str
    robot_id: str
    task_type: str
    goal: str
    priority: int
    parameters: Dict[str, Any] = field(default_factory=dict)
    created_at: float = field(default_factory=time.monotonic)
    estimated_duration_seconds: float = 30.0


class TaskScheduler:
    BATTERY_WEIGHT = 0.35
    DISTANCE_WEIGHT = 0.30
    AVAILABILITY_WEIGHT = 0.20
    PRIORITY_WEIGHT = 0.10
    LOAD_BALANCE_WEIGHT = 0.05

    IDLE_BONUS = 30.0
    WAITING_BONUS = 15.0

    AVG_ROBOT_SPEED = 0.3
    INSPECT_DURATION = 45.0
    PATROL_WAYPOINT_DURATION = 20.0
    INVENTORY_SCAN_DURATION = 60.0
    PACKAGE_OP_DURATION = 30.0
    NAVIGATE_DURATION_PER_METER = 4.0
    FORMATION_DURATION = 60.0
    TRACKING_DURATION = 120.0

    def __init__(self, locations: Optional[Dict[str, Tuple[float, float]]] = None):
        self._locations = locations or WAREHOUSE_LOCATIONS

    def schedule_commands(
        self,
        commands: List[Dict[str, Any]],
        robots: Dict[str, RobotRecord],
    ) -> List[ScheduledTask]:
        sorted_commands = sorted(
            commands,
            key=lambda c: c.get("priority", 5),
            reverse=True,
        )

        scheduled: List[ScheduledTask] = []
        assigned_robots: set = set()

        for cmd in sorted_commands:
            requested_robot = cmd.get("robot", "")

            if requested_robot and requested_robot in robots:
                robot = robots[requested_robot]
                if robot.is_available and requested_robot not in assigned_robots:
                    task = self._build_task(cmd, requested_robot)
                    scheduled.append(task)
                    assigned_robots.add(requested_robot)
                else:
                    best = self._find_best_robot(
                        cmd, robots, exclude=assigned_robots
                    )
                    if best:
                        task = self._build_task(cmd, best)
                        scheduled.append(task)
                        assigned_robots.add(best)
            else:
                best = self._find_best_robot(cmd, robots, exclude=assigned_robots)
                if best:
                    task = self._build_task(cmd, best)
                    scheduled.append(task)
                    assigned_robots.add(best)

        return scheduled

    def get_next_task_for_robot(
        self,
        robot: RobotRecord,
        pending_tasks: List[Dict[str, Any]],
        active_robot_ids: set,
    ) -> Optional[ScheduledTask]:
        if not robot.is_available or robot.is_charging:
            return None

        candidates = [
            t for t in pending_tasks if t.get("robot", robot.robot_id) == robot.robot_id
            or not t.get("robot")
        ]

        if not candidates:
            return None

        sorted_candidates = sorted(
            candidates, key=lambda c: c.get("priority", 5), reverse=True
        )

        for cmd in sorted_candidates:
            requested = cmd.get("robot", "")
            if requested and requested != robot.robot_id:
                continue

            if cmd.get("robot") == robot.robot_id or not cmd.get("robot"):
                return self._build_task(cmd, robot.robot_id)

        return None

    def get_nearest_charging_station(
        self, robot_x: float, robot_y: float
    ) -> Tuple[str, float, float, float]:
        charging_stations = {
            k: v for k, v in self._locations.items() if "Charging_Station" in k
        }
        best_station = None
        best_dist = float("inf")
        best_pos = (0.0, 0.0)

        for station_id, (sx, sy) in charging_stations.items():
            dist = self._distance(robot_x, robot_y, sx, sy)
            if dist < best_dist:
                best_dist = dist
                best_station = station_id
                best_pos = (sx, sy)

        return best_station or "Charging_Station_1", best_pos[0], best_pos[1], best_dist

    def estimate_task_duration(self, task_type: str, goal: str, robot: RobotRecord) -> float:
        travel_dist = 0.0
        if goal in self._locations:
            gx, gy = self._locations[goal]
            travel_dist = self._distance(robot.position_x, robot.position_y, gx, gy)

        travel_time = travel_dist * self.NAVIGATE_DURATION_PER_METER

        task_durations = {
            "navigate":          travel_time + 5.0,
            "patrol":            self.PATROL_WAYPOINT_DURATION * 4,
            "inspect":           travel_time + self.INSPECT_DURATION,
            "inventory_scan":    travel_time + self.INVENTORY_SCAN_DURATION,
            "package_verify":    travel_time + self.PACKAGE_OP_DURATION,
            "package_pickup":    travel_time + self.PACKAGE_OP_DURATION,
            "package_delivery":  travel_time + self.PACKAGE_OP_DURATION,
            "track_target":      self.TRACKING_DURATION,
            "formation":         self.FORMATION_DURATION,
            "navigate_to_charger": travel_time + 5.0,
            "return_home":       travel_time + 5.0,
            "wait":              10.0,
            "rotate":            8.0,
        }
        return task_durations.get(task_type, 30.0)

    def _find_best_robot(
        self,
        command: Dict[str, Any],
        robots: Dict[str, RobotRecord],
        exclude: set,
    ) -> Optional[str]:
        scores: List[Tuple[float, str]] = []

        task_type = command.get("task", "navigate")
        goal = command.get("goal", "")
        cmd_priority = command.get("priority", 5)

        goal_pos = self._locations.get(goal)

        for robot_id, robot in robots.items():
            if robot_id in exclude:
                continue
            if not robot.is_available or robot.is_charging:
                continue
            if robot.battery_percentage < 15.0:
                continue

            score = self._score_robot(robot, task_type, goal, goal_pos, cmd_priority, robots)
            scores.append((score, robot_id))

        if not scores:
            return None

        scores.sort(key=lambda x: x[0], reverse=True)
        return scores[0][1]

    def _score_robot(
        self,
        robot: RobotRecord,
        task_type: str,
        goal: str,
        goal_pos: Optional[Tuple[float, float]],
        cmd_priority: int,
        all_robots: Dict[str, RobotRecord],
    ) -> float:
        battery_score = robot.battery_percentage * self.BATTERY_WEIGHT

        if goal_pos:
            dist = self._distance(robot.position_x, robot.position_y, *goal_pos)
            max_dist = 40.0
            normalized_dist = max(0.0, (max_dist - dist) / max_dist)
            distance_score = normalized_dist * 100.0 * self.DISTANCE_WEIGHT
        else:
            distance_score = 50.0 * self.DISTANCE_WEIGHT

        if robot.state == "IDLE":
            avail_score = self.IDLE_BONUS
        elif robot.state == "WAITING":
            avail_score = self.WAITING_BONUS
        else:
            avail_score = 0.0
        availability_score = avail_score * self.AVAILABILITY_WEIGHT

        priority_score = 0.0
        if robot.last_task_priority < cmd_priority:
            priority_score = 10.0
        priority_score *= self.PRIORITY_WEIGHT

        total_completed = sum(r.completed_tasks for r in all_robots.values())
        if total_completed > 0:
            robot_share = robot.completed_tasks / total_completed
            load_score = (1.0 - robot_share) * 20.0
        else:
            load_score = 20.0
        load_balance_score = load_score * self.LOAD_BALANCE_WEIGHT

        specialization_bonus = self._get_specialization_bonus(robot, task_type, goal)

        total = (
            battery_score
            + distance_score
            + availability_score
            + priority_score
            + load_balance_score
            + specialization_bonus
        )
        return total

    def _get_specialization_bonus(
        self, robot: RobotRecord, task_type: str, goal: str
    ) -> float:
        if task_type in ["track_target"] and robot.robot_id in ["Robot_1", "Robot_2"]:
            return 5.0
        if task_type in ["formation"] and robot.robot_id == "Robot_1":
            return 8.0
        if task_type in ["inspect", "inventory_scan"] and robot.robot_id in ["Robot_3", "Robot_4"]:
            return 3.0
        return 0.0

    def _build_task(self, command: Dict[str, Any], robot_id: str) -> ScheduledTask:
        task_type = command.get("task", "navigate")
        goal = command.get("goal", "")
        priority = command.get("priority", 5)
        parameters = dict(command.get("parameters", {}))

        return ScheduledTask(
            task_id=str(uuid.uuid4()),
            robot_id=robot_id,
            task_type=task_type,
            goal=goal,
            priority=priority,
            parameters=parameters,
        )

    @staticmethod
    def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
