#!/usr/bin/env python3
"""
Mission Coordinator Node — Fleet Manager

Continuously orchestrates all warehouse robots. Responsibilities:
  - Load and dispatch validated missions
  - Maintain the full robot fleet database
  - Run dynamic task scheduling
  - Monitor task execution and handle failures
  - Initiate battery charging and task resumption
  - Generate new missions when the current one completes
  - Never stop — the warehouse operates forever
"""

import json
import time
import uuid
import threading
import math
from typing import Dict, List, Any, Optional
from datetime import datetime, timezone
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

from warehouse_msgs.msg import (
    RobotState,
    BatteryStatus,
    TaskStatus,
    FleetStatus,
    MissionStatus,
    RobotInfo,
    ChargingStatus,
)
from warehouse_msgs.srv import AssignTask, EmergencyStop, GetNearestCharger

from .robot_state_machine import RobotStateMachine, RobotState as State
from .task_scheduler import TaskScheduler, RobotRecord, ScheduledTask, WAREHOUSE_LOCATIONS


ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]

ROBOT_HOME_POSITIONS = {
    "Robot_1": (-8.0, -8.0),
    "Robot_2": (-8.0,  8.0),
    "Robot_3": ( 8.0, -8.0),
    "Robot_4": ( 8.0,  8.0),
}

BATTERY_THRESHOLD = 20.0
COORDINATION_RATE_HZ = 10.0
TASK_TIMEOUT_SECONDS = 300.0
STALL_DETECTION_SECONDS = 60.0


class RobotFleetRecord:
    def __init__(self, robot_id: str, logger):
        self.robot_id = robot_id
        self.state_machine = RobotStateMachine(robot_id, logger)
        self.battery_percentage: float = 100.0
        self.is_charging: bool = False
        self.position_x: float = ROBOT_HOME_POSITIONS.get(robot_id, (0.0, 0.0))[0]
        self.position_y: float = ROBOT_HOME_POSITIONS.get(robot_id, (0.0, 0.0))[1]
        self.speed: float = 0.0
        self.current_task: Optional[ScheduledTask] = None
        self.saved_task: Optional[ScheduledTask] = None
        self.target_charger: Optional[str] = None
        self.task_start_time: float = 0.0
        self.last_task_progress: float = 0.0
        self.last_progress_update: float = time.monotonic()
        self.completed_tasks: int = 0
        self.failed_tasks: int = 0
        self.zone: str = ""

    @property
    def is_available(self) -> bool:
        return self.state_machine.is_available()

    def to_robot_record(self) -> RobotRecord:
        hx, hy = ROBOT_HOME_POSITIONS.get(self.robot_id, (0.0, 0.0))
        return RobotRecord(
            robot_id=self.robot_id,
            state=self.state_machine.state_name,
            battery_percentage=self.battery_percentage,
            position_x=self.position_x,
            position_y=self.position_y,
            speed=self.speed,
            is_available=self.is_available,
            is_charging=self.is_charging,
            current_task_id=self.current_task.task_id if self.current_task else None,
            current_task_type=self.current_task.task_type if self.current_task else None,
            current_goal=self.current_task.goal if self.current_task else None,
            home_position=(hx, hy),
            zone=self.zone,
            completed_tasks=self.completed_tasks,
            failed_tasks=self.failed_tasks,
        )


class MissionCoordinatorNode(Node):

    def __init__(self):
        super().__init__("mission_coordinator_node")

        self.declare_parameter("battery_threshold", BATTERY_THRESHOLD)
        self.declare_parameter("coordination_rate_hz", COORDINATION_RATE_HZ)
        self.declare_parameter("task_timeout_seconds", TASK_TIMEOUT_SECONDS)
        self.declare_parameter("auto_generate_missions", True)
        self.declare_parameter("mission_loop_delay_seconds", 5.0)

        self._battery_threshold = self.get_parameter("battery_threshold").value
        self._task_timeout = self.get_parameter("task_timeout_seconds").value
        self._auto_generate = self.get_parameter("auto_generate_missions").value
        self._loop_delay = self.get_parameter("mission_loop_delay_seconds").value

        self._cb_group_subs = ReentrantCallbackGroup()
        self._cb_group_timers = MutuallyExclusiveCallbackGroup()

        self._fleet: Dict[str, RobotFleetRecord] = {
            rid: RobotFleetRecord(rid, self.get_logger()) for rid in ROBOT_IDS
        }

        self._scheduler = TaskScheduler()

        self._current_mission: Optional[Dict[str, Any]] = None
        self._mission_commands: List[Dict[str, Any]] = []
        self._pending_tasks: List[ScheduledTask] = []
        self._active_assignments: Dict[str, ScheduledTask] = {}
        self._completed_tasks: List[ScheduledTask] = []
        self._failed_tasks: List[ScheduledTask] = []
        self._mission_loop_count: int = 0
        self._lock = threading.Lock()

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._validated_mission_sub = self.create_subscription(
            String,
            "/warehouse/validated_mission",
            self._mission_callback,
            latching_qos,
            callback_group=self._cb_group_subs,
        )

        for robot_id in ROBOT_IDS:
            self.create_subscription(
                TaskStatus,
                f"/warehouse/{robot_id}/task_status",
                lambda msg, rid=robot_id: self._task_status_callback(msg, rid),
                10,
                callback_group=self._cb_group_subs,
            )
            self.create_subscription(
                BatteryStatus,
                f"/warehouse/{robot_id}/battery_status",
                lambda msg, rid=robot_id: self._battery_callback(msg, rid),
                10,
                callback_group=self._cb_group_subs,
            )
            self.create_subscription(
                RobotState,
                f"/warehouse/{robot_id}/robot_state",
                lambda msg, rid=robot_id: self._robot_state_callback(msg, rid),
                10,
                callback_group=self._cb_group_subs,
            )

        self._task_cmd_pubs: Dict[str, Any] = {}
        self._nav_goal_pubs: Dict[str, Any] = {}
        for robot_id in ROBOT_IDS:
            self._task_cmd_pubs[robot_id] = self.create_publisher(
                String, f"/warehouse/{robot_id}/task_command", 10
            )
            self._nav_goal_pubs[robot_id] = self.create_publisher(
                String, f"/warehouse/{robot_id}/nav_goal", 10
            )

        self._fleet_status_pub = self.create_publisher(
            FleetStatus, "/warehouse/fleet_status", 10
        )
        self._mission_status_pub = self.create_publisher(
            MissionStatus, "/warehouse/mission_status", 10
        )

        self._emergency_srv = self.create_service(
            EmergencyStop,
            "/warehouse/emergency_stop",
            self._emergency_stop_handler,
        )

        period = 1.0 / COORDINATION_RATE_HZ
        self._coord_timer = self.create_timer(
            period, self._coordination_loop, callback_group=self._cb_group_timers
        )
        self._health_timer = self.create_timer(
            5.0, self._robot_health_check, callback_group=self._cb_group_timers
        )
        self._status_timer = self.create_timer(
            2.0, self._publish_fleet_status, callback_group=self._cb_group_timers
        )

        self.get_logger().info(
            "Mission Coordinator (Fleet Manager) started — monitoring "
            f"{len(ROBOT_IDS)} robots continuously"
        )

    def _mission_callback(self, msg: String) -> None:
        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid mission JSON: {exc}")
            return

        if not mission.get("validated"):
            self.get_logger().warning("Received non-validated mission — ignoring")
            return

        with self._lock:
            self.get_logger().info(
                f'Loading mission: "{mission["mission_name"]}" '
                f'({len(mission.get("commands", []))} commands)'
            )
            self._current_mission = mission
            self._mission_commands = deepcopy(mission.get("commands", []))
            self._pending_tasks.clear()
            self._completed_tasks.clear()
            self._failed_tasks.clear()
            self._mission_loop_count = 0
            self._schedule_mission_commands()

    def _schedule_mission_commands(self) -> None:
        robot_records = {rid: f.to_robot_record() for rid, f in self._fleet.items()}
        scheduled = self._scheduler.schedule_commands(self._mission_commands, robot_records)
        self._pending_tasks = scheduled
        self.get_logger().info(f"Scheduled {len(scheduled)} tasks from mission commands")

    def _coordination_loop(self) -> None:
        with self._lock:
            self._assign_pending_tasks()
            self._check_battery_levels()
            self._check_mission_completion()

    def _assign_pending_tasks(self) -> None:
        if not self._pending_tasks:
            return

        unassigned = []
        for task in self._pending_tasks:
            robot_id = task.robot_id
            robot = self._fleet.get(robot_id)

            if robot and robot.is_available and robot_id not in self._active_assignments:
                if robot.battery_percentage > self._battery_threshold + 5.0:
                    self._dispatch_task(robot_id, task)
                else:
                    self.get_logger().warning(
                        f"Robot {robot_id} battery too low ({robot.battery_percentage:.1f}%) "
                        f"to start task — queuing charger"
                    )
                    self._initiate_charging(robot_id)
            else:
                unassigned.append(task)

        for task in list(self._pending_tasks):
            if task not in unassigned and task in self._pending_tasks:
                pass

        self._pending_tasks = [t for t in self._pending_tasks if t in unassigned or
                                t.robot_id in self._active_assignments]

        remaining = []
        for task in self._pending_tasks:
            if task.robot_id in self._active_assignments:
                remaining.append(task)
            elif any(t.task_id == task.task_id for t in unassigned):
                remaining.append(task)
        self._pending_tasks = remaining

    def _dispatch_task(self, robot_id: str, task: ScheduledTask) -> None:
        robot = self._fleet[robot_id]
        robot.current_task = task
        robot.task_start_time = time.monotonic()
        robot.last_task_progress = 0.0
        robot.last_progress_update = time.monotonic()

        task_state_map = {
            "navigate": State.NAVIGATING,
            "patrol": State.PATROLLING,
            "inspect": State.INSPECTING,
            "inventory_scan": State.INSPECTING,
            "package_verify": State.INSPECTING,
            "package_pickup": State.NAVIGATING,
            "package_delivery": State.NAVIGATING,
            "track_target": State.TRACKING,
            "formation": State.FORMATION,
            "navigate_to_charger": State.RETURNING_TO_CHARGER,
            "return_home": State.NAVIGATING,
            "wait": State.WAITING,
            "rotate": State.MOVING,
        }

        new_state = task_state_map.get(task.task_type, State.NAVIGATING)
        robot.state_machine.transition(new_state, reason=f"task:{task.task_type}")

        self._active_assignments[robot_id] = task

        command_msg = {
            "task_id": task.task_id,
            "task_type": task.task_type,
            "goal": task.goal,
            "priority": task.priority,
            "parameters": task.parameters,
        }

        msg = String()
        msg.data = json.dumps(command_msg)
        self._task_cmd_pubs[robot_id].publish(msg)

        self.get_logger().info(
            f"Dispatched to {robot_id}: task={task.task_type} goal={task.goal}"
        )

    def _check_battery_levels(self) -> None:
        for robot_id, robot in self._fleet.items():
            if robot.battery_percentage <= self._battery_threshold:
                if not robot.state_machine.is_charging():
                    self.get_logger().warning(
                        f"Robot {robot_id} battery critical: {robot.battery_percentage:.1f}% "
                        f"— initiating charging"
                    )
                    self._initiate_charging(robot_id)

    def _initiate_charging(self, robot_id: str) -> None:
        robot = self._fleet[robot_id]

        if robot.state_machine.is_charging():
            return

        if robot_id in self._active_assignments:
            robot.saved_task = self._active_assignments.pop(robot_id)
            self.get_logger().info(
                f"Robot {robot_id} saving task {robot.saved_task.task_type} before charging"
            )

        station_id, sx, sy, dist = self._scheduler.get_nearest_charging_station(
            robot.position_x, robot.position_y
        )
        robot.target_charger = station_id

        robot.state_machine.transition(
            State.RETURNING_TO_CHARGER, reason="low_battery"
        )

        charge_cmd = {
            "task_id": str(uuid.uuid4()),
            "task_type": "navigate_to_charger",
            "goal": station_id,
            "priority": 10,
            "parameters": {
                "station": station_id,
                "station_x": sx,
                "station_y": sy,
                "charge_to_percentage": 100.0,
            },
        }
        msg = String()
        msg.data = json.dumps(charge_cmd)
        self._task_cmd_pubs[robot_id].publish(msg)

        self.get_logger().info(
            f"Robot {robot_id} heading to charger {station_id} "
            f"(distance: {dist:.1f}m, battery: {robot.battery_percentage:.1f}%)"
        )

    def _check_mission_completion(self) -> None:
        if self._current_mission is None:
            return

        active_count = len(self._active_assignments)
        pending_count = len(self._pending_tasks)

        if active_count == 0 and pending_count == 0:
            total_cmds = len(self._mission_commands)
            completed = len(self._completed_tasks)
            failed = len(self._failed_tasks)

            if self._current_mission.get("loop", False):
                self._mission_loop_count += 1
                self.get_logger().info(
                    f'Mission "{self._current_mission["mission_name"]}" '
                    f"loop {self._mission_loop_count} complete — restarting"
                )
                self._reschedule_for_available_robots()
            else:
                self.get_logger().info(
                    f'Mission "{self._current_mission["mission_name"]}" complete: '
                    f"{completed} done, {failed} failed"
                )
                if self._auto_generate:
                    self._generate_continuous_tasks()

    def _reschedule_for_available_robots(self) -> None:
        robot_records = {rid: f.to_robot_record() for rid, f in self._fleet.items()}
        available = {rid: r for rid, r in robot_records.items() if r.is_available}

        if not available:
            return

        scheduled = self._scheduler.schedule_commands(self._mission_commands, available)
        self._pending_tasks.extend(scheduled)

    def _generate_continuous_tasks(self) -> None:
        self.get_logger().info("Generating autonomous continuous warehouse tasks")

        continuous_commands = [
            {
                "robot": "Robot_1",
                "task": "patrol",
                "goal": "Waypoint_1",
                "priority": 4,
                "parameters": {
                    "waypoints": ["Waypoint_1", "Waypoint_2", "Waypoint_3", "Waypoint_4"],
                    "loop": True,
                    "speed": 0.3,
                },
            },
            {
                "robot": "Robot_2",
                "task": "patrol",
                "goal": "Waypoint_5",
                "priority": 4,
                "parameters": {
                    "waypoints": ["Waypoint_5", "Waypoint_6", "Waypoint_7", "Waypoint_8"],
                    "loop": True,
                    "speed": 0.3,
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
                    "speed": 0.2,
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
                    "speed": 0.2,
                },
            },
        ]

        robot_records = {rid: f.to_robot_record() for rid, f in self._fleet.items()}
        scheduled = self._scheduler.schedule_commands(continuous_commands, robot_records)
        self._pending_tasks.extend(scheduled)
        self.get_logger().info(f"Generated {len(scheduled)} autonomous tasks")

    def _task_status_callback(self, msg: TaskStatus, robot_id: str) -> None:
        robot = self._fleet.get(robot_id)
        if not robot:
            return

        robot.last_task_progress = msg.progress
        robot.last_progress_update = time.monotonic()

        if msg.completed:
            with self._lock:
                self.get_logger().info(
                    f"Robot {robot_id} completed task: {msg.task_type} ({msg.goal})"
                )
                robot.completed_tasks += 1
                task = self._active_assignments.pop(robot_id, None)
                if task:
                    self._completed_tasks.append(task)
                robot.current_task = None
                robot.state_machine.transition(State.IDLE, reason="task_complete")

                if robot.saved_task and robot.battery_percentage > self._battery_threshold + 10.0:
                    self.get_logger().info(
                        f"Robot {robot_id} resuming saved task: {robot.saved_task.task_type}"
                    )
                    robot.state_machine.transition(State.RESUMING, reason="resuming_after_charge")
                    self._pending_tasks.insert(0, robot.saved_task)
                    robot.saved_task = None

        elif msg.failed:
            with self._lock:
                self.get_logger().warning(
                    f"Robot {robot_id} failed task: {msg.task_type} — {msg.error_message}"
                )
                robot.failed_tasks += 1
                task = self._active_assignments.pop(robot_id, None)
                if task:
                    self._failed_tasks.append(task)
                robot.current_task = None
                robot.state_machine.transition(State.IDLE, reason="task_failed")

    def _battery_callback(self, msg: BatteryStatus, robot_id: str) -> None:
        robot = self._fleet.get(robot_id)
        if not robot:
            return
        robot.battery_percentage = msg.percentage
        robot.is_charging = msg.is_charging

        if msg.is_charging:
            if not robot.state_machine.is_charging():
                robot.state_machine.transition(State.CHARGING, reason="charging_started", force=True)
        elif msg.percentage >= 99.0 and robot.state_machine.state == State.CHARGING:
            self.get_logger().info(f"Robot {robot_id} fully charged — returning to duty")
            robot.state_machine.transition(State.IDLE, reason="charging_complete")
            robot.target_charger = None

    def _robot_state_callback(self, msg: RobotState, robot_id: str) -> None:
        robot = self._fleet.get(robot_id)
        if not robot:
            return
        robot.position_x = msg.position.x
        robot.position_y = msg.position.y
        robot.speed = msg.speed
        robot.zone = msg.zone

    def _robot_health_check(self) -> None:
        now = time.monotonic()
        with self._lock:
            for robot_id, robot in self._fleet.items():
                task = self._active_assignments.get(robot_id)
                if task is None:
                    continue

                time_on_task = now - robot.task_start_time
                if time_on_task > self._task_timeout:
                    self.get_logger().warning(
                        f"Robot {robot_id} task timeout ({time_on_task:.0f}s) "
                        f"on {task.task_type} — cancelling"
                    )
                    robot.failed_tasks += 1
                    self._active_assignments.pop(robot_id, None)
                    self._failed_tasks.append(task)
                    robot.current_task = None
                    robot.state_machine.transition(State.IDLE, reason="task_timeout", force=True)
                    continue

                stall_time = now - robot.last_progress_update
                if (
                    stall_time > STALL_DETECTION_SECONDS
                    and robot.state_machine.is_working()
                    and task.task_type not in ["patrol", "formation"]
                ):
                    self.get_logger().warning(
                        f"Robot {robot_id} appears stalled for {stall_time:.0f}s — recovering"
                    )
                    robot.last_progress_update = now

    def _publish_fleet_status(self) -> None:
        msg = FleetStatus()

        if self._current_mission:
            msg.mission_name = self._current_mission.get("mission_name", "")
            msg.mission_type = self._current_mission.get("mission_type", "")
            msg.is_looping = self._current_mission.get("loop", False)
        else:
            msg.mission_name = "No Mission"
            msg.mission_type = "NONE"
            msg.is_looping = False

        msg.total_robots = len(self._fleet)
        msg.active_robots = sum(1 for r in self._fleet.values() if r.state_machine.is_working())
        msg.charging_robots = sum(1 for r in self._fleet.values() if r.state_machine.is_charging())
        msg.idle_robots = sum(1 for r in self._fleet.values() if r.is_available)
        msg.error_robots = sum(1 for r in self._fleet.values() if r.state_machine.is_error())
        msg.completed_tasks = len(self._completed_tasks)
        msg.failed_tasks = len(self._failed_tasks)
        msg.pending_tasks = len(self._pending_tasks)
        msg.mission_loop_count = self._mission_loop_count

        total_tasks = msg.completed_tasks + msg.failed_tasks + msg.pending_tasks
        total_tasks += len(self._active_assignments)
        if total_tasks > 0:
            msg.mission_progress = float(msg.completed_tasks) / float(total_tasks)
        else:
            msg.mission_progress = 0.0

        for robot_id, robot in self._fleet.items():
            info = RobotInfo()
            info.robot_id = robot_id
            info.state = robot.state_machine.state_name
            info.battery_percentage = robot.battery_percentage
            info.is_charging = robot.is_charging
            info.is_available = robot.is_available
            pos = Pose2D()
            pos.x = robot.position_x
            pos.y = robot.position_y
            info.position = pos
            info.speed = robot.speed
            info.zone = robot.zone
            if robot.current_task:
                info.current_task = robot.current_task.task_type
                info.task_id = robot.current_task.task_id
                info.goal = robot.current_task.goal
            if robot.target_charger:
                info.charging_station = robot.target_charger
            info.completed_tasks = robot.completed_tasks
            info.failed_tasks = robot.failed_tasks
            msg.robots.append(info)

        from builtin_interfaces.msg import Time
        msg.timestamp = self.get_clock().now().to_msg()
        self._fleet_status_pub.publish(msg)

        if self._current_mission:
            status = MissionStatus()
            status.mission_name = msg.mission_name
            status.mission_type = msg.mission_type
            status.state = "EXECUTING" if msg.active_robots > 0 else "WAITING"
            status.progress = msg.mission_progress
            status.is_looping = msg.is_looping
            status.loop_count = self._mission_loop_count
            status.total_commands = len(self._mission_commands)
            status.completed_commands = msg.completed_tasks
            status.failed_commands = msg.failed_tasks
            status.pending_commands = msg.pending_tasks
            status.active_robot_ids = list(self._active_assignments.keys())
            self._mission_status_pub.publish(status)

    def _emergency_stop_handler(
        self, request: EmergencyStop.Request, response: EmergencyStop.Response
    ) -> EmergencyStop.Response:
        targets = ROBOT_IDS if request.stop_all else list(request.robot_ids)
        stopped = []
        failed = []

        with self._lock:
            for robot_id in targets:
                robot = self._fleet.get(robot_id)
                if robot:
                    robot.state_machine.transition(
                        State.EMERGENCY_STOP,
                        reason=f"emergency: {request.reason}",
                        force=True,
                    )
                    task = self._active_assignments.pop(robot_id, None)
                    robot.current_task = None
                    stop_cmd = {"task_type": "emergency_stop", "reason": request.reason}
                    msg = String()
                    msg.data = json.dumps(stop_cmd)
                    self._task_cmd_pubs[robot_id].publish(msg)
                    stopped.append(robot_id)
                else:
                    failed.append(robot_id)

        response.success = len(failed) == 0
        response.robots_stopped = len(stopped)
        response.stopped_robot_ids = stopped
        response.failed_robot_ids = failed
        response.message = (
            f"Emergency stop: {len(stopped)} robots stopped. Reason: {request.reason}"
        )
        self.get_logger().error(
            f"EMERGENCY STOP: {len(stopped)} robots halted. Reason: {request.reason}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
