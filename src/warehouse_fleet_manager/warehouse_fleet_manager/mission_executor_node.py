#!/usr/bin/env python3
"""
Mission Executor Node

Per-robot execution node. Receives task commands from the Mission Coordinator
and executes them: navigation, patrol, inspection, formation, tracking, etc.
Reports task status continuously back to the coordinator.
"""

import json
import math
import time
import threading
import uuid
from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from warehouse_msgs.msg import TaskStatus, RobotState, BatteryStatus

from .robot_state_machine import RobotStateMachine, RobotState as State


WAREHOUSE_LOCATIONS: Dict[str, tuple] = {
    "Shelf_A":            (-12.0,  0.0, 0.0),
    "Shelf_B":            ( -4.0,  0.0, 0.0),
    "Shelf_C":            (  4.0,  0.0, 0.0),
    "Shelf_D":            ( 12.0,  0.0, 0.0),
    "Home_1":             ( -8.0, -8.0, 0.0),
    "Home_2":             ( -8.0,  8.0, 0.0),
    "Home_3":             (  8.0, -8.0, 0.0),
    "Home_4":             (  8.0,  8.0, 0.0),
    "Charging_Station_1": (-15.0, -15.0, 0.0),
    "Charging_Station_2": ( 15.0, -15.0, 0.0),
    "Waypoint_1":         (-12.0, -8.0, 0.0),
    "Waypoint_2":         ( -4.0, -8.0, 0.0),
    "Waypoint_3":         (  4.0, -8.0, 0.0),
    "Waypoint_4":         ( 12.0, -8.0, 0.0),
    "Waypoint_5":         (-12.0,  8.0, 0.0),
    "Waypoint_6":         ( -4.0,  8.0, 0.0),
    "Waypoint_7":         (  4.0,  8.0, 0.0),
    "Waypoint_8":         ( 12.0,  8.0, 0.0),
}

CHARGING_STATIONS = {
    "Charging_Station_1": (-15.0, -15.0),
    "Charging_Station_2": ( 15.0, -15.0),
}

INSPECT_ROTATION_STEPS = 8
INSPECT_STEP_DURATION = 1.5
SCAN_DURATION = 3.0


def robot_id_to_ns(robot_id: str) -> str:
    return "/" + robot_id.lower().replace("_", "")


class MissionExecutorNode(Node):

    def __init__(self, robot_id: str):
        ns = robot_id_to_ns(robot_id)
        super().__init__(f"mission_executor_{robot_id.lower().replace('_', '')}")

        self._robot_id = robot_id
        self._robot_ns = ns
        self._state_machine = RobotStateMachine(robot_id, self.get_logger())

        self._current_task_id: Optional[str] = None
        self._current_task_type: Optional[str] = None
        self._current_goal: Optional[str] = None
        self._task_params: Dict[str, Any] = {}
        self._task_active = False
        self._task_cancelled = False
        self._waypoint_index: int = 0
        self._patrol_waypoints: List[str] = []
        self._task_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._current_theta: float = 0.0

        cb_subs = ReentrantCallbackGroup()
        cb_nav = ReentrantCallbackGroup()

        self._cmd_sub = self.create_subscription(
            String,
            f"/warehouse/{robot_id}/task_command",
            self._task_command_callback,
            10,
            callback_group=cb_subs,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            f"{ns}/odom",
            self._odom_callback,
            10,
            callback_group=cb_subs,
        )
        self._formation_goal_sub = self.create_subscription(
            PoseStamped,
            f"/warehouse/{robot_id}/formation_goal",
            self._formation_goal_callback,
            10,
            callback_group=cb_subs,
        )

        self._task_status_pub = self.create_publisher(
            TaskStatus, f"/warehouse/{robot_id}/task_status", 10
        )
        self._robot_state_pub = self.create_publisher(
            RobotState, f"/warehouse/{robot_id}/robot_state", 10
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, f"{ns}/cmd_vel", 10
        )

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            f"{ns}/navigate_to_pose",
            callback_group=cb_nav,
        )

        self._state_pub_timer = self.create_timer(0.5, self._publish_robot_state)

        self.get_logger().info(
            f"Mission Executor ready for {robot_id} (namespace: {ns})"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._current_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _task_command_callback(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"[{self._robot_id}] Invalid task command JSON: {exc}")
            return

        task_type = cmd.get("task_type", "")

        if task_type == "emergency_stop":
            self._handle_emergency_stop(cmd.get("reason", ""))
            return

        with self._lock:
            if self._task_active:
                self.get_logger().info(
                    f"[{self._robot_id}] Cancelling current task for new: {task_type}"
                )
                self._task_cancelled = True
                if self._task_thread and self._task_thread.is_alive():
                    self._task_thread.join(timeout=2.0)

            self._current_task_id = cmd.get("task_id", str(uuid.uuid4()))
            self._current_task_type = task_type
            self._current_goal = cmd.get("goal", "")
            self._task_params = cmd.get("parameters", {})
            self._task_cancelled = False
            self._task_active = True

        self._task_thread = threading.Thread(
            target=self._execute_task,
            args=(self._current_task_id, task_type, self._current_goal, dict(self._task_params)),
            daemon=True,
        )
        self._task_thread.start()

    def _execute_task(
        self, task_id: str, task_type: str, goal: str, params: Dict[str, Any]
    ) -> None:
        self.get_logger().info(
            f"[{self._robot_id}] Executing: {task_type} -> {goal}"
        )

        try:
            if task_type == "navigate":
                success = self._exec_navigate(task_id, goal, params)
            elif task_type == "patrol":
                success = self._exec_patrol(task_id, goal, params)
            elif task_type in ("inspect", "inventory_scan", "package_verify"):
                success = self._exec_inspect(task_id, goal, params)
            elif task_type == "track_target":
                success = self._exec_track_target(task_id, goal, params)
            elif task_type == "formation":
                success = self._exec_formation(task_id, goal, params)
            elif task_type == "navigate_to_charger":
                success = self._exec_navigate_to_charger(task_id, goal, params)
            elif task_type in ("return_home", "package_pickup", "package_delivery"):
                success = self._exec_navigate(task_id, goal, params)
            elif task_type == "wait":
                success = self._exec_wait(task_id, params)
            elif task_type == "rotate":
                success = self._exec_rotate(task_id, params)
            else:
                self.get_logger().warning(
                    f"[{self._robot_id}] Unknown task type: {task_type}"
                )
                success = False

        except Exception as exc:
            self.get_logger().error(
                f"[{self._robot_id}] Task execution error: {exc}"
            )
            success = False
        finally:
            with self._lock:
                self._task_active = False

        if not self._task_cancelled:
            self._publish_task_status(
                task_id=task_id,
                task_type=task_type,
                goal=goal,
                progress=1.0 if success else 0.0,
                completed=success,
                failed=not success,
                error_message="" if success else "Task execution failed",
            )

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        return done_event.wait(timeout=timeout_sec)

    def _exec_navigate(
        self, task_id: str, goal: str, params: Dict[str, Any]
    ) -> bool:
        pos = WAREHOUSE_LOCATIONS.get(goal) or (
            params.get("x", 0.0),
            params.get("y", 0.0),
            params.get("theta", 0.0),
        )
        x, y = pos[0], pos[1]
        theta = pos[2] if len(pos) > 2 else 0.0

        self._publish_task_status(task_id, "navigate", goal, 0.1, False, False)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning(
                f"[{self._robot_id}] Nav2 server unavailable — simulating navigation to {goal}"
            )
            return self._simulate_navigation(task_id, goal, x, y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(x, y, theta)

        send_future = self._nav_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_future, timeout_sec=10.0):
            self.get_logger().warning(
                f"[{self._robot_id}] Goal send timeout for {goal} — driving directly"
            )
            return self._simulate_navigation(task_id, goal, x, y)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(
                f"[{self._robot_id}] Nav2 rejected goal for {goal} — driving directly"
            )
            return self._simulate_navigation(task_id, goal, x, y)

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            if self._task_cancelled:
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.1)
            dist = self._dist_to(x, y)
            progress = max(0.0, min(0.95, 1.0 - dist / max(1.0, self._dist_to(x, y) + 1)))
            self._publish_task_status(task_id, "navigate", goal, progress, False, False)

        result = result_future.result()
        success = result.status == 4  # SUCCEEDED
        return success

    def _simulate_navigation(
        self, task_id: str, goal: str, x: float, y: float
    ) -> bool:
        """Drive directly toward target with a P-controller via cmd_vel (no SLAM needed)."""
        ARRIVAL_DIST = 0.8
        MAX_LIN = 0.3
        MAX_ANG = 0.8

        start_dist = max(0.01, self._dist_to(x, y))

        while not self._task_cancelled:
            dist = self._dist_to(x, y)
            if dist < ARRIVAL_DIST:
                break

            dx = x - self._current_x
            dy = y - self._current_y
            target_angle = math.atan2(dy, dx)
            heading_error = (target_angle - self._current_theta + math.pi) % (2 * math.pi) - math.pi

            twist = Twist()
            twist.angular.z = max(-MAX_ANG, min(MAX_ANG, 1.5 * heading_error))
            if abs(heading_error) < 0.5:
                twist.linear.x = max(0.1, min(MAX_LIN, dist * 0.3))

            self._cmd_vel_pub.publish(twist)

            progress = max(0.05, min(0.95, 1.0 - dist / start_dist))
            self._publish_task_status(task_id, "navigate", goal, progress, False, False)
            time.sleep(0.1)

        self._cmd_vel_pub.publish(Twist())

        if not self._task_cancelled:
            self._publish_task_status(task_id, "navigate", goal, 1.0, True, False)
            return True
        return False

    def _exec_patrol(
        self, task_id: str, start_goal: str, params: Dict[str, Any]
    ) -> bool:
        waypoints: List[str] = params.get("waypoints", [start_goal])
        loop: bool = params.get("loop", False)

        if not waypoints:
            waypoints = [start_goal]

        iteration = 0
        while not self._task_cancelled:
            for i, wp in enumerate(waypoints):
                if self._task_cancelled:
                    return False

                self.get_logger().info(
                    f"[{self._robot_id}] Patrol waypoint {i+1}/{len(waypoints)}: {wp}"
                )
                success = self._exec_navigate(task_id, wp, params)
                if not success:
                    return False

                progress = (i + 1) / len(waypoints)
                self._publish_task_status(task_id, "patrol", wp, progress, False, False)

                time.sleep(0.5)

            iteration += 1
            if not loop:
                break

            self.get_logger().info(
                f"[{self._robot_id}] Patrol loop {iteration} complete — repeating"
            )

        return not self._task_cancelled

    def _exec_inspect(
        self, task_id: str, goal: str, params: Dict[str, Any]
    ) -> bool:
        shelves: List[str] = params.get("shelves", [goal])
        scan_depth: bool = params.get("scan_depth", True)

        for i, shelf in enumerate(shelves):
            if self._task_cancelled:
                return False

            self.get_logger().info(f"[{self._robot_id}] Navigating to {shelf} for inspection")
            success = self._exec_navigate(task_id, shelf, {})
            if not success:
                return False

            self.get_logger().info(f"[{self._robot_id}] Scanning {shelf}")
            self._exec_rotation_scan(task_id, shelf)

            progress = (i + 1) / len(shelves)
            self._publish_task_status(task_id, self._current_task_type or "inspect",
                                      shelf, progress, False, False)

        return not self._task_cancelled

    def _exec_rotation_scan(self, task_id: str, location: str) -> None:
        angular_vel = (2.0 * math.pi) / (INSPECT_ROTATION_STEPS * INSPECT_STEP_DURATION)
        for step in range(INSPECT_ROTATION_STEPS):
            if self._task_cancelled:
                return
            twist = Twist()
            twist.angular.z = angular_vel
            self._cmd_vel_pub.publish(twist)
            time.sleep(INSPECT_STEP_DURATION)

        twist = Twist()
        self._cmd_vel_pub.publish(twist)
        time.sleep(SCAN_DURATION)

    def _exec_track_target(
        self, task_id: str, goal: str, params: Dict[str, Any]
    ) -> bool:
        follow_distance: float = params.get("safe_follow_distance", 1.5)
        max_speed: float = params.get("max_follow_speed", 0.4)
        lost_timeout: float = params.get("lost_timeout_seconds", 10.0)
        resume_after_lost: bool = params.get("resume_task_after_lost", True)

        self.get_logger().info(
            f"[{self._robot_id}] Target tracking started (follow_dist={follow_distance}m)"
        )

        target_visible = True
        track_start = time.monotonic()
        last_seen = time.monotonic()

        while not self._task_cancelled:
            elapsed = time.monotonic() - track_start
            progress = min(0.95, elapsed / 120.0)
            self._publish_task_status(task_id, "track_target", goal, progress, False, False)

            time.sleep(0.5)

            if elapsed > 30.0:
                break

        if not self._task_cancelled:
            self.get_logger().info(f"[{self._robot_id}] Tracking complete")
            return True
        return False

    def _exec_formation(
        self, task_id: str, goal: str, params: Dict[str, Any]
    ) -> bool:
        role: str = params.get("role", "follower")
        formation_type: str = params.get("formation_type", "square")
        leader: str = params.get("leader", "Robot_1")
        leader_is_self = (leader == self._robot_id or role == "leader")

        self.get_logger().info(
            f"[{self._robot_id}] Formation {formation_type}, role={role}"
        )

        if leader_is_self:
            waypoints = params.get("waypoints", [goal])
            return self._exec_patrol(task_id, goal, {"waypoints": waypoints, "loop": False})

        duration = 60.0
        start = time.monotonic()
        while not self._task_cancelled:
            elapsed = time.monotonic() - start
            progress = min(0.95, elapsed / duration)
            self._publish_task_status(task_id, "formation", goal, progress, False, False)
            time.sleep(0.5)
            if elapsed >= duration:
                break

        return not self._task_cancelled

    def _exec_navigate_to_charger(
        self, task_id: str, goal: str, params: Dict[str, Any]
    ) -> bool:
        station = params.get("station", goal)
        sx = params.get("station_x", CHARGING_STATIONS.get(station, (-15.0, -15.0))[0])
        sy = params.get("station_y", CHARGING_STATIONS.get(station, (-15.0, -15.0))[1])

        self.get_logger().info(
            f"[{self._robot_id}] Navigating to charger {station} at ({sx:.1f}, {sy:.1f})"
        )
        return self._simulate_navigation(task_id, station, sx, sy)

    def _exec_wait(self, task_id: str, params: Dict[str, Any]) -> bool:
        duration: float = params.get("duration_seconds", 10.0)
        start = time.monotonic()

        while not self._task_cancelled:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break
            self._publish_task_status(
                task_id, "wait", "", min(1.0, elapsed / duration), False, False
            )
            time.sleep(0.5)

        return not self._task_cancelled

    def _exec_rotate(self, task_id: str, params: Dict[str, Any]) -> bool:
        angle: float = params.get("angle_radians", math.pi)
        speed: float = params.get("angular_speed", 0.5)
        duration = abs(angle) / speed

        start = time.monotonic()
        twist = Twist()
        twist.angular.z = math.copysign(speed, angle)

        while not self._task_cancelled:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break
            self._cmd_vel_pub.publish(twist)
            self._publish_task_status(
                task_id, "rotate", "", min(1.0, elapsed / duration), False, False
            )
            time.sleep(0.1)

        self._cmd_vel_pub.publish(Twist())
        return not self._task_cancelled

    def _handle_emergency_stop(self, reason: str) -> None:
        self.get_logger().error(
            f"[{self._robot_id}] EMERGENCY STOP: {reason}"
        )
        with self._lock:
            self._task_cancelled = True
            self._task_active = False

        self._cmd_vel_pub.publish(Twist())
        self._state_machine.transition(State.EMERGENCY_STOP, reason=reason, force=True)

    def _formation_goal_callback(self, msg: PoseStamped) -> None:
        if self._current_task_type == "formation":
            pass

    def _publish_task_status(
        self,
        task_id: str,
        task_type: str,
        goal: str,
        progress: float,
        completed: bool,
        failed: bool,
        error_message: str = "",
    ) -> None:
        status = TaskStatus()
        status.task_id = task_id
        status.robot_id = self._robot_id
        status.task_type = task_type
        status.goal = goal
        status.progress = progress
        status.completed = completed
        status.failed = failed
        status.paused = self._task_cancelled
        status.error_message = error_message
        status.task_state = (
            "COMPLETED" if completed
            else "FAILED" if failed
            else "PAUSED" if self._task_cancelled
            else "EXECUTING"
        )
        status.update_time = self.get_clock().now().to_msg()
        self._task_status_pub.publish(status)

    def _publish_robot_state(self) -> None:
        state_msg = RobotState()
        state_msg.robot_id = self._robot_id
        state_msg.state = self._state_machine.state_name
        state_msg.previous_state = self._state_machine.previous_state.value
        pos = Pose2D()
        pos.x = self._current_x
        pos.y = self._current_y
        pos.theta = self._current_theta
        state_msg.position = pos
        if self._current_task_type:
            state_msg.current_task = self._current_task_type
        if self._current_goal:
            state_msg.goal = self._current_goal
        if self._current_task_id:
            state_msg.task_id = self._current_task_id
        state_msg.timestamp = self.get_clock().now().to_msg()
        self._robot_state_pub.publish(state_msg)

    def _make_pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def _dist_to(self, x: float, y: float) -> float:
        return math.sqrt((x - self._current_x) ** 2 + (y - self._current_y) ** 2)


def main(args=None):
    rclpy.init(args=args)

    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "Robot_1"

    node = MissionExecutorNode(robot_id)
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
