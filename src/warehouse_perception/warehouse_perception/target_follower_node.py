#!/usr/bin/env python3
"""
Target Follower Node

Subscribes to Detection messages and follows the highest-confidence
TARGET detection within range. Maintains a safe follow distance using
a proportional velocity controller. When the target is lost for longer
than the configured timeout the robot publishes a task-resume signal
to the Mission Coordinator so the previous mission task can continue.
"""

import math
import time
import json
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from warehouse_msgs.msg import Detection, TaskStatus


ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]


def robot_ns(robot_id: str) -> str:
    return "/" + robot_id.lower().replace("_", "")


class TargetFollowerNode(Node):

    def __init__(self, robot_id: str):
        super().__init__(f"target_follower_{robot_id.lower().replace('_', '')}")

        self._robot_id = robot_id

        self.declare_parameter("follow_distance", 1.5)
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("lost_timeout_seconds", 10.0)
        self.declare_parameter("detection_confidence_threshold", 0.70)
        self.declare_parameter("linear_gain", 0.6)
        self.declare_parameter("angular_gain", 1.2)

        self._follow_distance = self.get_parameter("follow_distance").value
        self._max_linear = self.get_parameter("max_linear_speed").value
        self._max_angular = self.get_parameter("max_angular_speed").value
        self._lost_timeout = self.get_parameter("lost_timeout_seconds").value
        self._conf_threshold = self.get_parameter("detection_confidence_threshold").value
        self._kp_linear = self.get_parameter("linear_gain").value
        self._kp_angular = self.get_parameter("angular_gain").value

        self._active_target: Optional[Detection] = None
        self._target_last_seen: float = 0.0
        self._following: bool = False
        self._current_task_id: Optional[str] = None

        ns = robot_ns(robot_id)

        self._detection_sub = self.create_subscription(
            Detection,
            f"/warehouse/{robot_id}/detections",
            self._detection_callback,
            10,
        )
        self._task_cmd_sub = self.create_subscription(
            String,
            f"/warehouse/{robot_id}/task_command",
            self._task_command_callback,
            10,
        )

        self._cmd_vel_pub = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)
        self._task_status_pub = self.create_publisher(
            TaskStatus, f"/warehouse/{robot_id}/task_status", 10
        )
        self._resume_signal_pub = self.create_publisher(
            String, f"/warehouse/{robot_id}/resume_previous_task", 10
        )

        self._control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f"TargetFollower ready for {robot_id} "
            f"(follow_dist={self._follow_distance}m, timeout={self._lost_timeout}s)"
        )

    def _task_command_callback(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if cmd.get("task_type") == "track_target":
            self._current_task_id = cmd.get("task_id")
            self._following = True
            self._active_target = None
            self._target_last_seen = 0.0
            self.get_logger().info(f"[{self._robot_id}] Target following activated")

        elif cmd.get("task_type") == "emergency_stop":
            self._stop_following()

    def _detection_callback(self, msg: Detection) -> None:
        if not self._following:
            return
        if not msg.is_target:
            return
        if msg.confidence < self._conf_threshold:
            return

        if self._active_target is None or msg.confidence > self._active_target.confidence:
            self._active_target = msg
            self._target_last_seen = time.monotonic()

    def _control_loop(self) -> None:
        if not self._following:
            return

        now = time.monotonic()

        if self._active_target is not None:
            time_since_seen = now - self._target_last_seen
            if time_since_seen > 0.5:
                self._active_target = None

        if self._active_target is None:
            self._cmd_vel_pub.publish(Twist())

            if self._target_last_seen > 0.0:
                lost_duration = now - self._target_last_seen
                if lost_duration > self._lost_timeout:
                    self.get_logger().info(
                        f"[{self._robot_id}] Target lost for {lost_duration:.1f}s — "
                        f"resuming previous task"
                    )
                    self._stop_following()
                    self._signal_resume()
            return

        error_dist = self._active_target.distance - self._follow_distance

        bearing = math.atan2(
            self._active_target.pose.position.y,
            self._active_target.pose.position.x,
        )

        linear_vel = max(-self._max_linear,
                         min(self._max_linear, self._kp_linear * error_dist))
        angular_vel = max(-self._max_angular,
                          min(self._max_angular, self._kp_angular * bearing))

        if abs(error_dist) < 0.1:
            linear_vel = 0.0

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self._cmd_vel_pub.publish(twist)

        progress = min(0.95, (now - self._target_last_seen + 1.0) / 30.0
                       if self._target_last_seen else 0.5)
        self._publish_task_status(progress)

    def _stop_following(self) -> None:
        self._following = False
        self._active_target = None
        self._cmd_vel_pub.publish(Twist())
        if self._current_task_id:
            status = TaskStatus()
            status.task_id = self._current_task_id
            status.robot_id = self._robot_id
            status.task_type = "track_target"
            status.task_state = "COMPLETED"
            status.completed = True
            status.progress = 1.0
            status.update_time = self.get_clock().now().to_msg()
            self._task_status_pub.publish(status)

    def _signal_resume(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "robot_id": self._robot_id,
            "action": "resume_previous_task",
            "reason": "target_lost",
        })
        self._resume_signal_pub.publish(msg)

    def _publish_task_status(self, progress: float) -> None:
        if not self._current_task_id:
            return
        status = TaskStatus()
        status.task_id = self._current_task_id
        status.robot_id = self._robot_id
        status.task_type = "track_target"
        status.task_state = "EXECUTING"
        status.progress = progress
        status.completed = False
        status.failed = False
        status.update_time = self.get_clock().now().to_msg()
        self._task_status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)

    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "Robot_1"

    node = TargetFollowerNode(robot_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
