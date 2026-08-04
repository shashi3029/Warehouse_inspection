#!/usr/bin/env python3
"""
Inspection Detector Node

Runs shelf and inventory inspection for each warehouse robot. Publishes
Detection messages for every object found within the robot's sensor range.

Detection types: SHELF, INVENTORY, PACKAGE, OBSTACLE, ROBOT, HUMAN, TARGET,
                 UNKNOWN_OBJECT

In simulation, detections are generated synthetically from the robot's
odometry position and the known warehouse shelf geometry. In a real
deployment this node subscribes to the camera and LiDAR topics and passes
frames through an on-board inference model (e.g. YOLO or a custom CNN).
"""

import math
import random
import time
import uuid
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from warehouse_msgs.msg import Detection, RobotState


ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]

SHELF_POSITIONS = {
    "Shelf_A": (-12.0, 0.0),
    "Shelf_B": ( -4.0, 0.0),
    "Shelf_C": (  4.0, 0.0),
    "Shelf_D": ( 12.0, 0.0),
}

DETECTION_RANGE = 5.0

DETECTION_CLASSES = [
    ("SHELF",          0.95),
    ("INVENTORY",      0.88),
    ("PACKAGE",        0.82),
    ("OBSTACLE",       0.75),
    ("ROBOT",          0.90),
    ("HUMAN",          0.70),
    ("TARGET",         0.85),
    ("UNKNOWN_OBJECT", 0.60),
]


def robot_ns(robot_id: str) -> str:
    return "/" + robot_id.lower().replace("_", "")


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class InspectionDetectorNode(Node):

    def __init__(self, robot_id: str):
        ns = robot_ns(robot_id)
        super().__init__(f"inspection_detector_{robot_id.lower().replace('_', '')}")

        self._robot_id = robot_id
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_theta: float = 0.0
        self._robot_state: str = "IDLE"

        self._odom_sub = self.create_subscription(
            Odometry,
            f"{ns}/odom",
            self._odom_callback,
            10,
        )
        self._state_sub = self.create_subscription(
            RobotState,
            f"/warehouse/{robot_id}/robot_state",
            self._state_callback,
            10,
        )

        self._detection_pub = self.create_publisher(
            Detection, f"/warehouse/{robot_id}/detections", 10
        )
        self._all_detections_pub = self.create_publisher(
            Detection, "/warehouse/all_detections", 10
        )

        self.create_timer(0.5, self._detection_loop)

        self.get_logger().info(
            f"InspectionDetector started for {robot_id}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _state_callback(self, msg: RobotState) -> None:
        self._robot_state = msg.state

    def _detection_loop(self) -> None:
        if self._robot_state in ("IDLE", "CHARGING", "EMERGENCY_STOP"):
            return

        detections = self._compute_detections()
        for det in detections:
            self._detection_pub.publish(det)
            self._all_detections_pub.publish(det)

    def _compute_detections(self) -> List[Detection]:
        results: List[Detection] = []

        for shelf_name, (sx, sy) in SHELF_POSITIONS.items():
            dist = math.sqrt((sx - self._robot_x) ** 2 + (sy - self._robot_y) ** 2)
            if dist <= DETECTION_RANGE:
                det = self._make_detection(
                    "SHELF", shelf_name, sx, sy, min(0.99, 0.85 + random.uniform(0, 0.1))
                )
                results.append(det)

                if random.random() < 0.7:
                    for i in range(random.randint(1, 4)):
                        jx = sx + random.uniform(-1.5, 1.5)
                        jy = sy + random.uniform(-0.3, 0.3)
                        results.append(self._make_detection(
                            "INVENTORY",
                            f"item_{shelf_name}_{i}",
                            jx, jy,
                            random.uniform(0.75, 0.95),
                        ))

        if random.random() < 0.1:
            ox = self._robot_x + random.uniform(-3, 3)
            oy = self._robot_y + random.uniform(-3, 3)
            results.append(self._make_detection(
                "PACKAGE", f"pkg_{uuid.uuid4().hex[:6]}", ox, oy, random.uniform(0.7, 0.9)
            ))

        if random.random() < 0.05:
            ox = self._robot_x + random.uniform(-4, 4)
            oy = self._robot_y + random.uniform(-4, 4)
            results.append(self._make_detection(
                "OBSTACLE", f"obs_{uuid.uuid4().hex[:4]}", ox, oy, random.uniform(0.65, 0.85)
            ))

        if random.random() < 0.03 and self._robot_state == "TRACKING":
            tx = self._robot_x + random.uniform(-3, 3)
            ty = self._robot_y + random.uniform(-3, 3)
            results.append(self._make_detection(
                "TARGET", "moving_package_1", tx, ty, random.uniform(0.75, 0.92)
            ))

        return results

    def _make_detection(
        self,
        det_type: str,
        obj_id: str,
        x: float,
        y: float,
        confidence: float,
    ) -> Detection:
        det = Detection()
        det.detection_id = str(uuid.uuid4())
        det.robot_id = self._robot_id
        det.detection_type = det_type
        det.object_id = obj_id
        det.object_class = det_type
        det.confidence = confidence
        det.distance = math.sqrt((x - self._robot_x) ** 2 + (y - self._robot_y) ** 2)
        det.is_obstacle = det_type == "OBSTACLE"
        det.is_target = det_type == "TARGET"
        det.is_human = det_type == "HUMAN"

        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        pose.orientation = yaw_to_quaternion(0.0)
        det.pose = pose
        det.timestamp = self.get_clock().now().to_msg()
        return det


def main(args=None):
    rclpy.init(args=args)

    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "Robot_1"

    node = InspectionDetectorNode(robot_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
