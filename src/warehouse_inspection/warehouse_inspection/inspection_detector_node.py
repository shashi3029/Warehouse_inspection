#!/usr/bin/env python3
"""
Inspection Detector Node

Combines two detection sources:
  1. Proximity-based: triggers when robot is within range of known shelf positions.
  2. LiDAR scan: clusters laser returns to detect any object (packages, obstacles,
     unknown items) that is not part of the warehouse structure.

The odom frame origin is the robot's spawn position.  All coordinates are
corrected back to world frame by adding the spawn offset so that distances to
world-frame shelf positions are computed correctly.
"""

import math
import random
import time
import uuid
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from warehouse_msgs.msg import Detection, RobotState


ROBOT_SPAWN_POSITIONS: Dict[str, Tuple[float, float]] = {
    "Robot_1": (-8.0, -8.0),
    "Robot_2": (-8.0,  8.0),
    "Robot_3": ( 8.0, -8.0),
    "Robot_4": ( 8.0,  8.0),
}

SHELF_POSITIONS = {
    "Shelf_A": (-12.0, 0.0),
    "Shelf_B": ( -4.0, 0.0),
    "Shelf_C": (  4.0, 0.0),
    "Shelf_D": ( 12.0, 0.0),
}

SHELF_DETECTION_RANGE = 6.0   # metres — proximity trigger for shelf detections
SCAN_MAX_RANGE       = 10.0   # only cluster returns closer than this
CLUSTER_GAP          = 1.0    # new cluster if consecutive points are > this apart (5× world)
CLUSTER_MIN_PTS      = 2      # minimum points to consider a valid cluster
SHELF_EXCLUSION_DIST = 3.0    # suppress LiDAR clusters that are near known shelves


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

        # Odom frame origin = spawn position; initialise to spawn so world coords
        # are correct even before the first odom message arrives.
        spawn = ROBOT_SPAWN_POSITIONS.get(robot_id, (0.0, 0.0))
        self._spawn_x: float = spawn[0]
        self._spawn_y: float = spawn[1]
        self._robot_x: float = spawn[0]
        self._robot_y: float = spawn[1]
        self._robot_theta: float = 0.0

        self._robot_state: str = "IDLE"
        self._last_scan: Optional[LaserScan] = None

        self._odom_sub = self.create_subscription(
            Odometry, f"{ns}/odom", self._odom_callback, 10,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, f"{ns}/scan", self._scan_callback, 10,
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

        self.get_logger().info(f"InspectionDetector started for {robot_id}")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        # Odom frame origin = spawn position → add spawn offset to get world coords
        self._robot_x = msg.pose.pose.position.x + self._spawn_x
        self._robot_y = msg.pose.pose.position.y + self._spawn_y
        q = msg.pose.pose.orientation
        self._robot_theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _state_callback(self, msg: RobotState) -> None:
        self._robot_state = msg.state

    # ── Main detection loop ──────────────────────────────────────────────────

    def _detection_loop(self) -> None:
        if self._robot_state == "EMERGENCY_STOP":
            return

        detections: List[Detection] = []
        detections.extend(self._detect_shelves())
        detections.extend(self._detect_from_scan())

        for det in detections:
            self._detection_pub.publish(det)
            self._all_detections_pub.publish(det)
            self.get_logger().info(
                f"[{self._robot_id}] DETECTED {det.detection_type} '{det.object_id}' "
                f"at ({det.pose.position.x:.1f}, {det.pose.position.y:.1f}) "
                f"dist={det.distance:.1f}m conf={det.confidence:.2f}"
            )

    # ── Source 1: proximity-based shelf detection ────────────────────────────

    def _detect_shelves(self) -> List[Detection]:
        results: List[Detection] = []

        for shelf_name, (sx, sy) in SHELF_POSITIONS.items():
            dist = math.hypot(sx - self._robot_x, sy - self._robot_y)
            if dist > SHELF_DETECTION_RANGE:
                continue

            results.append(self._make_detection(
                "SHELF", shelf_name, sx, sy,
                min(0.99, 0.85 + random.uniform(0, 0.10)),
            ))

            if random.random() < 0.7:
                for i in range(random.randint(1, 4)):
                    jx = sx + random.uniform(-1.5, 1.5)
                    jy = sy + random.uniform(-0.3, 0.3)
                    results.append(self._make_detection(
                        "INVENTORY", f"item_{shelf_name}_{i}",
                        jx, jy, random.uniform(0.75, 0.95),
                    ))

        return results

    # ── Source 2: LiDAR scan clustering ─────────────────────────────────────

    def _detect_from_scan(self) -> List[Detection]:
        if self._last_scan is None:
            return []

        scan = self._last_scan
        ranges = scan.ranges
        if not ranges:
            return []

        max_r = scan.range_max or 15.0
        min_r = scan.range_min or 0.10

        # Convert valid scan points to world-frame Cartesian
        world_pts: List[Tuple[float, float, float]] = []  # (wx, wy, range)
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < min_r or r > SCAN_MAX_RANGE:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            ct, st = math.cos(self._robot_theta), math.sin(self._robot_theta)
            wx = self._robot_x + lx * ct - ly * st
            wy = self._robot_y + lx * st + ly * ct
            world_pts.append((wx, wy, r))

        if not world_pts:
            return []

        # Group into clusters by consecutive-point distance
        clusters: List[List[Tuple[float, float, float]]] = []
        cur = [world_pts[0]]
        for pt in world_pts[1:]:
            gap = math.hypot(pt[0] - cur[-1][0], pt[1] - cur[-1][1])
            if gap < CLUSTER_GAP:
                cur.append(pt)
            else:
                if len(cur) >= CLUSTER_MIN_PTS:
                    clusters.append(cur)
                cur = [pt]
        if len(cur) >= CLUSTER_MIN_PTS:
            clusters.append(cur)

        detections: List[Detection] = []
        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            # Skip clusters that are part of the warehouse shelves
            near_shelf = any(
                math.hypot(cx - sx, cy - sy) < SHELF_EXCLUSION_DIST
                for sx, sy in SHELF_POSITIONS.values()
            )
            if near_shelf:
                continue

            # Estimate bounding width of cluster (max pairwise distance, cheap approx)
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))

            if width < 1.2:
                det_type = "PACKAGE"
                conf = random.uniform(0.80, 0.95)
            elif width < 2.5:
                det_type = "OBSTACLE"
                conf = random.uniform(0.75, 0.92)
            else:
                det_type = "UNKNOWN_OBJECT"
                conf = random.uniform(0.65, 0.85)

            # Stable object ID derived from grid position to avoid duplicate spam
            gx = int(round(cx * 2))
            gy = int(round(cy * 2))
            obj_id = f"{det_type.lower()}_{gx}_{gy}"

            detections.append(self._make_detection(det_type, obj_id, cx, cy, conf))

        return detections

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _make_detection(
        self,
        det_type: str,
        obj_id: str,
        x: float,
        y: float,
        confidence: float,
    ) -> Detection:
        det = Detection()
        det.detection_id   = str(uuid.uuid4())
        det.robot_id       = self._robot_id
        det.detection_type = det_type
        det.object_id      = obj_id
        det.object_class   = det_type
        det.confidence     = confidence
        det.distance       = math.hypot(x - self._robot_x, y - self._robot_y)
        det.is_obstacle    = det_type == "OBSTACLE"
        det.is_target      = det_type == "TARGET"
        det.is_human       = det_type == "HUMAN"

        pose = Pose()
        pose.position    = Point(x=x, y=y, z=0.0)
        pose.orientation = yaw_to_quaternion(0.0)
        det.pose      = pose
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
