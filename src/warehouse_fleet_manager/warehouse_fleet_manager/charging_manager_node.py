#!/usr/bin/env python3
"""
Charging Manager Node

Manages all charging stations in the warehouse. Assigns robots to the
nearest available charging station, queues robots when stations are at
capacity, prevents multiple robots occupying the same dock, and
publishes charging status for the fleet dashboard.
"""

import json
import math
import time
from typing import Dict, List, Optional, Tuple
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from warehouse_msgs.msg import ChargingStatus, BatteryStatus, RobotState
from warehouse_msgs.srv import GetNearestCharger


CHARGING_STATIONS: Dict[str, Dict] = {
    "Charging_Station_1": {
        "position": (-15.0, -15.0),
        "capacity": 2,
        "dock_positions": [(-14.5, -15.0), (-15.5, -15.0)],
    },
    "Charging_Station_2": {
        "position": (15.0, -15.0),
        "capacity": 2,
        "dock_positions": [(14.5, -15.0), (15.5, -15.0)],
    },
}

ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]


class ChargingStation:
    def __init__(self, station_id: str, config: Dict):
        self.station_id = station_id
        self.position: Tuple[float, float] = tuple(config["position"])
        self.capacity: int = config["capacity"]
        self.dock_positions: List[Tuple[float, float]] = [
            tuple(d) for d in config.get("dock_positions", [config["position"]])
        ]
        self.occupants: List[str] = []
        self.queue: deque = deque()
        self.charge_start_times: Dict[str, float] = {}

    @property
    def is_available(self) -> bool:
        return len(self.occupants) < self.capacity

    @property
    def available_slots(self) -> int:
        return self.capacity - len(self.occupants)

    def assign(self, robot_id: str) -> bool:
        if robot_id in self.occupants:
            return True
        if self.is_available:
            self.occupants.append(robot_id)
            self.charge_start_times[robot_id] = time.monotonic()
            return True
        if robot_id not in self.queue:
            self.queue.append(robot_id)
        return False

    def release(self, robot_id: str) -> Optional[str]:
        if robot_id in self.occupants:
            self.occupants.remove(robot_id)
            self.charge_start_times.pop(robot_id, None)
        elif robot_id in self.queue:
            queue_list = list(self.queue)
            queue_list.remove(robot_id)
            self.queue = deque(queue_list)

        if self.queue and self.is_available:
            next_robot = self.queue.popleft()
            self.occupants.append(next_robot)
            self.charge_start_times[next_robot] = time.monotonic()
            return next_robot
        return None

    def get_dock_position(self, robot_id: str) -> Tuple[float, float]:
        if robot_id in self.occupants:
            idx = self.occupants.index(robot_id)
            if idx < len(self.dock_positions):
                return self.dock_positions[idx]
        return self.position

    def queue_position(self, robot_id: str) -> int:
        if robot_id in self.queue:
            return list(self.queue).index(robot_id) + 1
        return 0


class ChargingManagerNode(Node):

    def __init__(self):
        super().__init__("charging_manager_node")

        self._stations: Dict[str, ChargingStation] = {
            sid: ChargingStation(sid, cfg)
            for sid, cfg in CHARGING_STATIONS.items()
        }

        self._robot_positions: Dict[str, Tuple[float, float]] = {
            rid: (0.0, 0.0) for rid in ROBOT_IDS
        }
        self._robot_batteries: Dict[str, float] = {
            rid: 100.0 for rid in ROBOT_IDS
        }
        self._robot_charging_stations: Dict[str, str] = {}

        cb_group = ReentrantCallbackGroup()

        for rid in ROBOT_IDS:
            self.create_subscription(
                RobotState,
                f"/warehouse/{rid}/robot_state",
                lambda msg, robot_id=rid: self._robot_state_callback(msg, robot_id),
                10,
                callback_group=cb_group,
            )
            self.create_subscription(
                BatteryStatus,
                f"/warehouse/{rid}/battery_status",
                lambda msg, robot_id=rid: self._battery_callback(msg, robot_id),
                10,
                callback_group=cb_group,
            )

        self._assign_request_sub = self.create_subscription(
            String,
            "/warehouse/charging/assign_request",
            self._assign_request_callback,
            10,
            callback_group=cb_group,
        )
        self._release_request_sub = self.create_subscription(
            String,
            "/warehouse/charging/release_request",
            self._release_request_callback,
            10,
            callback_group=cb_group,
        )

        self._charging_status_pub = self.create_publisher(
            ChargingStatus, "/warehouse/charging/status", 10
        )
        self._station_status_pub = self.create_publisher(
            String, "/warehouse/charging/station_status", 10
        )

        self._nearest_charger_srv = self.create_service(
            GetNearestCharger,
            "/warehouse/get_nearest_charger",
            self._get_nearest_charger_handler,
            callback_group=cb_group,
        )

        self._status_timer = self.create_timer(2.0, self._publish_station_status)

        self.get_logger().info(
            f"Charging Manager started with {len(self._stations)} stations: "
            f"{list(self._stations.keys())}"
        )

    def _robot_state_callback(self, msg: RobotState, robot_id: str) -> None:
        self._robot_positions[robot_id] = (msg.position.x, msg.position.y)

        if msg.state == "CHARGING" and robot_id not in self._robot_charging_stations:
            station_id = msg.goal or self._find_nearest_station(*self._robot_positions[robot_id])[0]
            self._do_assign(robot_id, station_id)

        elif msg.state not in ("CHARGING", "RETURNING_TO_CHARGER"):
            if robot_id in self._robot_charging_stations:
                self._do_release(robot_id)

    def _battery_callback(self, msg: BatteryStatus, robot_id: str) -> None:
        self._robot_batteries[robot_id] = msg.percentage

        if msg.percentage >= 99.5 and msg.is_charging:
            station = self._robot_charging_stations.get(robot_id)
            if station:
                self.get_logger().info(
                    f"{robot_id} fully charged at {station} — releasing dock"
                )
                self._do_release(robot_id)

    def _assign_request_callback(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = req.get("robot_id", "")
        requested_station = req.get("station_id", "")

        if requested_station and requested_station in self._stations:
            station_id = requested_station
        else:
            pos = self._robot_positions.get(robot_id, (0.0, 0.0))
            station_id, _, _ = self._find_nearest_station(*pos)

        success = self._do_assign(robot_id, station_id)
        self.get_logger().info(
            f"Assign request: {robot_id} -> {station_id}: {'OK' if success else 'QUEUED'}"
        )

    def _release_request_callback(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot_id = req.get("robot_id", "")
        if robot_id:
            self._do_release(robot_id)

    def _do_assign(self, robot_id: str, station_id: str) -> bool:
        if station_id not in self._stations:
            return False

        for sid, station in self._stations.items():
            if robot_id in station.occupants or robot_id in station.queue:
                if sid != station_id:
                    station.release(robot_id)

        station = self._stations[station_id]
        result = station.assign(robot_id)
        if result:
            self._robot_charging_stations[robot_id] = station_id
        return result

    def _do_release(self, robot_id: str) -> None:
        station_id = self._robot_charging_stations.pop(robot_id, None)
        if station_id and station_id in self._stations:
            next_robot = self._stations[station_id].release(robot_id)
            if next_robot:
                self._robot_charging_stations[next_robot] = station_id
                self.get_logger().info(
                    f"Dock released by {robot_id} at {station_id} — "
                    f"assigned to queued robot {next_robot}"
                )

    def _get_nearest_charger_handler(
        self,
        request: GetNearestCharger.Request,
        response: GetNearestCharger.Response,
    ) -> GetNearestCharger.Response:
        rx = request.robot_position.x
        ry = request.robot_position.y

        if request.require_available:
            avail_stations = {
                sid: s for sid, s in self._stations.items() if s.is_available
            }
        else:
            avail_stations = self._stations

        if not avail_stations:
            response.success = False
            response.message = "No available charging stations"
            return response

        station_id, sx, sy = self._find_nearest_station_from(rx, ry, avail_stations)
        dist = math.sqrt((sx - rx) ** 2 + (sy - ry) ** 2)
        station = self._stations[station_id]

        response.success = True
        response.station_id = station_id
        pos = Pose2D()
        pos.x = sx
        pos.y = sy
        response.station_position = pos
        response.distance = dist
        response.is_available = station.is_available
        response.queue_length = len(station.queue)
        response.message = (
            f"Nearest station: {station_id} ({dist:.1f}m away, "
            f"{station.available_slots} slots available)"
        )
        return response

    def _find_nearest_station(
        self, rx: float, ry: float
    ) -> Tuple[str, float, float]:
        return self._find_nearest_station_from(rx, ry, self._stations)

    def _find_nearest_station_from(
        self, rx: float, ry: float, stations: Dict
    ) -> Tuple[str, float, float]:
        best_id = ""
        best_dist = float("inf")
        best_pos = (0.0, 0.0)

        for sid, station in stations.items():
            sx, sy = station.position
            dist = math.sqrt((sx - rx) ** 2 + (sy - ry) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_id = sid
                best_pos = (sx, sy)

        return best_id, best_pos[0], best_pos[1]

    def _publish_station_status(self) -> None:
        station_data = {}
        for sid, station in self._stations.items():
            station_data[sid] = {
                "station_id": sid,
                "position": list(station.position),
                "capacity": station.capacity,
                "occupants": list(station.occupants),
                "queue": list(station.queue),
                "available_slots": station.available_slots,
                "is_available": station.is_available,
            }

        msg = String()
        msg.data = json.dumps(station_data)
        self._station_status_pub.publish(msg)

        for robot_id, station_id in self._robot_charging_stations.items():
            station = self._stations.get(station_id)
            if not station:
                continue

            cs = ChargingStatus()
            cs.robot_id = robot_id
            cs.station_id = station_id
            cs.is_charging = robot_id in station.occupants
            cs.is_queued = robot_id in station.queue
            cs.queue_position = station.queue_position(robot_id)
            cs.charge_percentage = self._robot_batteries.get(robot_id, 0.0)
            cs.station_available = station.is_available
            cs.station_capacity = station.capacity
            cs.station_current_occupancy = len(station.occupants)
            dock_x, dock_y = station.get_dock_position(robot_id)
            pos = Pose2D()
            pos.x = dock_x
            pos.y = dock_y
            cs.station_position = pos
            self._charging_status_pub.publish(cs)


def main(args=None):
    rclpy.init(args=args)
    node = ChargingManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
