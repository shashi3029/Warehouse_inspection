#!/usr/bin/env python3

from enum import Enum
from typing import Optional, Dict, Any, Callable, List
import time
import threading


class RobotState(Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    PATROLLING = "PATROLLING"
    NAVIGATING = "NAVIGATING"
    INSPECTING = "INSPECTING"
    FORMATION = "FORMATION"
    TRACKING = "TRACKING"
    WAITING = "WAITING"
    RETURNING_TO_CHARGER = "RETURNING_TO_CHARGER"
    CHARGING = "CHARGING"
    RESUMING = "RESUMING"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    ERROR = "ERROR"


VALID_TRANSITIONS: Dict[RobotState, List[RobotState]] = {
    RobotState.IDLE: [
        RobotState.MOVING,
        RobotState.PATROLLING,
        RobotState.NAVIGATING,
        RobotState.INSPECTING,
        RobotState.FORMATION,
        RobotState.TRACKING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.CHARGING,
        RobotState.WAITING,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.MOVING: [
        RobotState.IDLE,
        RobotState.PATROLLING,
        RobotState.NAVIGATING,
        RobotState.INSPECTING,
        RobotState.WAITING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.PATROLLING: [
        RobotState.IDLE,
        RobotState.INSPECTING,
        RobotState.TRACKING,
        RobotState.WAITING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.NAVIGATING: [
        RobotState.IDLE,
        RobotState.INSPECTING,
        RobotState.PATROLLING,
        RobotState.FORMATION,
        RobotState.TRACKING,
        RobotState.WAITING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.CHARGING,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.INSPECTING: [
        RobotState.IDLE,
        RobotState.NAVIGATING,
        RobotState.PATROLLING,
        RobotState.WAITING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.FORMATION: [
        RobotState.IDLE,
        RobotState.NAVIGATING,
        RobotState.WAITING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.TRACKING: [
        RobotState.IDLE,
        RobotState.PATROLLING,
        RobotState.NAVIGATING,
        RobotState.WAITING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.WAITING: [
        RobotState.IDLE,
        RobotState.MOVING,
        RobotState.NAVIGATING,
        RobotState.PATROLLING,
        RobotState.INSPECTING,
        RobotState.FORMATION,
        RobotState.TRACKING,
        RobotState.RESUMING,
        RobotState.RETURNING_TO_CHARGER,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.RETURNING_TO_CHARGER: [
        RobotState.CHARGING,
        RobotState.IDLE,
        RobotState.WAITING,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.CHARGING: [
        RobotState.IDLE,
        RobotState.RESUMING,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.RESUMING: [
        RobotState.IDLE,
        RobotState.NAVIGATING,
        RobotState.PATROLLING,
        RobotState.INSPECTING,
        RobotState.FORMATION,
        RobotState.TRACKING,
        RobotState.EMERGENCY_STOP,
        RobotState.ERROR,
    ],
    RobotState.EMERGENCY_STOP: [
        RobotState.IDLE,
        RobotState.ERROR,
    ],
    RobotState.ERROR: [
        RobotState.IDLE,
        RobotState.EMERGENCY_STOP,
    ],
}

WORKING_STATES = {
    RobotState.MOVING,
    RobotState.PATROLLING,
    RobotState.NAVIGATING,
    RobotState.INSPECTING,
    RobotState.FORMATION,
    RobotState.TRACKING,
    RobotState.RESUMING,
}

AVAILABLE_STATES = {
    RobotState.IDLE,
    RobotState.WAITING,
}

CHARGING_STATES = {
    RobotState.RETURNING_TO_CHARGER,
    RobotState.CHARGING,
}

BATTERY_DRAINING_STATES = {
    RobotState.MOVING,
    RobotState.PATROLLING,
    RobotState.NAVIGATING,
    RobotState.INSPECTING,
    RobotState.FORMATION,
    RobotState.TRACKING,
    RobotState.RESUMING,
    RobotState.RETURNING_TO_CHARGER,
}


class RobotStateMachine:
    def __init__(self, robot_id: str, logger=None):
        self.robot_id = robot_id
        self.logger = logger
        self._state = RobotState.IDLE
        self._previous_state = RobotState.IDLE
        self._state_entry_time = time.monotonic()
        self._lock = threading.RLock()
        self._transition_callbacks: Dict[tuple, Callable] = {}
        self._state_entry_callbacks: Dict[RobotState, List[Callable]] = {}
        self._state_exit_callbacks: Dict[RobotState, List[Callable]] = {}
        self._transition_history: List[Dict[str, Any]] = []

    @property
    def state(self) -> RobotState:
        with self._lock:
            return self._state

    @property
    def previous_state(self) -> RobotState:
        with self._lock:
            return self._previous_state

    @property
    def state_name(self) -> str:
        with self._lock:
            return self._state.value

    @property
    def state_duration_seconds(self) -> float:
        return time.monotonic() - self._state_entry_time

    def can_transition(self, new_state: RobotState) -> bool:
        with self._lock:
            return new_state in VALID_TRANSITIONS.get(self._state, [])

    def transition(self, new_state: RobotState, reason: str = "", force: bool = False) -> bool:
        with self._lock:
            if not force and new_state not in VALID_TRANSITIONS.get(self._state, []):
                if self.logger:
                    self.logger.warning(
                        f"[{self.robot_id}] Invalid transition: "
                        f"{self._state.value} -> {new_state.value}"
                    )
                return False

            old_state = self._state
            self._transition_history.append({
                "from": old_state.value,
                "to": new_state.value,
                "reason": reason,
                "timestamp": time.monotonic(),
                "duration_in_previous": time.monotonic() - self._state_entry_time,
            })
            if len(self._transition_history) > 100:
                self._transition_history.pop(0)

            exit_cbs = self._state_exit_callbacks.get(old_state, [])
            self._previous_state = old_state
            self._state = new_state
            self._state_entry_time = time.monotonic()
            entry_cbs = self._state_entry_callbacks.get(new_state, [])
            pair_cb = self._transition_callbacks.get((old_state, new_state))

        if self.logger:
            self.logger.info(
                f"[{self.robot_id}] State: {old_state.value} -> {new_state.value}"
                + (f" ({reason})" if reason else "")
            )

        for cb in exit_cbs:
            try:
                cb(old_state, new_state)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[{self.robot_id}] Exit callback error: {e}")

        for cb in entry_cbs:
            try:
                cb(old_state, new_state)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[{self.robot_id}] Entry callback error: {e}")

        if pair_cb:
            try:
                pair_cb()
            except Exception as e:
                if self.logger:
                    self.logger.error(f"[{self.robot_id}] Transition callback error: {e}")

        return True

    def register_transition_callback(
        self, from_state: RobotState, to_state: RobotState, callback: Callable
    ) -> None:
        self._transition_callbacks[(from_state, to_state)] = callback

    def register_state_entry_callback(
        self, state: RobotState, callback: Callable
    ) -> None:
        if state not in self._state_entry_callbacks:
            self._state_entry_callbacks[state] = []
        self._state_entry_callbacks[state].append(callback)

    def register_state_exit_callback(
        self, state: RobotState, callback: Callable
    ) -> None:
        if state not in self._state_exit_callbacks:
            self._state_exit_callbacks[state] = []
        self._state_exit_callbacks[state].append(callback)

    def is_available(self) -> bool:
        with self._lock:
            return self._state in AVAILABLE_STATES

    def is_working(self) -> bool:
        with self._lock:
            return self._state in WORKING_STATES

    def is_charging(self) -> bool:
        with self._lock:
            return self._state in CHARGING_STATES

    def is_draining_battery(self) -> bool:
        with self._lock:
            return self._state in BATTERY_DRAINING_STATES

    def is_error(self) -> bool:
        with self._lock:
            return self._state in {RobotState.ERROR, RobotState.EMERGENCY_STOP}

    def get_transition_history(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._transition_history)

    def to_dict(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "robot_id": self.robot_id,
                "state": self._state.value,
                "previous_state": self._previous_state.value,
                "state_duration_seconds": time.monotonic() - self._state_entry_time,
                "is_available": self._state in AVAILABLE_STATES,
                "is_working": self._state in WORKING_STATES,
                "is_charging": self._state in CHARGING_STATES,
                "is_error": self._state in {RobotState.ERROR, RobotState.EMERGENCY_STOP},
            }
