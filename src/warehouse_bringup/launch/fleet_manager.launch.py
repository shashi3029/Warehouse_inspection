"""
Launch the Warehouse Fleet Management System (no simulation, no Nav2).
Use this when Gazebo + Nav2 are already running, or for testing the fleet
manager in isolation against a real or externally-simulated robot fleet.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


_ROBOT_IDS = ["Robot_1", "Robot_2", "Robot_3", "Robot_4"]
_ROBOT_NS  = ["robot1",  "robot2",  "robot3",  "robot4"]

_PKG_FM   = "warehouse_fleet_manager"
_PKG_NAV  = "warehouse_navigation"
_PKG_INS  = "warehouse_inspection"
_PKG_BU   = "warehouse_bringup"


def generate_launch_description():
    pkg_bringup = get_package_share_directory(_PKG_BU)

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ------------------------------------------------------------------ #
    # Core fleet management nodes (single instances, not per-robot)       #
    # ------------------------------------------------------------------ #
    mission_planner = Node(
        package=_PKG_FM,
        executable="mission_planner",
        name="mission_planner",
        namespace="warehouse",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    mission_validator = Node(
        package=_PKG_FM,
        executable="mission_validator",
        name="mission_validator",
        namespace="warehouse",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    mission_coordinator = Node(
        package=_PKG_FM,
        executable="mission_coordinator",
        name="mission_coordinator",
        namespace="warehouse",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    fleet_dashboard = Node(
        package=_PKG_FM,
        executable="fleet_dashboard",
        name="fleet_dashboard",
        namespace="warehouse",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    charging_manager = Node(
        package=_PKG_FM,
        executable="charging_manager",
        name="charging_manager",
        namespace="warehouse",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    # Formation controller (C++ node — one instance for all robots)
    formation_controller = Node(
        package=_PKG_NAV,
        executable="formation_controller",
        name="formation_controller",
        namespace="warehouse",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Per-robot nodes                                                      #
    # ------------------------------------------------------------------ #
    per_robot_nodes = []
    for robot_id, ns in zip(_ROBOT_IDS, _ROBOT_NS):
        battery_manager = Node(
            package=_PKG_FM,
            executable="battery_manager",
            name=f"battery_manager_{ns}",
            namespace="warehouse",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_id": robot_id,
            }],
            arguments=[robot_id],
            output="screen",
            emulate_tty=True,
        )

        mission_executor = Node(
            package=_PKG_FM,
            executable="mission_executor",
            name=f"mission_executor_{ns}",
            namespace="warehouse",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_id": robot_id,
            }],
            arguments=[robot_id],
            output="screen",
            emulate_tty=True,
        )

        inspection_detector = Node(
            package=_PKG_INS,
            executable="inspection_detector",
            name=f"inspection_detector_{ns}",
            namespace="warehouse",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_id": robot_id,
            }],
            arguments=[robot_id],
            output="screen",
            emulate_tty=True,
        )

        inspection_tracker = Node(
            package=_PKG_INS,
            executable="inspection_tracker",
            name=f"inspection_tracker_{ns}",
            namespace="warehouse",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_id": robot_id,
            }],
            arguments=[robot_id],
            output="screen",
            emulate_tty=True,
        )

        per_robot_nodes.extend([
            battery_manager,
            mission_executor,
            inspection_detector,
            inspection_tracker,
        ])

    # ------------------------------------------------------------------ #
    # Startup sequence: core nodes first, per-robot nodes after 3 s       #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),

        LogInfo(msg="[FleetManager] Starting core fleet management nodes..."),

        mission_planner,
        mission_validator,
        charging_manager,
        formation_controller,

        # Coordinator starts after planner/validator are ready
        TimerAction(period=2.0, actions=[
            LogInfo(msg="[FleetManager] Starting mission coordinator..."),
            mission_coordinator,
        ]),

        # Per-robot nodes after coordinator
        TimerAction(period=4.0, actions=[
            LogInfo(msg="[FleetManager] Starting per-robot nodes..."),
            *per_robot_nodes,
        ]),

        # Dashboard last so it has data to display
        TimerAction(period=6.0, actions=[
            LogInfo(msg="[FleetManager] Starting fleet dashboard..."),
            fleet_dashboard,
        ]),
    ])
