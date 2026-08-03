"""
Launch Nav2 for all 4 warehouse robots in parallel, each in its own namespace.
RewrittenYaml prepends the robot namespace to every top-level param key so that
/robot1/controller_server finds its params at 'robot1/controller_server'.
"""

import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


_ROBOTS = [
    {"id": "Robot_1", "ns": "robot1", "x": -8.0, "y": -8.0, "yaw": 0.0},
    {"id": "Robot_2", "ns": "robot2", "x": -8.0, "y":  8.0, "yaw": 0.0},
    {"id": "Robot_3", "ns": "robot3", "x":  8.0, "y": -8.0, "yaw": 0.0},
    {"id": "Robot_4", "ns": "robot4", "x":  8.0, "y":  8.0, "yaw": 0.0},
]

_PKG_BU = "warehouse_bringup"


def _make_nav2_group(robot: dict, nav2_params_file: str, map_yaml: str,
                     use_sim_time: LaunchConfiguration,
                     use_slam: LaunchConfiguration) -> GroupAction:
    ns   = robot["ns"]
    x, y = robot["x"], robot["y"]
    yaw  = robot["yaw"]

    # Rewrite the params file so each key is prefixed with the robot namespace.
    # E.g. "controller_server:" → "robot1/controller_server:"
    # This allows nodes running under /robot1 to resolve their parameters.
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=ns,
        param_rewrites={},
        convert_types=True,
    )

    remappings = [
        ("/tf",        "tf"),
        ("/tf_static", "tf_static"),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[configured_params, {
            "use_sim_time":  use_sim_time,
            "yaml_filename": map_yaml,
        }],
        remappings=remappings,
        condition=UnlessCondition(use_slam),
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[configured_params, {
            "use_sim_time":     use_sim_time,
            "initial_pose_x":   x,
            "initial_pose_y":   y,
            "initial_pose_a":   yaw,
        }],
        remappings=remappings,
        condition=UnlessCondition(use_slam),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory(_PKG_BU), "config", "slam_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=remappings,
        condition=IfCondition(use_slam),
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params, {
            "use_sim_time":      use_sim_time,
            "global_frame":      "map",
            "robot_base_frame":  f"{ns}/base_link",
            "odom_topic":        f"/{ns}/odom",
        }],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart":    True,
            "node_names":   ["map_server", "amcl"],
        }],
        condition=UnlessCondition(use_slam),
    )

    lifecycle_manager_localization_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart":    True,
            "node_names":   ["slam_toolbox"],
        }],
        condition=IfCondition(use_slam),
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart":    True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "smoother_server",
                "velocity_smoother",
                "collision_monitor",
                "bt_navigator",
                "waypoint_follower",
            ],
        }],
    )

    return GroupAction([
        PushRosNamespace(ns),
        map_server,
        amcl,
        slam_toolbox,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        velocity_smoother,
        collision_monitor,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_localization,
        lifecycle_manager_localization_slam,
        lifecycle_manager_navigation,
    ])


def generate_launch_description():
    pkg_bringup = get_package_share_directory(_PKG_BU)

    nav2_params_file = os.path.join(pkg_bringup, "config", "nav2_params.yaml")
    map_yaml         = LaunchConfiguration("map_yaml", default="")
    use_sim_time     = LaunchConfiguration("use_sim_time", default="true")
    use_slam         = LaunchConfiguration("use_slam",     default="false")

    actions: list = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation clock"),
        DeclareLaunchArgument("use_slam", default_value="false",
                              description="Use SLAM Toolbox instead of AMCL"),
        DeclareLaunchArgument("map_yaml", default_value="",
                              description="Absolute path to pre-built map YAML (AMCL mode)"),
    ]

    for i, robot in enumerate(_ROBOTS):
        delay = float(i) * 2.0
        group = _make_nav2_group(
            robot, nav2_params_file, map_yaml, use_sim_time, use_slam
        )
        actions.append(
            TimerAction(period=delay, actions=[
                LogInfo(msg=f"[MultiRobot] Launching Nav2 for {robot['ns']} (delay={delay}s)"),
                group,
            ])
        )

    return LaunchDescription(actions)
