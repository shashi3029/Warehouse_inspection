#!/usr/bin/env python3
"""
Simulation launch file — spawns the warehouse Gazebo world and four TurtleBot3 robots.
Each robot gets its own namespace, TF prefix, and robot_state_publisher.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


ROBOT_CONFIGURATIONS = [
    {"name": "robot1", "robot_id": "Robot_1", "x":  -8.0, "y":  -8.0, "yaw": 0.0},
    {"name": "robot2", "robot_id": "Robot_2", "x":  -8.0, "y":   8.0, "yaw": 0.0},
    {"name": "robot3", "robot_id": "Robot_3", "x":   8.0, "y":  -8.0, "yaw": 0.0},
    {"name": "robot4", "robot_id": "Robot_4", "x":   8.0, "y":   8.0, "yaw": 0.0},
]


def generate_launch_description():
    warehouse_sim_dir = get_package_share_directory("warehouse_simulation")
    turtlebot3_description_dir = get_package_share_directory("turtlebot3_description")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    turtlebot3_model = LaunchConfiguration("turtlebot3_model", default="burger")

    world_file = os.path.join(warehouse_sim_dir, "worlds", "warehouse.world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    declare_model = DeclareLaunchArgument(
        "turtlebot3_model",
        default_value="burger",
        description="TurtleBot3 model type (burger/waffle/waffle_pi)",
    )

    set_tb3_model = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL", value=turtlebot3_model
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
            "pause": "false",
        }.items(),
    )

    robot_nodes = []
    for i, cfg in enumerate(ROBOT_CONFIGURATIONS):
        ns = cfg["name"]
        robot_id = cfg["robot_id"]
        x = cfg["x"]
        y = cfg["y"]
        yaw = cfg["yaw"]

        urdf_file = os.path.join(
            turtlebot3_description_dir, "urdf", "turtlebot3_burger.urdf"
        )

        robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time, "robot_description": open(urdf_file).read()},
                {"frame_prefix": ns + "/"},
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        spawn_robot = TimerAction(
            period=float(i) * 3.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-topic", f"/{ns}/robot_description",
                        "-entity", ns,
                        "-robot_namespace", ns,
                        "-x", str(x),
                        "-y", str(y),
                        "-z", "0.01",
                        "-Y", str(yaw),
                    ],
                    output="screen",
                )
            ],
        )

        robot_nodes.extend([robot_state_pub, spawn_robot])

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_model,
            set_tb3_model,
            gazebo,
        ]
        + robot_nodes
    )
