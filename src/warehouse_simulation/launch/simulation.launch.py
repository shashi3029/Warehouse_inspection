#!/usr/bin/env python3
"""
Simulation launch file — spawns the warehouse Gazebo world and four robots.
Uses the warehouse_description xacro URDF (robot_name arg sets namespace prefix).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


ROBOT_CONFIGURATIONS = [
    {"name": "robot1", "robot_id": "Robot_1", "x": -8.0, "y": -8.0, "yaw": 0.0},
    {"name": "robot2", "robot_id": "Robot_2", "x": -8.0, "y":  8.0, "yaw": 0.0},
    {"name": "robot3", "robot_id": "Robot_3", "x":  8.0, "y": -8.0, "yaw": 0.0},
    {"name": "robot4", "robot_id": "Robot_4", "x":  8.0, "y":  8.0, "yaw": 0.0},
]


def generate_launch_description():
    warehouse_sim_dir  = get_package_share_directory("warehouse_simulation")
    warehouse_desc_dir = get_package_share_directory("warehouse_description")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_file   = LaunchConfiguration(
        "world_file",
        default=os.path.join(warehouse_sim_dir, "worlds", "warehouse.world"),
    )

    xacro_file = os.path.join(warehouse_desc_dir, "urdf", "warehouse_robot.urdf.xacro")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world":   world_file,
            "verbose": "false",
            "pause":   "false",
        }.items(),
    )

    robot_groups = []
    for i, cfg in enumerate(ROBOT_CONFIGURATIONS):
        ns       = cfg["name"]
        robot_id = cfg["robot_id"]
        x        = cfg["x"]
        y        = cfg["y"]
        yaw      = cfg["yaw"]

        robot_description = Command(
            ["xacro ", xacro_file,
             " robot_name:=", ns,
             " robot_id:=",   robot_id]
        )

        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time":       use_sim_time,
                "robot_description":  robot_description,
                "frame_prefix":       ns + "/",
            }],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        spawn = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic",            f"/{ns}/robot_description",
                "-entity",           ns,
                "-robot_namespace",  ns,
                "-x",  str(x),
                "-y",  str(y),
                "-z",  "0.01",
                "-Y",  str(yaw),
            ],
            output="screen",
        )

        robot_groups.append(
            TimerAction(
                period=float(i) * 3.0,
                actions=[
                    GroupAction([
                        PushRosNamespace(ns),
                        rsp,
                        spawn,
                    ])
                ],
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=os.path.join(warehouse_sim_dir, "worlds", "warehouse.world"),
            description="Absolute path to the Gazebo .world file",
        ),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        gazebo,
        *robot_groups,
    ])
