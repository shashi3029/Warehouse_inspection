"""
Master launch file — brings up the complete Warehouse Fleet Management System:
  1. Gazebo simulation (warehouse world + 4 robots)
  2. Robot state publishers (one per robot)
  3. Nav2 navigation stack (one full stack per robot)
  4. Warehouse Fleet Manager (planner → validator → coordinator → executors …)
  5. RViz2 (optional, enable with rviz:=true)

Typical usage:
  ros2 launch warehouse_bringup full_system.launch.py
  ros2 launch warehouse_bringup full_system.launch.py rviz:=true
  ros2 launch warehouse_bringup full_system.launch.py use_slam:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PKG_BU  = "warehouse_bringup"
_PKG_SIM = "warehouse_simulation"
_PKG_DESC = "warehouse_description"


def _include(pkg: str, launch_file: str, **kwargs):
    pkg_dir = get_package_share_directory(pkg)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", launch_file)
        ),
        launch_arguments=kwargs.items(),
    )


def generate_launch_description():
    pkg_bringup = get_package_share_directory(_PKG_BU)
    pkg_desc    = get_package_share_directory(_PKG_DESC)

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_slam     = LaunchConfiguration("use_slam",     default="false")
    use_nav2     = LaunchConfiguration("use_nav2",     default="false")
    use_rviz     = LaunchConfiguration("rviz",         default="false")
    map_yaml     = LaunchConfiguration("map_yaml",     default="")
    world_file   = LaunchConfiguration(
        "world_file",
        default=os.path.join(
            get_package_share_directory(_PKG_SIM), "worlds", "warehouse.world"
        ),
    )
    rviz_config  = os.path.join(pkg_desc, "rviz", "fleet_view.rviz")

    # ------------------------------------------------------------------ #
    # 1. Gazebo + robot spawning                                           #
    # ------------------------------------------------------------------ #
    simulation = TimerAction(
        period=0.0,
        actions=[
            LogInfo(msg="[FullSystem] Step 1: Starting Gazebo simulation..."),
            _include(_PKG_SIM, "simulation.launch.py",
                     world_file=world_file,
                     use_sim_time=use_sim_time),
        ],
    )

    # ------------------------------------------------------------------ #
    # 2. Nav2 for all robots (optional — skip when no map/SLAM ready)     #
    # ------------------------------------------------------------------ #
    navigation = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[FullSystem] Step 2: Starting Nav2 navigation stack..."),
            _include(_PKG_BU, "multi_robot.launch.py",
                     use_sim_time=use_sim_time,
                     use_slam=use_slam,
                     map_yaml=map_yaml),
        ],
        condition=IfCondition(use_nav2),
    )

    # ------------------------------------------------------------------ #
    # 3. Fleet management stack                                            #
    # ------------------------------------------------------------------ #
    fleet_manager = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="[FullSystem] Step 3: Starting fleet management stack..."),
            _include(_PKG_BU, "fleet_manager.launch.py",
                     use_sim_time=use_sim_time),
        ],
    )

    # ------------------------------------------------------------------ #
    # 4. RViz2 (optional)                                                  #
    # ------------------------------------------------------------------ #
    rviz = TimerAction(
        period=22.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ],
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use Gazebo simulation clock"),
        DeclareLaunchArgument("use_nav2",     default_value="false",
                              description="Launch Nav2 stack (requires map or SLAM ready)"),
        DeclareLaunchArgument("use_slam",     default_value="false",
                              description="Map with SLAM Toolbox; skip AMCL"),
        DeclareLaunchArgument("map_yaml",     default_value="",
                              description="Path to pre-built map YAML for AMCL"),
        DeclareLaunchArgument("rviz",         default_value="false",
                              description="Launch RViz2 fleet monitor"),
        DeclareLaunchArgument(
            "world_file",
            default_value=os.path.join(
                get_package_share_directory(_PKG_SIM), "worlds", "warehouse.world"
            ),
            description="Absolute path to the Gazebo .world file",
        ),

        LogInfo(msg="========================================================"),
        LogInfo(msg=" Warehouse Fleet Management System — Full System Launch  "),
        LogInfo(msg="========================================================"),
        LogInfo(msg=" Robots  : Robot_1, Robot_2, Robot_3, Robot_4           "),
        LogInfo(msg=" Sim     : Gazebo Classic + warehouse.world              "),
        LogInfo(msg=" Nav     : Nav2 (DWB + NavFn + ObstacleLayer costmaps)  "),
        LogInfo(msg=" Fleet   : Mission Planner → Coordinator → Executors    "),
        LogInfo(msg="========================================================"),

        simulation,
        navigation,
        fleet_manager,
        rviz,
    ])
