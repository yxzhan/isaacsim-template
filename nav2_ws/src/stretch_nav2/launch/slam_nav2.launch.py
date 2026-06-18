"""Bring up SLAM (slam_toolbox) + Nav2 for the Stretch Isaac Sim apartment demo.

slam_toolbox publishes map->odom from the RTX lidar /scan; the Isaac Sim node
publishes odom->base_link, /odom and /scan. Nav2's controller publishes /cmd_vel,
which the Stretch sim node subscribes to (alongside /stretch/cmd_vel for manual
steering).

The sim node stamps everything with the wall clock and does not publish /clock,
so use_sim_time is False throughout.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("stretch_nav2")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    slam_params = os.path.join(pkg_share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "nav2.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use /clock time (the sim stamps with wall clock, so keep false).",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Start RViz with the nav2 config (the notebook starts RViz separately).",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        slam_toolbox,
        nav2,
        rviz,
    ])
