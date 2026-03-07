"""Simulation mode: Gazebo + fake ESP32 telemetry + dashboard.

Usage:
    ros2 launch kart_bringup sim.launch.py
    ros2 launch kart_bringup sim.launch.py track:=hairpin gui:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_kart_sim = get_package_share_directory("kart_sim")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_kart_sim, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "track": LaunchConfiguration("track"),
            "use_yolo": LaunchConfiguration("use_yolo"),
            "gui": LaunchConfiguration("gui"),
            "controller": LaunchConfiguration("controller"),
            "weights_json": LaunchConfiguration("weights_json"),
        }.items(),
    )

    esp32_sim = Node(
        package="kart_sim",
        executable="esp32_sim_node.py",
        name="esp32_sim",
        output="screen",
    )

    dashboard = Node(
        package="kb_dashboard",
        executable="dashboard",
        name="kb_dashboard",
        parameters=[{"port": 8080}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("track", default_value="oval"),
        DeclareLaunchArgument("use_yolo", default_value="false"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("controller", default_value="neural_v2"),
        DeclareLaunchArgument(
            "weights_json",
            default_value=os.path.join(pkg_kart_sim, "config", "neural_v2_weights.json"),
        ),
        simulation,
        TimerAction(period=5.0, actions=[esp32_sim]),
        TimerAction(period=5.0, actions=[dashboard]),
    ])
