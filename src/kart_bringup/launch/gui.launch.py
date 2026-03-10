"""GUI overlay viewer — launch separately from autonomous.launch.py.

Usage: ros2 launch kart_bringup gui.launch.py
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os


def generate_launch_description():
    set_display = SetEnvironmentVariable("DISPLAY", os.environ.get("DISPLAY", ":1"))
    set_xauth = SetEnvironmentVariable(
        "XAUTHORITY",
        os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"),
    )

    hud_viewer = Node(
        package="kart_perception",
        executable="hud_viewer",
        name="hud_viewer",
        output="screen",
    )

    return LaunchDescription([set_display, set_xauth, hud_viewer])
