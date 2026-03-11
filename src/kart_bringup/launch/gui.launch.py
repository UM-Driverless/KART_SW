"""GUI visualization -- launch separately from autonomous.launch.py.

Provides the HUD viewer and optionally rviz2 for 3D marker visualization.
Kills stale rviz2/rqt_image_view processes before starting to prevent
GPU resource accumulation on the Orin.

Usage:
    ros2 launch kart_bringup gui.launch.py
    ros2 launch kart_bringup gui.launch.py use_rviz:=true camera_model:=zed2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    set_display = SetEnvironmentVariable("DISPLAY", os.environ.get("DISPLAY", ":1"))
    set_xauth = SetEnvironmentVariable(
        "XAUTHORITY",
        os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"),
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch rviz2 for 3D marker visualization.",
    )
    camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="zed2",
        description="Camera model (used for rviz2 config selection).",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    camera_model = LaunchConfiguration("camera_model")

    # Kill stale GUI processes to prevent accumulation
    cleanup_gui = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "killall -q rviz2 rqt_image_view 2>/dev/null; "
            "pkill -f 'rviz2|rqt_image_view' 2>/dev/null; "
            "sleep 0.5; exit 0",
        ],
        name="cleanup_stale_gui",
        output="log",
    )

    hud_viewer = Node(
        package="kart_perception",
        executable="hud_viewer",
        name="hud_viewer",
        output="screen",
    )

    # Optional rviz2 node -- replaces the need to manually run
    # display_zed_cam.launch.py alongside autonomous.launch.py
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            set_display,
            set_xauth,
            use_rviz_arg,
            camera_model_arg,
            cleanup_gui,
            hud_viewer,
            rviz2_node,
        ]
    )
