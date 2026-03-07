from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kart_perception"),
                "launch",
                "perception_3d.launch.py",
            )
        )
    )

    cone_follower = Node(
        package="kart_sim",
        executable="cone_follower_node.py",
        name="cone_follower",
        output="screen",
        parameters=[{"controller_type": "geometric"}],
    )

    cmd_vel_bridge = Node(
        package="kart_bringup",
        executable="cmd_vel_bridge_node.py",
        name="cmd_vel_bridge",
        output="screen",
    )

    comms = Node(
        package="kb_coms_micro",
        executable="KB_Coms_micro",
        name="kb_coms_micro",
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
        perception_launch,
        cone_follower,
        cmd_vel_bridge,
        comms,
        dashboard,
    ])
