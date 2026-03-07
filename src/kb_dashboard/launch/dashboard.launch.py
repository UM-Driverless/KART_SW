from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kb_dashboard",
            executable="dashboard",
            name="kb_dashboard",
            parameters=[{"port": 8080}],
            output="screen",
        ),
    ])
