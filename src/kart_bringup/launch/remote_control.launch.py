# Remote control via the web dashboard.
# Launches comms, state machine, cmd_vel bridge, and dashboard.
# Drive from the dashboard's touch pad, with the mission set to remote_control.
# No perception nodes needed. Gamepad support was removed on 2026-08-10 — the touch pad is
# the only remote control, on the ROS side as well as in the browser.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    comms = Node(
        package="kb_coms_micro",
        executable="KB_Coms_micro",
        name="kb_coms_micro",
        output="screen",
    )

    state_machine = Node(
        package="kart_control",
        executable="state_machine_node.py",
        name="state_machine",
        output="screen",
    )

    cmd_vel_bridge = Node(
        package="kart_control",
        executable="cmd_vel_bridge_node.py",
        name="cmd_vel_bridge",
        output="screen",
    )

    dashboard = Node(
        package="kb_dashboard",
        executable="dashboard",
        name="kb_dashboard",
        parameters=[{"port": 80}],
        output="screen",
    )

    return LaunchDescription([
        comms,
        state_machine,
        cmd_vel_bridge,
        dashboard,
    ])
