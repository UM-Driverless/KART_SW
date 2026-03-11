import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory('kart_bringup'),
        'config',
        'teleop_params.yaml'
    )

    # Reads /joy and produces Twist on /kart/cmd_vel
    joy_node_cmd = Node(
        package='joy_to_cmd_vel',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen',
        parameters=[joy_params]
    )

    # Reads the physical gamepad → /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # Converts /kart/cmd_vel (Twist) → /orin/* protobuf Frame messages
    cmd_vel_bridge = Node(
        package='kart_bringup',
        executable='cmd_vel_bridge_node.py',
        name='cmd_vel_bridge',
        output='screen',
    )

    # Serial bridge: /esp32/tx (Frame) → ESP32 UART
    comms_node = Node(
        package='kb_coms_micro',
        executable='KB_Coms_micro',
        name='kb_coms_micro',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(joy_node_cmd)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(comms_node)

    return ld
