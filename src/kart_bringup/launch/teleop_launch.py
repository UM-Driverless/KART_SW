import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory('kart_bringup'),
        'config',
        'teleop_params.yaml'
    )

    # Nodo que lee /joy y crea msg de movimiento
    joy_node_cmd = Node(
        package='joy_to_cmd_vel',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen',
        parameters=[joy_params]
    )
    
    # Nodo que lee el mando, y deja sus valores en ROS2
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

    # Nodo que comunica Orin con microcontrolador
    comms_node_cmd = Node(
        package='msgs_to_micro',
        executable='comms_micro',
        name='comms_micro',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(joy_node_cmd)
    ld.add_action(joy_node)
    ld.add_action(comms_node_cmd)

    return ld
