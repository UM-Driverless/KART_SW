"""Perception pipeline using ZED SDK built-in object detection.

Nodes subscribe directly to ZED ObjectsStamped — no bridge needed.
Only cone_marker_viz_3d is launched here (yolo_detector and
cone_depth_localizer are not needed in this mode).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    marker_viz_3d = Node(
        package="kart_perception",
        executable="cone_marker_viz_3d",
        name="cone_marker_viz_3d",
        output="screen",
        parameters=[
            {
                "detections_topic": "/perception/cones_3d",
                "markers_topic": "/perception/cones_3d_markers",
            }
        ],
    )

    return LaunchDescription([marker_viz_3d])
