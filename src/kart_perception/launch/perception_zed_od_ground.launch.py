"""Perception pipeline using ZED SDK built-in object detection, *plus* the
IMU-corrected ground-plane localizer side-by-side.

Same as ``perception_zed_od.launch.py`` but additionally launches
``ground_plane_localizer``, which republishes cones with camera pitch and roll
undone via the ZED's fused IMU orientation. Lets RViz show both
``/perception/cones_3d_markers`` (original, in optical frame) and
``/perception/cones_3d_ground_markers`` (pitch/roll-corrected) at the same
time so the correction can be visually validated at the workshop before
migrating downstream consumers.
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

    ground_plane_localizer = Node(
        package="kart_perception",
        executable="ground_plane_localizer",
        name="ground_plane_localizer",
        output="screen",
        parameters=[
            {
                "imu_topic": "/zed/zed_node/imu/data",
                "objects_topic": "/zed/zed_node/obj_det/objects",
                "output_topic": "/perception/cones_3d_ground",
                "markers_topic": "/perception/cones_3d_ground_markers",
                "imu_match_window_ms": 50,
                "invert_correction": False,
                "gravity_axis": [0.0, 0.0, 1.0],  # world up; flip if workshop test fails
                "cone_scale_m": 0.3,
            }
        ],
    )

    return LaunchDescription([marker_viz_3d, ground_plane_localizer])
