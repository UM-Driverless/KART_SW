from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_arg = DeclareLaunchArgument(
        "source",
        default_value="test_data/driverless_test_media/image1.png",
        description="Image, video, or directory path for replay.",
    )
    weights_arg = DeclareLaunchArgument(
        "weights",
        default_value="models/perception/yolo/ruben_yolov11n_2026_03.pt",
        description="Path to YOLO weights (.pt).",
    )

    image_source = Node(
        package="kart_perception",
        executable="image_source",
        name="image_source",
        output="screen",
        parameters=[
            {
                "source": LaunchConfiguration("source"),
                "image_topic": "/image_raw",
            }
        ],
    )

    yolo_detector = Node(
        package="kart_perception",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[
            {
                "image_topic": "/image_raw",
                "detections_topic": "/perception/cones_2d",
                "debug_image_topic": "/perception/yolo/annotated",
                "weights_path": LaunchConfiguration("weights"),
            }
        ],
    )

    marker_viz = Node(
        package="kart_perception",
        executable="cone_marker_viz",
        name="cone_marker_viz",
        output="screen",
        parameters=[
            {
                "detections_topic": "/perception/cones_2d",
                "markers_topic": "/perception/cones_markers",
                "pixel_scale": 0.01,
            }
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_camera",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "camera"],
    )

    return LaunchDescription(
        [
            source_arg,
            weights_arg,
            image_source,
            yolo_detector,
            marker_viz,
            static_tf,
        ]
    )
