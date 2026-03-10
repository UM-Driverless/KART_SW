from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/zed/zed_node/rgb/image_rect_color",
        description="RGB image topic for YOLO inference.",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/zed/zed_node/depth/depth_registered",
        description="Aligned depth image topic.",
    )
    camera_info_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/zed/zed_node/rgb/camera_info",
        description="CameraInfo for RGB/depth alignment.",
    )
    weights_arg = DeclareLaunchArgument(
        "weights",
        default_value="models/perception/yolo/nava_yolov11_2026_02.engine",
        description="Path to YOLO weights (.pt).",
    )

    yolo_detector = Node(
        package="kart_perception",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": "/perception/cones_2d",
                "debug_image_topic": "/perception/yolo/annotated",
                "weights_path": LaunchConfiguration("weights"),
            }
        ],
    )

    cone_localizer = Node(
        package="kart_perception",
        executable="cone_depth_localizer",
        name="cone_depth_localizer",
        output="screen",
        parameters=[
            {
                "detections_topic": "/perception/cones_2d",
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "output_topic": "/perception/cones_3d",
            }
        ],
    )

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

    return LaunchDescription(
        [
            image_topic_arg,
            depth_topic_arg,
            camera_info_arg,
            weights_arg,
            yolo_detector,
            cone_localizer,
            marker_viz_3d,
        ]
    )
