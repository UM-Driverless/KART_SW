#!/usr/bin/env python3
from typing import Tuple

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

from kart_perception.zed_od_utils import HAS_ZED_INTERFACES, zed_objects_to_det3d
if HAS_ZED_INTERFACES:
    from zed_interfaces.msg import ObjectsStamped


def class_color(class_id: str) -> Tuple[float, float, float]:
    """@brief Map a cone class name to an RGB color tuple for RViz markers.

    @param class_id Cone class name (e.g. "blue_cone", "yellow_cone").
    @return RGB color as floats in [0, 1].
    """
    if class_id == "blue_cone":
        return (0.1, 0.3, 1.0)
    if class_id == "yellow_cone":
        return (1.0, 0.9, 0.1)
    if class_id == "orange_cone":
        return (1.0, 0.5, 0.1)
    if class_id == "large_orange_cone":
        return (1.0, 0.3, 0.0)
    return (0.7, 0.7, 0.7)


class ConeMarkerViz3DNode(Node):
    """@brief ROS2 node that converts 3D cone detections into RViz MarkerArray visualizations.

    Creates colored sphere markers at 3D positions and text labels with class name
    and confidence score.
    """

    def __init__(self) -> None:
        """@brief Initialize the 3D cone marker visualization node.

        Declares parameters for input/output topics, reference frame, and cone
        marker scale in meters.
        """
        super().__init__("cone_marker_viz_3d")

        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("markers_topic", "/perception/cones_3d_markers")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("cone_scale_m", 0.3)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.markers_topic = str(self.get_parameter("markers_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.cone_scale_m = float(self.get_parameter("cone_scale_m").value)

        self.publisher = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.subscription = self.create_subscription(
            Detection3DArray, self.detections_topic, self._on_detections, 10
        )
        # Also subscribe to ZED SDK ObjectsStamped for built-in OD mode
        if HAS_ZED_INTERFACES:
            self.create_subscription(
                ObjectsStamped,
                "/zed/zed_node/obj_det/objects",
                lambda msg: self._on_detections(zed_objects_to_det3d(msg)),
                10,
            )

    def _on_detections(self, msg: Detection3DArray) -> None:
        """@brief Callback for 3D detections. Creates sphere and text markers for each cone.

        @param msg Array of 3D cone detections from the depth localizer.
        """
        markers = MarkerArray()
        header = msg.header
        if not header.frame_id:
            header.frame_id = self.frame_id

        marker_id = 0
        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            color = class_color(class_id)

            bbox = det.bbox
            marker = Marker()
            marker.header = header
            marker.ns = "cones_3d"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = bbox.center.position.x
            marker.pose.position.y = bbox.center.position.y
            marker.pose.position.z = bbox.center.position.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.cone_scale_m
            marker.scale.y = self.cone_scale_m
            marker.scale.z = self.cone_scale_m
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.9
            markers.markers.append(marker)

            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "cones_3d_labels"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = bbox.center.position.x
            text_marker.pose.position.y = bbox.center.position.y
            text_marker.pose.position.z = bbox.center.position.z + self.cone_scale_m
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = max(0.2, self.cone_scale_m * 0.6)
            text_marker.color.r = color[0]
            text_marker.color.g = color[1]
            text_marker.color.b = color[2]
            text_marker.color.a = 1.0
            text_marker.text = f"{class_id} {score:.2f}"
            markers.markers.append(text_marker)

        self.publisher.publish(markers)


def main() -> None:
    """@brief Entry point for the 3D cone marker visualization node."""
    rclpy.init()
    node = ConeMarkerViz3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
