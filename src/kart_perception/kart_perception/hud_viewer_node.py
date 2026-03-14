#!/usr/bin/env python3
"""Lightweight HUD viewer using cv2.imshow instead of rqt_image_view."""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class HudViewerNode(Node):
    """@brief ROS2 node that displays HUD images in an OpenCV window for local viewing."""

    def __init__(self):
        """@brief Initialize the HUD viewer node.

        Declares the input topic parameter and subscribes to it.
        """
        super().__init__("hud_viewer")
        self.declare_parameter("topic", "/perception/hud")
        topic = str(self.get_parameter("topic").value)
        self.bridge = CvBridge()
        self.create_subscription(Image, topic, self._on_image, 1)
        self.get_logger().info(f"HUD viewer on {topic}")

    def _on_image(self, msg: Image):
        """@brief Callback that displays each received image in an OpenCV window.

        @param msg ROS Image message to display.
        """
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("HUD", img)
        cv2.waitKey(1)


def main():
    """@brief Entry point for the HUD viewer node."""
    rclpy.init()
    node = HudViewerNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
