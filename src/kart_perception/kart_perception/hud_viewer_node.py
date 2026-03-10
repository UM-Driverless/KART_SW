#!/usr/bin/env python3
"""Lightweight HUD viewer using cv2.imshow instead of rqt_image_view."""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class HudViewerNode(Node):
    def __init__(self):
        super().__init__("hud_viewer")
        self.declare_parameter("topic", "/perception/hud")
        topic = str(self.get_parameter("topic").value)
        self.bridge = CvBridge()
        self.create_subscription(Image, topic, self._on_image, 1)
        self.get_logger().info(f"HUD viewer on {topic}")

    def _on_image(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("HUD", img)
        cv2.waitKey(1)


def main():
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
