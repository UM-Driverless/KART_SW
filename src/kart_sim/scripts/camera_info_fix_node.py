#!/usr/bin/env python3
"""Fix CameraInfo intrinsics from Gazebo Fortress.

Gazebo Fortress computes incorrect intrinsics for the RGBD camera
(wrong FX/FY and off-center CX/CY). This node subscribes to the raw
CameraInfo, recomputes the intrinsic matrix from the known HFOV and
actual image dimensions, and republishes the corrected message.

Sim-only node: not needed on real hardware where the ZED SDK provides
correct intrinsics.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoFixNode(Node):
    """@brief Fix incorrect CameraInfo intrinsics from Gazebo Fortress.

    Recomputes FX, FY, CX, CY from the known HFOV and actual image dimensions,
    overriding the K and P matrices before republishing. Sim-only node.
    """

    def __init__(self):
        """@brief Initialize with HFOV parameter, subscriber, and publisher."""
        super().__init__("camera_info_fix")

        self.declare_parameter("hfov", 1.396)  # 80 degrees in radians
        self.declare_parameter("input_topic", "/zed/zed_node/rgb/camera_info_raw")
        self.declare_parameter("output_topic", "/zed/zed_node/rgb/camera_info")

        self.hfov = float(self.get_parameter("hfov").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.pub = self.create_publisher(CameraInfo, output_topic, 10)
        self.sub = self.create_subscription(
            CameraInfo, input_topic, self._on_camera_info, 10
        )
        self._logged = False

    def _on_camera_info(self, msg: CameraInfo):
        """@brief Callback for raw CameraInfo. Recomputes intrinsics from HFOV and republishes.

        @param msg Raw CameraInfo message from the Gazebo camera sensor.
        """
        width = msg.width
        height = msg.height

        fx = width / (2.0 * math.tan(self.hfov / 2.0))
        fy = fx  # square pixels
        cx = width / 2.0
        cy = height / 2.0

        # Override K matrix [fx 0 cx; 0 fy cy; 0 0 1]
        k = list(msg.k)
        k[0] = fx
        k[2] = cx
        k[4] = fy
        k[5] = cy
        msg.k = k

        # Override P matrix [fx 0 cx Tx; 0 fy cy Ty; 0 0 1 0]
        p = list(msg.p)
        p[0] = fx
        p[2] = cx
        p[5] = fy
        p[6] = cy
        msg.p = p

        self.pub.publish(msg)

        if not self._logged:
            self.get_logger().info(
                f"CameraInfo fix: {width}x{height} HFOV={math.degrees(self.hfov):.1f}deg "
                f"-> FX={fx:.1f} FY={fy:.1f} CX={cx:.1f} CY={cy:.1f}"
            )
            self._logged = True


def main():
    """@brief Entrypoint for the CameraInfo fix node."""
    rclpy.init()
    node = CameraInfoFixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
