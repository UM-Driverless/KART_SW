"""Publish an estimated forward speed derived from cone detections.

Subscribes to the same 3D cone detections the steering controller uses, matches
cones between frames, and turns the rate their distances shrink into a speed. The
geometry and the filter live in speed_model.py; this node is only the ROS plumbing
around them.

  in:  /perception/cones_3d      (vision_msgs/Detection3DArray)
  out: /kart/speed               (std_msgs/Float32, m/s)

Nothing is published while the estimate is not backed by recent cone evidence. The
dashboard's speed readout would otherwise show a confident number sourced from
nothing, and a false reading is indistinguishable from a real one — the same
failure the steering gauge and piston readout had before 2026-07-25.

This output is unvalidated. See the accuracy note in speed_model.py before wiring
it into any controller.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from vision_msgs.msg import Detection3DArray

from kart_perception.speed_model import ConeTracker, SpeedFilter


class SpeedEstimatorNode(Node):
    """Estimate forward speed from cone range rates and publish it."""

    def __init__(self) -> None:
        super().__init__("speed_estimator")

        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("speed_topic", "/kart/speed")
        self.declare_parameter("publish_rate_hz", 20.0)
        # How hard the kart is assumed able to accelerate. Sets how quickly the
        # estimate's uncertainty grows while no cones are being matched.
        self.declare_parameter("process_accel", 3.0)
        # Above this uncertainty the estimate stops being published at all.
        self.declare_parameter("max_valid_stddev", 2.0)

        det_topic = str(self.get_parameter("detections_topic").value)
        speed_topic = str(self.get_parameter("speed_topic").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        self.tracker = ConeTracker()
        self.filter = SpeedFilter(
            process_accel=float(self.get_parameter("process_accel").value),
            max_valid_stddev=float(self.get_parameter("max_valid_stddev").value),
        )

        self._last_predict_time = self._now()
        self._frames_with_measurement = 0
        self._frames_without = 0

        self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
        self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )
        self.create_timer(1.0 / rate, self._publish_tick)
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f"speed_estimator up: {det_topic} → {speed_topic}. "
            "Output is UNVALIDATED — do not close a control loop on it yet."
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_detections(self, msg: Detection3DArray) -> None:
        """Match this frame's cones against the last and fold the result into the filter.

        Detections arrive in the camera optical frame, where z is forward and x is
        right, so the conversion to the kart's forward/left convention matches the
        one cone_follower_node.py does on the same topic.
        """
        cones = []
        for det in msg.detections:
            if not det.results:
                continue
            pos = det.results[0].pose.pose.position
            cones.append((det.results[0].hypothesis.class_id, pos.z, -pos.x))

        stamp = self._now()
        self._advance_to(stamp)

        measurement = self.tracker.update(cones, stamp)
        if measurement is None:
            self._frames_without += 1
            return
        self._frames_with_measurement += 1
        self.filter.update(measurement.speed, measurement.stddev)

    def _advance_to(self, stamp: float) -> None:
        """Run the filter's prediction step forward to a given time."""
        self.filter.predict(stamp - self._last_predict_time)
        self._last_predict_time = stamp

    def _publish_tick(self) -> None:
        """Publish the current estimate, or nothing if it is no longer evidence-backed."""
        self._advance_to(self._now())
        if not self.filter.is_valid:
            return
        self.speed_pub.publish(Float32(data=float(self.filter.speed)))

    def _log_stats(self) -> None:
        """Periodically report how often frames are actually yielding a measurement."""
        total = self._frames_with_measurement + self._frames_without
        if total == 0:
            self.get_logger().warn("speed_estimator: no cone detections arriving")
        else:
            share = 100.0 * self._frames_with_measurement / total
            self.get_logger().info(
                f"speed_estimator: {share:.0f}% of {total} frames yielded a "
                f"measurement, speed={self.filter.speed:.2f} m/s "
                f"±{self.filter.stddev:.2f} valid={self.filter.is_valid}"
            )
        self._frames_with_measurement = 0
        self._frames_without = 0


def main(args=None):
    rclpy.init(args=args)
    node = SpeedEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
