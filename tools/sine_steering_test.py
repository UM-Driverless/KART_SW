#!/usr/bin/env python3
"""Send sinusoidal steering commands to test motor movement.

Usage (on Orin):
    python3 sine_steering_test.py [--amplitude 0.3] [--period 4.0] [--duration 20]

Publishes Frame messages on /orin/steering with type=0x22 (ORIN_TARG_STEERING).
The payload is int16 big-endian: target radians * 1000.

First reads the current steering angle and oscillates around it.
"""
import argparse
import math
import struct
import time

import rclpy
from rclpy.node import Node
from kb_interfaces.msg import Frame

ORIN_TARG_STEERING = 0x22


class SineSteeringTest(Node):
    def __init__(self, amplitude: float, period: float, duration: float):
        super().__init__("sine_steering_test")
        self.amplitude = amplitude
        self.period = period
        self.duration = duration

        self.pub = self.create_publisher(Frame, "/orin/steering", 10)

        # Read current steering to use as center
        self.center = None
        self.sub = self.create_subscription(
            Frame, "/esp32/steering", self._on_steering, 10
        )
        self.get_logger().info(
            f"Waiting for current steering reading... "
            f"(amplitude={amplitude:.3f} rad / {math.degrees(amplitude):.1f} deg, "
            f"period={period:.1f}s, duration={duration:.0f}s)"
        )

    def _on_steering(self, msg: Frame):
        if self.center is None and len(msg.payload) >= 2:
            raw = struct.unpack(">h", bytes(msg.payload[:2]))[0]
            self.center = raw / 1000.0
            self.get_logger().info(
                f"Current steering: {self.center:.3f} rad "
                f"({math.degrees(self.center):.1f} deg). Starting sine wave."
            )

    def send_target(self, target_rad: float):
        msg = Frame()
        msg.type = ORIN_TARG_STEERING
        raw = int(target_rad * 1000)
        raw = max(-32768, min(32767, raw))
        msg.payload = list(struct.pack(">h", raw))
        self.pub.publish(msg)

    def run(self):
        # Wait for first steering reading
        while self.center is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        t0 = time.monotonic()
        next_tick = t0

        try:
            while rclpy.ok():
                t = time.monotonic() - t0
                if t > self.duration:
                    break

                target = self.center + self.amplitude * math.sin(
                    2 * math.pi * t / self.period
                )
                self.send_target(target)

                next_tick += 0.01  # 100 Hz
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass

        # Return to center
        self.get_logger().info(f"Done. Returning to center: {self.center:.3f} rad")
        self.send_target(self.center)
        time.sleep(0.1)
        self.send_target(self.center)


def main():
    parser = argparse.ArgumentParser(description="Sinusoidal steering test")
    parser.add_argument(
        "--amplitude", type=float, default=0.3,
        help="Amplitude in radians (default: 0.3 rad = ~17 deg)",
    )
    parser.add_argument(
        "--period", type=float, default=4.0,
        help="Period of sine wave in seconds (default: 4.0)",
    )
    parser.add_argument(
        "--duration", type=float, default=20.0,
        help="Total duration in seconds (default: 20)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = SineSteeringTest(args.amplitude, args.period, args.duration)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
