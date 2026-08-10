"""Record /battery/state to CSV, to calibrate the pack's charge model.

Two numbers in kb_bms/soc_model.py are provisional generic figures rather than
measurements of the kart's own 13S4P Molicel P42A pack: the open-circuit voltage
curve (OCV_TABLE) and the per-cell internal resistance (CELL_RESISTANCE_OHM).
Until both are measured the voltage-based correction cannot be trusted, so the
fusion stays switched off and the dashboard keeps showing the raw BMS figure.

This node produces the data to replace them. Run it across one full discharge and
one full recharge:

    ros2 run kb_bms log_battery --ros-args -p path:=/tmp/battery-run.csv

Both numbers come out of the same trace.

  * The curve: take samples where the current is near zero, so terminal voltage is
    close to open circuit. Plot cell voltage against cumulative charge, normalise
    charge so the low-voltage cutoff is 0 and the charge termination is 1, and read
    off the table.
  * The resistance: find consecutive sample pairs where the current stepped sharply
    while the charge barely moved, and take dV/dI across each. Charge changes little
    over one 2.2 s poll interval, so the voltage difference across such a pair is
    almost entirely resistive.

One caveat worth knowing before reading the file: the BMS is polled over BLE at
about 0.45 Hz, so the current column is a sparse sample of a signal that moves far
faster than that. Individual current readings during driving are near meaningless
as instantaneous truth. The resting samples and the charge column are what carry
the useful information.
"""

from __future__ import annotations

import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


class BatteryLogger(Node):
    """@brief Appends every /battery/state message to a CSV file."""

    COLUMNS = [
        "wall_time",
        "elapsed_s",
        "pack_voltage_v",
        "current_a",
        "remain_ah",
        "design_capacity_ah",
        "bms_soc_percent",
        "temperature_c",
        "cell_mv",
    ]

    def __init__(self):
        super().__init__("kb_battery_logger")
        self.declare_parameter("path", "/tmp/battery-run.csv")
        path = str(self.get_parameter("path").value)

        self._start = time.time()
        self._rows = 0
        # Line buffered, so a run that ends with the kart being switched off still
        # leaves every sample up to that moment on disk.
        self._file = open(path, "a", newline="", buffering=1)
        self._writer = csv.writer(self._file)
        if self._file.tell() == 0:
            self._writer.writerow(self.COLUMNS)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(BatteryState, "/battery/state", self._on_battery, qos)
        self.get_logger().info(f"logging /battery/state to {path}")

    def _on_battery(self, msg: BatteryState):
        """@brief Write one sample. Per-cell voltages go in one space-separated field
        so the row stays a fixed width whether or not the cell read succeeded."""
        cells = " ".join(f"{round(v * 1000.0)}" for v in msg.cell_voltage)
        self._writer.writerow([
            f"{time.time():.3f}",
            f"{time.time() - self._start:.3f}",
            f"{msg.voltage:.2f}",
            f"{msg.current:.2f}",
            f"{msg.charge:.2f}",
            f"{msg.design_capacity:.2f}",
            f"{round(msg.percentage * 100.0)}",
            f"{msg.temperature:.1f}",
            cells,
        ])
        self._rows += 1
        if self._rows % 100 == 0:
            self.get_logger().info(f"{self._rows} samples written")

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
