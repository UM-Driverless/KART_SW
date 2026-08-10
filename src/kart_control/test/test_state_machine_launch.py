"""Integration test: the real state_machine node, exercised over ROS topics.

Needs ROS 2 — runs on the VM or Orin, not the Mac:
    cd ~/kart-brain && colcon build --packages-select kart_control kb_interfaces
    source install/setup.bash
    python3 -m pytest src/kart_control/test/test_state_machine_launch.py -q

Unlike test_state_logic.py (pure logic, runs anywhere), this checks the
plumbing: that the dashboard-facing topics reach the logic and that the frames
kb_coms_micro relays to the ESP32 — /kart/cmd_vel_muxed, /orin/steer_mode —
actually carry what the logic decided. The scenario is the 2026-08-10 incident:
select an autonomous mission, do NOT press Start, and verify nothing that
powers the steering motor goes out.
"""

import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="needs ROS 2 (run on the VM or Orin)")

from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from kb_interfaces.msg import Frame  # noqa: E402

NODE_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "state_machine_node.py"
STEER_MODE_PWM = 1


class Harness:
    """Publishes as the dashboard, records what the node emits."""

    def __init__(self, node):
        self.node = node
        self.mission_pub = node.create_publisher(String, "/dashboard/mission", 10)
        self.cmd_pub = node.create_publisher(String, "/dashboard/state_cmd", 10)
        self.auto_pub = node.create_publisher(Twist, "/kart/cmd_vel", 10)
        self.muxed = []
        self.steer_modes = []
        self.states = []
        node.create_subscription(Twist, "/kart/cmd_vel_muxed", self.muxed.append, 10)
        node.create_subscription(Frame, "/orin/steer_mode", self.steer_modes.append, 10)
        node.create_subscription(String, "/kart/state", self.states.append, 10)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def send(self, pub, msg_type, data):
        msg = msg_type()
        msg.data = data
        pub.publish(msg)


@pytest.fixture
def harness():
    rclpy.init()
    proc = subprocess.Popen([sys.executable, str(NODE_SCRIPT)])
    node = rclpy.create_node("state_machine_test_harness")
    h = Harness(node)
    h.spin(1.0)  # let discovery settle
    yield h
    proc.terminate()
    proc.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()


def test_mission_select_without_start_keeps_steering_unpowered(harness):
    # A live autonomous command is present the whole time — it must be gated.
    auto = Twist()
    auto.linear.x = 3.0
    auto.angular.z = 0.5
    harness.auto_pub.publish(auto)

    harness.send(harness.mission_pub, String, "autonomous")
    harness.muxed.clear()
    harness.steer_modes.clear()
    harness.spin(1.0)

    assert harness.muxed, "mux is not publishing at all"
    for twist in harness.muxed:
        assert twist.linear.x == 0.0 and twist.angular.z == 0.0
    # The 10 Hz heartbeat must be holding direct-PWM (unpowered) steer mode.
    pwm_frames = [f for f in harness.steer_modes if list(f.payload) == [STEER_MODE_PWM]]
    assert len(pwm_frames) >= 5, "heartbeat is not asserting PWM steer mode"
    assert "AS_READY" in [s.data for s in harness.states]


def test_start_passes_commands_and_mission_change_stops_them(harness):
    auto = Twist()
    auto.linear.x = 3.0
    auto.angular.z = 0.5

    harness.send(harness.mission_pub, String, "autonomous")
    harness.spin(0.3)
    harness.send(harness.cmd_pub, String, "start")
    harness.spin(0.3)
    harness.auto_pub.publish(auto)
    harness.muxed.clear()
    harness.spin(0.5)
    assert any(t.linear.x == 3.0 for t in harness.muxed), "driving does not pass cmd_vel"

    # Mission change without stop → emergency, output back to zero.
    harness.send(harness.mission_pub, String, "trackdrive")
    harness.spin(0.3)
    harness.muxed.clear()
    harness.spin(0.5)
    for twist in harness.muxed:
        assert twist.linear.x == 0.0 and twist.angular.z == 0.0
    assert "AS_EMERGENCY" in [s.data for s in harness.states]
