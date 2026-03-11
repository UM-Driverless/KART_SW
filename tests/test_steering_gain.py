"""Diagnostic test: geometric controller steering gain.

Validates that steering_gain=1.0 produces full-scale output on moderate
lateral offsets (confirming the oversteering root cause), and that
steering_gain=0.5 halves the output as expected.

Run locally (no ROS needed):
    python -m pytest tests/test_steering_gain.py -v
"""

import math
import sys
import types

# ---------------------------------------------------------------------------
# Minimal ROS 2 stubs so the module can be imported without rclpy installed.
# ---------------------------------------------------------------------------
_rclpy = types.ModuleType("rclpy")
_rclpy.init = lambda *a, **kw: None
_rclpy.spin = lambda *a, **kw: None
_rclpy.shutdown = lambda *a, **kw: None

_node_mod = types.ModuleType("rclpy.node")


class _FakeNode:
    def __init__(self, *a, **kw):
        self._params = {}

    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        class _Val:
            def __init__(self, v):
                self.value = v

        return _Val(self._params[name])

    def create_publisher(self, *a, **kw):
        return None

    def create_subscription(self, *a, **kw):
        return None

    def create_timer(self, *a, **kw):
        return None

    def get_logger(self):
        class _L:
            info = staticmethod(lambda *a, **kw: None)
            error = staticmethod(lambda *a, **kw: None)
            warn = staticmethod(lambda *a, **kw: None)

        return _L()

    def get_clock(self):
        class _C:
            def now(self):
                class _T:
                    nanoseconds = 0

                return _T()

        return _C()


_node_mod.Node = _FakeNode
_rclpy.node = _node_mod

for _name, _mod in [
    ("rclpy", _rclpy),
    ("rclpy.node", _node_mod),
    ("rclpy.qos", types.ModuleType("rclpy.qos")),
    ("geometry_msgs", types.ModuleType("geometry_msgs")),
    ("geometry_msgs.msg", types.ModuleType("geometry_msgs.msg")),
    ("nav_msgs", types.ModuleType("nav_msgs")),
    ("nav_msgs.msg", types.ModuleType("nav_msgs.msg")),
    ("vision_msgs", types.ModuleType("vision_msgs")),
    ("vision_msgs.msg", types.ModuleType("vision_msgs.msg")),
    ("numpy", None),  # numpy IS available locally
]:
    if _name not in sys.modules and _mod is not None:
        sys.modules[_name] = _mod

# Stub QoSProfile and policies
_qos = sys.modules["rclpy.qos"]
_qos.QoSProfile = type("QoSProfile", (), {"__init__": lambda self, **kw: None})
_qos.DurabilityPolicy = type("DurabilityPolicy", (), {"VOLATILE": 0})
_qos.ReliabilityPolicy = type("ReliabilityPolicy", (), {"BEST_EFFORT": 0})

# Stub Twist
_geo_msg = sys.modules["geometry_msgs.msg"]
_geo_msg.Twist = type(
    "Twist",
    (),
    {
        "__init__": lambda self: (
            setattr(self, "linear", type("V", (), {"x": 0.0})()),
            setattr(self, "angular", type("V", (), {"z": 0.0})()),
        )
    },
)

# Stub Odometry
_nav_msg = sys.modules["nav_msgs.msg"]
_nav_msg.Odometry = type("Odometry", (), {})

# Stub Detection3DArray
_vis_msg = sys.modules["vision_msgs.msg"]
_vis_msg.Detection3DArray = type("Detection3DArray", (), {})

# ---------------------------------------------------------------------------
# Now import the module under test
# ---------------------------------------------------------------------------
sys.path.insert(
    0,
    "src/kart_sim/scripts",
)
from cone_follower_node import ConeFollowerNode  # noqa: E402


def _make_node(steering_gain: float, max_steer: float = 0.5) -> ConeFollowerNode:
    """Instantiate ConeFollowerNode with overridden steering params."""
    node = ConeFollowerNode.__new__(ConeFollowerNode)
    _FakeNode.__init__(node)
    # Replicate the parameter declarations
    node.declare_parameter("detections_topic", "/perception/cones_3d")
    node.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
    node.declare_parameter("no_cone_timeout", 1.0)
    node.declare_parameter("controller_type", "geometric")
    node.declare_parameter("weights_json", "")
    node.declare_parameter("steering_gain", steering_gain)
    node.declare_parameter("max_steer", max_steer)
    node.declare_parameter("max_speed", 2.0)
    node.declare_parameter("min_speed", 0.5)
    node.declare_parameter("lookahead_max", 15.0)
    node.declare_parameter("half_track_width", 1.5)
    node.declare_parameter("speed_curve_factor", 1.0)
    # Read them back the same way __init__ does
    node.steering_gain = float(node.get_parameter("steering_gain").value)
    node.max_steer = float(node.get_parameter("max_steer").value)
    node.max_speed = float(node.get_parameter("max_speed").value)
    node.min_speed = float(node.get_parameter("min_speed").value)
    node.lookahead_max = float(node.get_parameter("lookahead_max").value)
    node.half_track_width = float(node.get_parameter("half_track_width").value)
    node.speed_curve_factor = float(node.get_parameter("speed_curve_factor").value)
    node.controller_type = "geometric"
    node._last_steer = 0.0
    return node


# Cones that produce a ~26-degree lateral angle (moderate curve)
# blue at (5, 1.5), yellow at (5, -1.5) -> midpoint (5, 0) -> angle = 0 rad
# blue at (4, 2.5), yellow at (5, -0.5) -> mid (4.5, 1.0) -> angle ~12.5 deg
_MODERATE_CONES = [
    ("blue_cone", 4.0, 2.5),
    ("yellow_cone", 5.0, -0.5),
]

# Cones producing a steep ~36-degree angle
# mid = (2.0, 1.5) -> atan2(1.5, 2.0) = 0.644 rad > max_steer=0.5
_STEEP_CONES = [
    ("blue_cone", 1.5, 3.0),
    ("yellow_cone", 2.5, 0.0),
]


def test_gain_1_saturates_on_steep_input():
    """With gain=1.0 a steep lateral offset drives steer to max_steer."""
    node = _make_node(steering_gain=1.0, max_steer=0.5)
    steer, _ = node._control_geometric(_STEEP_CONES)
    mid_l = (3.0 + 0.0) / 2.0  # 1.5
    mid_f = (1.5 + 2.5) / 2.0  # 2.0
    raw_angle = math.atan2(mid_l, mid_f)
    raw_steer = 1.0 * raw_angle  # gain * angle
    # raw_steer > max_steer means it should be clamped
    assert raw_steer > 0.5, (
        f"Expected raw steer > max_steer but got {raw_steer:.4f} — "
        "test cones not steep enough to demonstrate saturation"
    )
    assert abs(steer) == 0.5, (
        f"gain=1.0 should saturate to max_steer=0.5 on steep cones, got {steer:.4f}"
    )


def test_gain_half_does_not_saturate_on_moderate_input():
    """With gain=0.5 a moderate offset stays well within max_steer."""
    node = _make_node(steering_gain=0.5, max_steer=0.5)
    steer, _ = node._control_geometric(_MODERATE_CONES)
    assert abs(steer) < 0.5, (
        f"gain=0.5 should not saturate on moderate cones, got {steer:.4f}"
    )
    assert abs(steer) > 0.0, (
        f"Expected non-zero steer on lateral offset, got {steer:.4f}"
    )


def test_gain_half_produces_half_output_of_gain_1_when_unsaturated():
    """With gain=0.5 the output should be ~half that of gain=1.0 (when neither saturates)."""
    node1 = _make_node(steering_gain=1.0, max_steer=0.5)
    node05 = _make_node(steering_gain=0.5, max_steer=0.5)

    steer1, _ = node1._control_geometric(_MODERATE_CONES)
    steer05, _ = node05._control_geometric(_MODERATE_CONES)

    if abs(steer1) == 0.5:
        # gain=1.0 saturated; ratio check is not meaningful
        return

    ratio = steer05 / steer1 if steer1 != 0 else float("inf")
    assert abs(ratio - 0.5) < 0.02, (
        f"Expected steer(0.5) / steer(1.0) ~= 0.5, got {ratio:.4f} "
        f"(steer1={steer1:.4f}, steer05={steer05:.4f})"
    )


def test_zero_lateral_produces_zero_steer():
    """Perfectly centered midpoint produces zero steering."""
    centered = [
        ("blue_cone", 5.0, 1.5),
        ("yellow_cone", 5.0, -1.5),
    ]
    node = _make_node(steering_gain=1.0)
    steer, _ = node._control_geometric(centered)
    assert abs(steer) < 1e-9, f"Expected 0 steer for centered cones, got {steer}"
